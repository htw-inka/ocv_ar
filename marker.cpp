/**
 * ocv_ar - OpenCV based Augmented Reality library
 *
 * Marker class to describe single found markers in an image -- implementation file.
 *
 * Authors: Markus Konrad <konrad@htw-berlin.de>, Alexander Godoba, June 2014.
 * INKA Research Group, HTW Berlin - http://inka.htw-berlin.de/
 *
 * This file contains code and inspiration from ArUco library developed at the
 * Ava group of the Univeristy of Cordoba (Spain).
 * See http://sourceforge.net/projects/aruco/
 *
 * See LICENSE for license.
 */

#include <limits>

#include "marker.h"

#include "conf.h"
#include "tools.h"

using namespace ocv_ar;


#pragma mark public methods

Marker::Marker(PointVec &pts) {
    // convert points to cv::Point2f and add to the vector
	for (int i = 0; i < 4; i++) {
		points.push_back(cv::Point2f(pts[i].x, pts[i].y));
	}
    
    // common init
	init();
}


Marker::Marker(Point2fVec &pts) {
	points.assign(pts.begin(), pts.begin() + 4);
    
    // common init
	init();
}

Marker::Marker(const Marker &other) {
    setPoints(other.getPoints());
    init();
    
    id = other.getId();
    rVec = other.getRVec().clone();
    tVec = other.getTVec().clone();
//    updatePoseMat(other.getRVec(), other.getTVec());
}

Marker::~Marker() {
//    printf("ocv_ar::Marker %d (%p) - deconstructor call\n", id, this);
    
    if (rVecHist) delete [] rVecHist;
    if (tVecHist) delete [] tVecHist;
}

void Marker::rotatePoints(int rot) {
	rotate(points.begin(), points.begin() + rot + 1, points.end());
}

void Marker::updateDetectionTime() {
    detectMs = Tools::nowMs();
}

void Marker::updateForTracking(const Marker &other) {
    // copy the vertex points
    setPoints(other.getPoints());
    
    const cv::Mat r = other.getRVec().clone();
    const cv::Mat t = other.getTVec().clone();
    
    if (!r.data || !t.data) return;
    
	// r and t are double vectors from solvePnP
	// convert them to floats and save them as member
    // variables <rVec> and <tVec>
	r.convertTo(rVec, CV_32F);
	t.convertTo(tVec, CV_32F);
    
    // get pointers to the data
    float *rVecPtr = rVec.ptr<float>(0);
    float *tVecPtr = tVec.ptr<float>(0);
    
    // "rVec" is an OpenCV rotation vector -> convert it to
    // an Euler vector for interpolation
    float rVecEu[3];
    Tools::rotVecToEuler(rVecPtr, rVecEu);
    
    // push the R and T vectors to the vector history for interpolation
    pushVecsToHistory(rVecEu, tVecPtr);
    
    // if we have enough data, calculate the interpolated pose vectors
    if (pushedHistVecs >= OCV_AR_CONF_SMOOTHING_HIST_SIZE) {
        calcSmoothPoseVecs(rVecEu, tVecPtr);
    }
    
    // convert back to rotation vector and save the result inside the
    // OpenCV matrix
    Tools::eulerToRotVec(rVecEu, rVecPtr);
    
    // re-calculate the pose matrix from <rVec> and <tVec>
    calcPoseMat();
}

void Marker::updatePoseMat(const cv::Mat &r, const cv::Mat &t) {
    if (!r.data || !t.data) return;
    
	// r and t are double vectors from solvePnP
	// convert them to floats and save them as member
    // variables <rVec> and <tVec>
	r.convertTo(rVec, CV_32F);
	t.convertTo(tVec, CV_32F);
    
    // re-calculate the pose matrix from <rVec> and <tVec>
    calcPoseMat();
}

#pragma mark private methods

void Marker::init() {
    // set defaults
    id = -1;
    pushedHistVecs = 0;
    
	rVec.zeros(3, 1, CV_32F);
	tVec.zeros(3, 1, CV_32F);
    
    // create vectory history arrays
    tVecHist = new float[OCV_AR_CONF_SMOOTHING_HIST_SIZE * 3];
    rVecHist = new float[OCV_AR_CONF_SMOOTHING_HIST_SIZE * 3];
    
    // initialize them with zeros
    memset(tVecHist, 0, sizeof(float) * OCV_AR_CONF_SMOOTHING_HIST_SIZE * 3);
    memset(rVecHist, 0, sizeof(float) * OCV_AR_CONF_SMOOTHING_HIST_SIZE * 3);
    
	sortPoints();
	calcShapeProperties();
    updateDetectionTime();  // set to now
}

void Marker::sortPoints() {
	// Sort the points in anti-clockwise order
	cv::Point v1 = points[1] - points[0];
	cv::Point v2 = points[2] - points[0];
    
	// if the third point is in the left side,
	// sort in anti-clockwise order
	if ((v1.x * v2.y) - (v1.y * v2.x) < 0.0) {
		swap(points[1], points[3]);
	}
}

void Marker::calcShapeProperties() {
    // centroid is the mean of all points
	centroid = 0.25f * (points[0] + points[1] + points[2] + points[3]);
    
    // perimeter radius is the maximum distance between the centroid
    // and a corner point
	float maxDist = numeric_limits<float>::min();
	for (Point2fVec::iterator it = points.begin();
		 it != points.end();
		 ++it)
	{
		float d = cv::norm(centroid - *it);
		maxDist = max(maxDist, d);
	}
    
	perimeterRad = maxDist;
}

void Marker::pushVecsToHistory(const float *r, const float *t) {
    const int numHistElems = OCV_AR_CONF_SMOOTHING_HIST_SIZE * 3;
    
    // delete the oldest elements and move up the newer ones
    for (int i = 3; i < numHistElems; i++) {
        tVecHist[i - 3] = tVecHist[i];
        rVecHist[i - 3] = rVecHist[i];
    }

    // add the new elements to the last position
    tVecHist[numHistElems - 3] = t[0];
    tVecHist[numHistElems - 2] = t[1];
    tVecHist[numHistElems - 1] = t[2];
    
    rVecHist[numHistElems - 3] = r[0];
    rVecHist[numHistElems - 2] = r[1];
    rVecHist[numHistElems - 1] = r[2];
    
    if (pushedHistVecs < OCV_AR_CONF_SMOOTHING_HIST_SIZE) {
        pushedHistVecs++;
    }
}

void Marker::calcSmoothPoseVecs(float *r, float *t) {
    // calculate the avarage rotation angle for all axes (n)
    for (int n = 0; n < 3; n++) {
        float buff[OCV_AR_CONF_SMOOTHING_HIST_SIZE];
        for (int i = 0; i < OCV_AR_CONF_SMOOTHING_HIST_SIZE; i++) {
            buff[i] = rVecHist[i * 3 + n];
        }
        r[n] = Tools::getAverageAngle(buff, OCV_AR_CONF_SMOOTHING_HIST_SIZE);
    }
    
    // calculate the translation vector by forming the average of the former values
    t[0] = t[1] = t[2] = 0;    // reset to zeros

    for (int i = 0; i < OCV_AR_CONF_SMOOTHING_HIST_SIZE; i++) {
        t[0] += tVecHist[i * 3    ];
        t[1] += tVecHist[i * 3 + 1];
        t[2] += tVecHist[i * 3 + 2];
    }
    
    t[0] /= OCV_AR_CONF_SMOOTHING_HIST_SIZE;
    t[1] /= OCV_AR_CONF_SMOOTHING_HIST_SIZE;
    t[2] /= OCV_AR_CONF_SMOOTHING_HIST_SIZE;
}

void Marker::calcPoseMat() {
    // create rotation matrix
	cv::Mat rotMat(3, 3, CV_32FC1);
	cv::Rodrigues(rVec, rotMat);
    
	/* BEGIN modified code from ArUco lib */
    float para[3][4];
    for (int i=0; i < 3; i++) {
    	float *rotMatRow = rotMat.ptr<float>(i);
        for (int j = 0; j < 3; j++) {
        	para[i][j] = rotMatRow[j];
        }
    }
    //now, add the translation
    float *tVecData = tVec.ptr<float>(0);
    para[0][3] = tVecData[0];
    para[1][3] = tVecData[1];
    para[2][3] = tVecData[2];
    
    // create and init modelview_matrix
    memset(poseMat, 0, 16 * sizeof(float));	// init with zeros
    
    for (int i = 0; i < 3; i++) {
    	float sign = (i != 2) ? 1.0f : -1.0f;
    	for (int j = 0; j < 4; j++) {
    		poseMat[i + j * 4] = sign * para[i][j];
    	}
    }
    
    poseMat[15] = 1.0f;
    
    //    Tools::printFloatMat(poseMat, 4, 4);
    /* END modified code from ArUco lib */
}