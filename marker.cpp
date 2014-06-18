/**
 * ocv_ar - OpenCV based Augmented Reality library
 *
 * Marker class to describe single found markers in an image -- implementation file.
 *
 * Author: Markus Konrad <konrad@htw-berlin.de>, June 2014.
 * INKA Research Group, HTW Berlin - http://inka.htw-berlin.de/
 *
 * This file contains code and inspiration from ArUco library developed at the
 * Ava group of the Univeristy of Cordoba (Spain).
 * See http://sourceforge.net/projects/aruco/
 *
 * See LICENSE for license.
 */

#include "marker.h"

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

void Marker::rotatePoints(int rot) {
	rotate(points.begin(), points.begin() + 4 - rot, points.end());
}

void Marker::updatePoseMat(const cv::Mat &r, const cv::Mat &t) {
	cv::Mat cvPoseMat;
	cvPoseMat.create(4, 4, CV_32F);
	cvPoseMat.eye(4, 4, CV_32F);
    
	// r and t are double vectors from solvePnP
	// convert them!
	r.convertTo(rVec, CV_32F);
	t.convertTo(tVec, CV_32F);
    
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
    /* END modified code from ArUco lib */
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

#pragma mark private methods

void Marker::init() {
    // set defaults
    id = -1;
    
	rVec.create(3, 1, CV_32F);
	tVec.create(3, 1, CV_32F);
    
	sortPoints();
	calcShapeProperties();
}