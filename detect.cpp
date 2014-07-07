/**
 * ocv_ar - OpenCV based Augmented Reality library
 *
 * Detection core implementation file.
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

#include "detect.h"

#include "tools.h"
#include "threading.h"

using namespace ocv_ar;


#pragma mark public methods

Detect::Detect(IdentificatorType identType, float markerSizeM, FlipMode flip) {
    // use marker size default
    if (markerSizeM <= 0.0f) markerSizeM = OCV_AR_CONF_DEFAULT_MARKER_SIZE_REAL;
    
    printf("ocv_ar::Detect - projection flip mode: %d\n", (int)flip);
    
    // set defaults
    markerScale = markerSizeM;
    flipProj = flip;
    prepared = false;
    inputFrameCvtType = -1;
    outFrameProcLvl = PROC_LEVEL_DEFAULT;
    
    memset(projMat, 0, sizeof(float) * 16);
    projMatUsedSize = cv::Size(0, 0);
    
    inFrameRef = NULL;
    inFrameOrigGray = NULL;
    inFrame = NULL;
    procFrame= NULL;
    outFrame = NULL;
    
    inputFrameW = inputFrameH = 0;
    downsampleSizeW = downsampleSizeH = 0;
    
    ident = NULL;
    
    // set identificator
    setIdentificatorType(identType);
    
#if !defined(OCV_AR_CONF_DOWNSAMPLE) && defined(OCV_AR_CONF_RESIZE_W) && defined(OCV_AR_CONF_RESIZE_H)
    downsampleSizeW = OCV_AR_CONF_RESIZE_W;
    downsampleSizeH = OCV_AR_CONF_RESIZE_H;
#endif
}

Detect::~Detect() {
    // delete allocated memory
    
    if (inFrameOrigGray) delete inFrameOrigGray;
    if (procFrame) delete procFrame;
    if (outFrame) delete outFrame;
    if (ident) delete ident;
}

void Detect::setIdentificatorType(IdentificatorType identType) {
    if (ident) delete ident;
    
    printf("ocv_ar::Detect - loading identificator type %d\n", identType);
    
    // create an identificator object
    switch (identType) {
        case IDENT_TYPE_CODE_7X7:
            ident = new Identificator7x7();
            printf("ocv_ar::Detect - identificator is set to '7 bit code'\n");
            break;

        case IDENT_TYPE_TEMPL_MATCH:
            ident = new IdentificatorTemplMatch();
            printf("ocv_ar::Detect - identificator is set to 'template matching'\n");
            break;
            
        default:
            ident = NULL;
            break;
    }
    
    // set normalized marker coordinates for 2D and 3D space
    normMarkerCoord2D.clear();
    normMarkerCoord3D.clear();
    
    if (ident) {
        normMarkerSize = ident->getRequiredMarkerSize();
        
        const int s = normMarkerSize - 1;
        normMarkerCoord2D.push_back(cv::Point2f(0, 0));
        normMarkerCoord2D.push_back(cv::Point2f(s, 0));
        normMarkerCoord2D.push_back(cv::Point2f(s, s));
        normMarkerCoord2D.push_back(cv::Point2f(0, s));
        
        const float sm = markerScale;	// size in meters
        normMarkerCoord3D.push_back(cv::Point3f(-0.5f * sm,  0.5f * sm, 0.0f));
        normMarkerCoord3D.push_back(cv::Point3f( 0.5f * sm,  0.5f * sm, 0.0f));
        normMarkerCoord3D.push_back(cv::Point3f( 0.5f * sm, -0.5f * sm, 0.0f));
        normMarkerCoord3D.push_back(cv::Point3f(-0.5f * sm, -0.5f * sm, 0.0f));
    } else {
        normMarkerSize = 0;
    }
}

void Detect::prepare(int frameW, int frameH, int frameChan, int cvtType) {
    assert(frameW > 0 && frameH > 0 && (frameChan == 1 || frameChan == 3 || frameChan == 4));
    
    inputFrameW = frameW;
    inputFrameH = frameH;
    
    // alloc mem for orig. sized image in grayscale format
    if (inFrameOrigGray) delete inFrameOrigGray;
    inFrameOrigGray = new cv::Mat(frameH, frameW, CV_8UC1);
    
    // alloc mem for downsampled images in grayscale format
    if (inFrame) {
        delete inFrame;
        inFrame = NULL;
    }
    if (procFrame) {
        delete procFrame;
        procFrame = NULL;
    }
    
#ifdef OCV_AR_CONF_DOWNSAMPLE
    int frac = pow(2.0f, (float)OCV_AR_CONF_DOWNSAMPLE);
    downsampleSizeW = frameW / frac;
    downsampleSizeH = frameH / frac;
#elif defined(OCV_AR_CONF_RESIZE_W) && defined(OCV_AR_CONF_RESIZE_H)
    assert(downsampleSizeW <= frameW && downsampleSizeH <= frameH);
#else
#error Either OCV_AR_CONF_DOWNSAMPLE or OCV_AR_CONF_RESIZE_W/H must be defined.
#endif
    
    inFrame = new cv::Mat(downsampleSizeH, downsampleSizeW, CV_8UC1);
    procFrame = new cv::Mat(downsampleSizeH, downsampleSizeW, CV_8UC1);
    
    if (cvtType < 0) {  // guess color convert type
        if (frameChan == 3) {
            cvtType = CV_RGB2GRAY;
        } else if (frameChan == 4) {
            cvtType = CV_RGBA2GRAY;
        }
    }
    
    if (frameChan == 1) {
        cvtType = -1;   // means we don't need to call cv::cvtColor
    }
    
    // set properties
    inputFrameCvtType = cvtType;
    prepared = true;
    
    printf("ocv_ar::Detect - prepared for frames: %dx%d (%d channels)\n", frameW, frameH, frameChan);
    printf("ocv_ar::Detect - will downscale to: %dx%d\n", downsampleSizeW, downsampleSizeH);
    printf("ocv_ar::Detect - will use convert mode: %d\n", inputFrameCvtType);
}

void Detect::setCamIntrinsics(const cv::Mat &cam, const cv::Mat &dist) {
    camMat = cam;
    distCoeff = dist;   // this mat can also be empty
}

float *Detect::getProjMat(float viewW, float viewH) {
    assert(!camMat.empty() && viewW > 0.0f && viewH > 0.0f && prepared);
    
    // re-calculate the projection matrix if necessary
    if (viewW != projMatUsedSize.width || viewH != projMatUsedSize.height) {
        calcProjMat(viewW, viewH);
    }
    
    return projMat;
}

void Detect::setFrameOutputLevel(FrameProcLevel level) {
    assert(prepared);
    
    if (outFrameProcLvl == level) return; // no change, no op.
    
    outFrameProcLvl = level;
    
    int outW, outH;
    
    outW = inFrame->cols;
    outH = inFrame->rows;
    
    if (outFrame) {
        if (outW == outFrame->cols && outH == outFrame->rows) return;   // no change in output frame size
        
        delete outFrame;
    }
    
    // allocate memory for the output frame
    outFrame = new cv::Mat(outH, outW, CV_8UC1);
    
    printf("ocv_ar::Detect - set output frame level: %d (output frame size %dx%d)", level, outW, outH);
}

void Detect::setInputFrame(const cv::Mat *frame, bool doNotCopyGrayscaleImg) {
    assert(prepared && frame);
    
    if (inputFrameCvtType >= 0) {   // convert to grayscale (will copy the frame!)
        cv::cvtColor(*frame, *inFrameOrigGray, inputFrameCvtType);
    } else {
        if (doNotCopyGrayscaleImg) {
            inFrameRef = frame;                 // only assign the frame pointer
            inFrameOrigGray = NULL;
        } else {
            frame->copyTo(*inFrameOrigGray);    // copy the frame
            inFrameRef = NULL;
        }
    }
}

void Detect::processFrame() {
    // defines the whole marker detection pipeline WITHOUT estimating the
    // 3D pose of a marker. you need to call <estimateMarkersPoses()> for
    // this afterwards
    
    preprocess();
    performThreshold();
    findContours();
    findMarkerCandidates();
    identifyMarkers();
}

void Detect::estimateMarkersPoses() {
    // estimate the 3D pose of each found marker
    for (vector<Marker *>::iterator it = foundMarkers.begin();
         it != foundMarkers.end();
         ++it)
    {
        Marker *marker = *it;
        
        // find marker pose from 3D-2D point correspondences between <normMarkerCoord3D>
        // and 2D points in <marker->getPoints()>
		cv::Mat rVec;   // pose rotation vector
		cv::Mat tVec;   // pose translation vector
		cv::solvePnP(normMarkerCoord3D, marker->getPoints(),
					 camMat, distCoeff,
					 rVec, tVec,
					 false);
        
        // generate an OpenGL model-view matrix from the rotation and translation vectors
        marker->updatePoseMat(rVec, tVec);
    }
}

cv::Mat *Detect::getOutputFrame() const {
    if (outFrameProcLvl == PROC_LEVEL_DEFAULT) return NULL;

    return outFrame;
}

#pragma mark private methods

void Detect::preprocess() {
    // downscale the image
    
#ifdef OCV_AR_CONF_DOWNSAMPLE
    if (inFrameRef) {
        cv::pyrDown(*inFrameRef, *inFrame);
    } else {
        for (int i = 0; i < OCV_AR_CONF_DOWNSAMPLE; i++) {
            cv::pyrDown(*inFrameOrigGray, *inFrame);
            
            inFrameOrigGray = inFrame;
        }
    }
#elif defined(OCV_AR_CONF_RESIZE_W) && defined(OCV_AR_CONF_RESIZE_H)
    cv::Mat *framePtr = inFrameRef ? inFrameRef : inFrameOrigGray;
    cv::resize(*inFrameOrigGray, *inFrame, cv::Size(OCV_AR_CONF_RESIZE_W, OCV_AR_CONF_RESIZE_H));
#else
#error Either OCV_AR_CONF_DOWNSAMPLE or OCV_AR_CONF_RESIZE_W/H must be defined.
#endif
    
    setOutputFrameOnCurProcLevel(PROC_LEVEL_PREPROC, inFrame);
}

void Detect::performThreshold() {
    // perform thresholding
    
	cv::adaptiveThreshold(*inFrame,
                          *procFrame,
                          255,
                          cv::ADAPTIVE_THRESH_MEAN_C, // ADAPTIVE_THRESH_GAUSSIAN_C
                          cv::THRESH_BINARY_INV,
                          OCV_AR_CONF_THRESH_BLOCK_SIZE,
                          OCV_AR_CONF_THRESH_C);
    
    setOutputFrameOnCurProcLevel(PROC_LEVEL_THRESH, procFrame);
}

//void Detect::threshPostProc() {
//    
//}

void Detect::findContours() {
	// find contours
	ContourVec allContours;
	cv::findContours(*procFrame, allContours,
                     OCV_AR_CONF_FIND_CONTOUR_TYPE,
                     CV_CHAIN_APPROX_SIMPLE);   // CV_CHAIN_APPROX_NONE or CV_CHAIN_APPROX_SIMPLE

//    printf("ocv_ar::Detect - num contours: %lu\n", allContours.size());
    
	// filter out contours consisting of
	// less than <minContourPointsAllowed> points
	curContours.clear();
	for (ContourVec::const_iterator it = allContours.begin();
         it != allContours.end();
         ++it)
	{
        // add this contour to our <curContour> vector in case it could form a marker
		if (it->size() >= OCV_AR_CONF_MIN_CONTOUR_PTS) {
			curContours.push_back(*it);
		}
	}
    
	// draw contours if necessary
	if (outFrameProcLvl == PROC_LEVEL_CONTOURS) {
        //		LOGINFO("Num. contours: %d", curContours.size());
		outFrame->setTo(cv::Scalar(0, 0, 0, 255));	// clear: fill black
		cv::drawContours(*outFrame, curContours, -1, cv::Scalar(255, 255, 255, 255));
	}
}

void Detect::findMarkerCandidates() {
    const float minContourLengthAllowed = OCV_AR_CONF_MIN_CONTOUR_LENGTH * OCV_AR_CONF_MIN_CONTOUR_LENGTH;
    
    // tabula rasa for the new frame
    foundMarkers.clear();
	possibleMarkers.clear();
	PointVec  approxCurve;
    
    // analyze each contour
	for (ContourVec::const_iterator it = curContours.begin();
         it != curContours.end();
         ++it)
	{
		PointVec contour = *it;
		// Approximate to a polygon
		float eps = contour.size() * 0.05f;
		cv::approxPolyDP(contour, approxCurve, eps, true);
        
		// we are only interested in convex polygons that contain exactly four points
		if (approxCurve.size() != 4 || !cv::isContourConvex(approxCurve)) continue;

		// Ensure that the distance between consecutive points is large enough
		float minDist = numeric_limits<float>::max();
		for (int i = 0; i < 4; i++) {
			cv::Point side = approxCurve[i] - approxCurve[(i+1)%4];
			float squaredSideLength = side.dot(side);
			minDist = min(minDist, squaredSideLength);
		}
        
		if (minDist < minContourLengthAllowed) continue;
        
		// Create new marker candidate
		// Fill it with the points of the curve
		// The Marker constructor will also sort the points in anti-clockwise order
		Marker markerCand(approxCurve);
        
		// Add the marker candidate
		possibleMarkers.push_back(markerCand);
    }

//    printf("ocv_ar::Detect - Num. marker candidates: %lu\n", possibleMarkers.size());
    
    // duplicate markers are possible, especially when double edges are detected
    // filter them out
    discardDuplicateMarkers(possibleMarkers);
    
//    printf("ocv_ar::Detect - Num. marker candidates without duplicates: %lu\n", possibleMarkers.size());
    
	// draw markers if necessary
	if (outFrameProcLvl == PROC_LEVEL_POSS_MARKERS) {
		procFrame->copyTo(*outFrame);
        //		outFrame->setTo(cv::Scalar(0, 0, 0, 255));	// clear: fill black
        
		// draw each marker candidate
		for (vector<Marker>::iterator it = possibleMarkers.begin();
             it != possibleMarkers.end();
             ++it)
		{
			drawMarker(*outFrame, *it, false);
		}
	}
}

void Detect::identifyMarkers() {
    if (!ident) return;
    
	if (outFrame && outFrameProcLvl == PROC_LEVEL_DETECTED_MARKERS) {
        //		outFrame->setTo(cv::Scalar(0, 0, 0, 255));	// clear: fill black
		inFrame->copyTo(*outFrame);
	}

//    foundMarkers.clear(); // this is done in findMarkerCandidates()
    
    // normalize (deskew) all possible markers and identify them
    for (vector<Marker>::iterator it = possibleMarkers.begin();
         it != possibleMarkers.end();
         ++it)
	{
        cv::Mat normMarkerImg(normMarkerSize, normMarkerSize, CV_8UC1);
        
		// Find the perspective transformation that brings current marker to
		// rectangular form
		const cv::Mat perspMat = cv::getPerspectiveTransform(it->getPoints(), normMarkerCoord2D);
		cv::warpPerspective(*inFrame, normMarkerImg,  perspMat, cv::Size(normMarkerSize, normMarkerSize), cv::INTER_NEAREST);
		cv::threshold(normMarkerImg, normMarkerImg, 125, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        // try to read the marker code
        // if this is successfull, it will be saved as the marker's ID
        // and the marker's corner points will be correctly rotated
        if (ident->readMarkerCode(normMarkerImg, *it)) {
            // a valid code could be read -> add this marker to the "found markers"
            foundMarkers.push_back(&(*it));
            
//            printf("ocv_ar::Detect - found valid marker with id %d\n", it->getId());
            
            // refine corners
#if OCV_AR_CONF_REFINE_CORNERS_ITER > 0
            cv::cornerSubPix(*inFrame, it->getPoints(),
                             cv::Size(5, 5), cv::Size(-1,-1),
                             cv::TermCriteria(CV_TERMCRIT_ITER, OCV_AR_CONF_REFINE_CORNERS_ITER, 0.1f));	// max. iterations, min. epsilon
#endif
            // draw marker
            if (outFrame && outFrameProcLvl == PROC_LEVEL_DETECTED_MARKERS) {
                float r = it->getPerimeterRadius();
                cv::Point o = it->getCentroid() - (0.5f * cv::Point2f(r, r));
                cv::Rect roi(o, normMarkerImg.size());
                cv::rectangle(*outFrame, roi, cv::Scalar(255,255,255,255));
                
                if (roi.x + roi.width > outFrame->cols) {
                    roi.width = roi.x + roi.width - outFrame->cols;
                }
                
                if (roi.y + roi.height > outFrame->rows) {
                    roi.height = roi.y + roi.height - outFrame->rows;
                }
                
                //                printf("ocv_ar::Detect - drawing marker with id %d at pos %d, %d with ROI at %d, %d (%d x %d)\n",
                //                       it->getId(), o.x, o.y, roi.x, roi.y, roi.width, roi.height);
                
                cv::Mat dstMat = (*outFrame)(roi);
                normMarkerImg.copyTo(dstMat);
                
                drawMarker(*outFrame, *it, true);
            }
        }
    }
}

void Detect::discardDuplicateMarkers(vector<Marker> &markerList) {
    const float maxDuplDistSquared = OCV_AR_CONF_MAX_DUPLICATE_DIST * OCV_AR_CONF_MAX_DUPLICATE_DIST;
    
    vector<Marker>::iterator toDel = markerList.end();  // iterator for elem. that will be removed
    for (vector<Marker>::iterator cur = markerList.begin();
         cur != markerList.end();)
    {
        for (vector<Marker>::iterator other = markerList.begin();
             other != markerList.end();
             ++other)
        {
            if (cur == other) continue;
            
            const float dist = Tools::distSquared(cur->getCentroid(), other->getCentroid());
            
            // mark for deletion if the distance is close and the current marker is bigger
            if (dist <= maxDuplDistSquared && cur->getPerimeterRadius() < other->getPerimeterRadius()) {
//                printf("ocv_ar::Detect - will remove duplicate! dist = %f, r1 = %f, r2 = %f \n",
//                       dist, cur->getPerimeterRadius(), other->getPerimeterRadius());
                
                toDel = cur;
                break;
            }
        }
        
        if (toDel != markerList.end()) {
            cur = markerList.erase(toDel);  // advances the iterator
            toDel = markerList.end();       // reset
        } else {
            ++cur;
        }
    }
}

void Detect::setOutputFrameOnCurProcLevel(FrameProcLevel curLvl, cv::Mat *srcFrame) {
    assert(srcFrame);
    
    if (curLvl == outFrameProcLvl) {
        srcFrame->copyTo(*outFrame);
    }
}

void Detect::drawMarker(cv::Mat &img, const Marker &m, bool drawId) {
	cv::Scalar p1clr(255, 255, 255, 255);
	cv::Scalar p2clr(191, 191, 191, 255);
	cv::Scalar p3clr(128, 128, 128, 255);
    cv::Scalar p4clr( 64,  64,  64, 255);
    
	cv::Scalar white(255, 255, 255, 255);
	cv::Scalar blue(0, 0, 255, 255);
    cv::Scalar green(0, 255, 0, 255);
    
	// draw outline
	Point2fVec markerPts = m.getPoints();
	const int numPts = (int)markerPts.size();
	for (int i = 0; i < numPts; i++) {
		cv::line(img, markerPts[i], markerPts[(i + 1) % numPts], white);
        
        cv::Point2f vertex = markerPts[i];
        cv::Point vertex1 = cv::Point(vertex.x - 5, vertex.y - 5);
        cv::Point vertex2 = cv::Point(vertex.x + 5, vertex.y + 5);
        cv::Scalar vertexClr(255 / (i + 1), 255 / (i + 1), 255 / (i + 1), 255);
        cv::rectangle(img, vertex1, vertex2, vertexClr);
	}
    
	// draw centroid
	cv::Point cross1(2, 2);
	cv::Point cross2(2, -2);
	cv::Point c = m.getCentroid();
	cv::line(img, c - cross1, c + cross1, green);
	cv::line(img, c + cross2, c - cross2, green);
    
    // draw id
    if (drawId) {
        stringstream idStr;
        idStr << m.getId();
        cv::putText(img, idStr.str(), c + cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 1.0, white);
    }
    
	// draw perimeter
	cv::circle(img, c, m.getPerimeterRadius(), blue);
}

void Detect::calcProjMat(float viewW, float viewH) {
    printf("ocv_ar::Detect - calculating projection matrix for view size %dx%d\n",
           (int)viewW, (int)viewH);
    
    const float projNear = OCV_AR_CONF_PROJMAT_NEAR_PLANE;
	const float projFar  = OCV_AR_CONF_PROJMAT_FAR_PLANE;
    
    projMatUsedSize = cv::Size(viewW, viewH);
    
	// intrinsics mat contains doubles. we need floats
	cv::Mat intrFloats(3, 3, CV_32F);
	camMat.convertTo(intrFloats, CV_32F);
    
	// get cam parameters
	/* BEGIN modified code from ArUco lib */
    const float Ax = viewW / (float)downsampleSizeW;
    const float Ay = viewH / (float)downsampleSizeH;
	const float f_x = intrFloats.at<float>(0, 0) * Ax;	// Focal length in x axis
	const float f_y = intrFloats.at<float>(1, 1) * Ay;	// Focal length in y axis
	const float c_x = intrFloats.at<float>(0, 2) * Ax; 	// Camera primary point x
	const float c_y = intrFloats.at<float>(1, 2) * Ay;	// Camera primary point y
    
    float cparam[3][4] =
    {
        {f_x,   0,  c_x, 0},
        {  0, f_y,  c_y, 0},
        {  0,   0,    1, 0}
    };

    cparam[0][2] *= -1.0;
    cparam[1][2] *= -1.0;
    cparam[2][2] *= -1.0;
    
    float   icpara[3][4];
    float   trans[3][4];
    float   p[3][3], q[4][4];
    
    Tools::arParamDecompMat(cparam, icpara, trans);
    
    for (int i = 0; i < 3; i++ )
    {
        for (int j = 0; j < 3; j++ )
        {
            p[i][j] = icpara[i][j] / icpara[2][2];
        }
    }
    
    q[0][0] = (2.0 * p[0][0] / viewW);
    q[0][1] = (2.0 * p[0][1] / viewW);
    q[0][2] = ((2.0 * p[0][2] / viewW)  - 1.0);
    q[0][3] = 0.0;
    
    q[1][0] = 0.0;
    q[1][1] = (2.0 * p[1][1] / viewH);
    q[1][2] = ((2.0 * p[1][2] / viewH) - 1.0);
    q[1][3] = 0.0;
    
    q[2][0] = 0.0;
    q[2][1] = 0.0;
    q[2][2] = (projFar + projNear)/(projFar - projNear);
    q[2][3] = -2.0 * projFar * projNear / (projFar - projNear);
    
    q[3][0] = 0.0;
    q[3][1] = 0.0;
    q[3][2] = 1.0;
    q[3][3] = 0.0;
    
    for (int i = 0; i < 4; i++ )
    {
        for (int j = 0; j < 3; j++ )
        {
            projMat[i+j*4] = q[i][0] * trans[0][j]
            + q[i][1] * trans[1][j]
            + q[i][2] * trans[2][j];
        }
        projMat[i+3*4] = q[i][0] * trans[0][3]
        + q[i][1] * trans[1][3]
        + q[i][2] * trans[2][3]
        + q[i][3];
    }
    
    if (flipProj == FLIP_H) {
        projMat[1]  = -projMat[1];
        projMat[5]  = -projMat[5];
        projMat[9]  = -projMat[9];
        projMat[13] = -projMat[13];
    } else if (flipProj == FLIP_V) {
        projMat[0] = -projMat[0];
        projMat[4]  = -projMat[4];
        projMat[8]  = -projMat[8];
        projMat[12]  = -projMat[12];
    }

    /* END modified code from ArUco lib */
}