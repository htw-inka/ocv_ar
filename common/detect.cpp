#include "detect.h"

#include "tools.h"

using namespace ocv_ar;

/* BEGIN code from ArUco lib */

static float norm( float a, float b, float c )
{
    return( sqrt( a*a + b*b + c*c ) );
}

static float dot( float a1, float a2, float a3,
                 float b1, float b2, float b3 )
{
    return( a1 * b1 + a2 * b2 + a3 * b3 );
}

static int arParamDecompMat( float source[3][4], float cpara[3][4], float trans[3][4] )
{
    int       r, c;
    float    Cpara[3][4];
    float    rem1, rem2, rem3;
    
    if ( source[2][3] >= 0 )
    {
        for ( r = 0; r < 3; r++ )
        {
            for ( c = 0; c < 4; c++ )
            {
                Cpara[r][c] = source[r][c];
            }
        }
    }
    else
    {
        for ( r = 0; r < 3; r++ )
        {
            for ( c = 0; c < 4; c++ )
            {
                Cpara[r][c] = -(source[r][c]);
            }
        }
    }
    
    for ( r = 0; r < 3; r++ )
    {
        for ( c = 0; c < 4; c++ )
        {
            cpara[r][c] = 0.0;
        }
    }
    cpara[2][2] = norm( Cpara[2][0], Cpara[2][1], Cpara[2][2] );
    trans[2][0] = Cpara[2][0] / cpara[2][2];
    trans[2][1] = Cpara[2][1] / cpara[2][2];
    trans[2][2] = Cpara[2][2] / cpara[2][2];
    trans[2][3] = Cpara[2][3] / cpara[2][2];
    
    cpara[1][2] = dot( trans[2][0], trans[2][1], trans[2][2],
                      Cpara[1][0], Cpara[1][1], Cpara[1][2] );
    rem1 = Cpara[1][0] - cpara[1][2] * trans[2][0];
    rem2 = Cpara[1][1] - cpara[1][2] * trans[2][1];
    rem3 = Cpara[1][2] - cpara[1][2] * trans[2][2];
    cpara[1][1] = norm( rem1, rem2, rem3 );
    trans[1][0] = rem1 / cpara[1][1];
    trans[1][1] = rem2 / cpara[1][1];
    trans[1][2] = rem3 / cpara[1][1];
    
    cpara[0][2] = dot( trans[2][0], trans[2][1], trans[2][2],
                      Cpara[0][0], Cpara[0][1], Cpara[0][2] );
    cpara[0][1] = dot( trans[1][0], trans[1][1], trans[1][2],
                      Cpara[0][0], Cpara[0][1], Cpara[0][2] );
    rem1 = Cpara[0][0] - cpara[0][1]*trans[1][0] - cpara[0][2]*trans[2][0];
    rem2 = Cpara[0][1] - cpara[0][1]*trans[1][1] - cpara[0][2]*trans[2][1];
    rem3 = Cpara[0][2] - cpara[0][1]*trans[1][2] - cpara[0][2]*trans[2][2];
    cpara[0][0] = norm( rem1, rem2, rem3 );
    trans[0][0] = rem1 / cpara[0][0];
    trans[0][1] = rem2 / cpara[0][0];
    trans[0][2] = rem3 / cpara[0][0];
    
    trans[1][3] = (Cpara[1][3] - cpara[1][2]*trans[2][3]) / cpara[1][1];
    trans[0][3] = (Cpara[0][3] - cpara[0][1]*trans[1][3]
                   - cpara[0][2]*trans[2][3]) / cpara[0][0];
    
    for ( r = 0; r < 3; r++ )
    {
        for ( c = 0; c < 3; c++ )
        {
            cpara[r][c] /= cpara[2][2];
        }
    }
    
    return 0;
}
/* END code from ArUco lib */


#pragma mark public methods

Detect::Detect(IdentificatorType identType, float markerSizeM) {
    if (markerSizeM <= 0.0f) markerSizeM = OCV_AR_CONF_DEFAULT_MARKER_SIZE_REAL;
    
    markerScale = markerSizeM;
    prepared = false;
    inputFrameCvtType = -1;
    outFrameProcLvl = DEFAULT;
    
    memset(projMat, 0, sizeof(float) * 16);
    projMatUsedSize = cv::Size(0, 0);
    
    inFrameOrigGray = NULL;
    inFrame = NULL;
    procFrame= NULL;
    outFrame = NULL;
    
    inputFrameW = inputFrameH = 0;
    downsampleSizeW = downsampleSizeH = 0;
    
    ident = NULL;
    setIdentificator(identType);
    
#if !defined(OCV_AR_CONF_DOWNSAMPLE) && defined(OCV_AR_CONF_RESIZE_W) && defined(OCV_AR_CONF_RESIZE_H)
    downsampleSizeW = OCV_AR_CONF_RESIZE_W;
    downsampleSizeH = OCV_AR_CONF_RESIZE_H;
#endif
}

Detect::~Detect() {
    if (inFrameOrigGray) delete inFrameOrigGray;
    if (inFrame) delete inFrame;
    if (procFrame) delete procFrame;
    if (outFrame) delete outFrame;
    if (ident) delete ident;
}

void Detect::setIdentificator(IdentificatorType identType) {
    if (ident) delete ident;
    
    switch (identType) {
        case CODE_7BIT:
            ident = new Identificator7BitCode();
            break;
            
        default:
            ident = NULL;
            break;
    }
    
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
    int frac = pow(2, OCV_AR_CONF_DOWNSAMPLE);
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
}

void Detect::setCamIntrinsics(const cv::Mat &cam, const cv::Mat &dist) {
    camMat = cam;
    distCoeff = dist;   // this mat can also be empty
}

float *Detect::getProjMat(float viewW, float viewH) {
    assert(!camMat.empty() && viewW > 0.0f && viewH > 0.0f && prepared);
    
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
    
    outFrame = new cv::Mat(outH, outW, CV_8UC1);
    
    printf("ocv_ar::Detect - set output frame level: %d (output frame size %dx%d)", level, outW, outH);
}

void Detect::setInputFrame(cv::Mat *frame) {
    assert(prepared && frame);
    
    if (inputFrameCvtType >= 0) {   // convert to grayscale
        cv::cvtColor(*frame, *inFrameOrigGray, inputFrameCvtType);
    }
}

void Detect::processFrame() {
    preprocess();
    performThreshold();
    findContours();
    findMarkerCandidates();
    identifyMarkers();
    estimatePositions();
}

cv::Mat *Detect::getOutputFrame() const {
    if (outFrameProcLvl == DEFAULT) return NULL;

    return outFrame;
}

#pragma mark private methods

void Detect::preprocess() {
#ifdef OCV_AR_CONF_DOWNSAMPLE
    for (int i = 0; i < OCV_AR_CONF_DOWNSAMPLE; i++) {
        cv::pyrDown(*inFrameOrigGray, *inFrame);
        
        inFrameOrigGray = inFrame;
    }
#elif defined(OCV_AR_CONF_RESIZE_W) && defined(OCV_AR_CONF_RESIZE_H)
    cv::resize(*inFrameOrigGray, *inFrame, cv::Size(OCV_AR_CONF_RESIZE_W, OCV_AR_CONF_RESIZE_H));
#else
#error Either OCV_AR_CONF_DOWNSAMPLE or OCV_AR_CONF_RESIZE_W/H must be defined.
#endif
    
    setOutputFrameOnCurProcLevel(PREPROC, inFrame);
}

void Detect::performThreshold() {
	cv::adaptiveThreshold(*inFrame,
                          *procFrame,
                          255,
                          cv::ADAPTIVE_THRESH_MEAN_C, // ADAPTIVE_THRESH_GAUSSIAN_C
                          cv::THRESH_BINARY_INV,
                          OCV_AR_CONF_THRESH_BLOCK_SIZE,
                          OCV_AR_CONF_THRESH_C);
    
    setOutputFrameOnCurProcLevel(THRESH, procFrame);
}

//void Detect::threshPostProc() {
//    
//}

void Detect::findContours() {
	// find contours
	ContourVec allContours;
	cv::findContours(*procFrame, allContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE); // CV_RETR_LIST or CV_RETR_EXTERNAL

	// filter out contours consisting of
	// less than <minContourPointsAllowed> points
	curContours.clear();
	for (ContourVec::const_iterator it = allContours.begin();
         it != allContours.end();
         ++it)
	{
		if (it->size() >= OCV_AR_CONF_MIN_CONTOUR_PTS) {
			curContours.push_back(*it);
		}
	}
    
	// draw contours if necessary
	if (outFrameProcLvl == CONTOURS) {
        //		LOGINFO("Num. contours: %d", curContours.size());
		outFrame->setTo(cv::Scalar(0, 0, 0, 255));	// clear: fill black
		cv::drawContours(*outFrame, curContours, -1, cv::Scalar(255, 255, 255, 255));
	}
}

void Detect::findMarkerCandidates() {
    const float minContourLengthAllowed = OCV_AR_CONF_MIN_CONTOUR_LENGTH * OCV_AR_CONF_MIN_CONTOUR_LENGTH;
    
    foundMarkers.clear();
	possibleMarkers.clear();
	PointVec  approxCurve;
    
	for (ContourVec::const_iterator it = curContours.begin();
         it != curContours.end();
         ++it)
	{
		PointVec contour = *it;
		// Approximate to a polygon
		float eps = contour.size() * 0.05f;
		cv::approxPolyDP(contour, approxCurve, eps, true);
        
		// We interested only in polygons that contains only four points
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
    
    discardDuplicateMarkers(possibleMarkers);
    
//    printf("ocv_ar::Detect - Num. marker candidates without duplicates: %lu\n", possibleMarkers.size());
    
	// draw markers if necessary
	if (outFrameProcLvl == POSS_MARKERS) {
		procFrame->copyTo(*outFrame);
        //		outFrame->setTo(cv::Scalar(0, 0, 0, 255));	// clear: fill black
        
		// draw each marker candidate
		for (vector<Marker>::iterator it = possibleMarkers.begin();
             it != possibleMarkers.end();
             ++it)
		{
			drawMarker(*outFrame, *it);
		}
	}
}

void Detect::identifyMarkers() {
    if (!ident) return;
    
	if (outFrame && outFrameProcLvl == DETECTED_MARKERS) {
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
            if (outFrame && outFrameProcLvl == DETECTED_MARKERS) {
                float r = it->getPerimeterRadius();
                cv::Point o = it->getCentroid() - (0.5f * cv::Point2f(r, r));
//                printf("ocv_ar::Detect - drawing marker with id %d at pos %d, %d\n", it->getId(), o.x, o.y);
                cv::Rect roi(o, normMarkerImg.size());
                cv::rectangle(*outFrame, roi, cv::Scalar(255,255,255,255));
                cv::Mat dstMat = (*outFrame)(roi);
                normMarkerImg.copyTo(dstMat);
                
                drawMarker(*outFrame, *it);
                
                //			LOGINFO("pers. mat.:");
                //			for (int i = 0; i < perspMat.rows; i++) {
                //				LOGINFO("%f\t%f\t%f", perspMat.at<float>(i, 0), perspMat.at<float>(i, 1), perspMat.at<float>(i, 2));
                //			}
            }
        }
    }
}

void Detect::estimatePositions() {
    for (vector<Marker *>::iterator it = foundMarkers.begin(); it != foundMarkers.end(); ++it) {
        Marker *marker = *it;
        
		cv::Mat rVec;
		cv::Mat tVec;
		cv::solvePnP(normMarkerCoord3D, marker->getPoints(),
					 camMat, distCoeff,
					 rVec, tVec,
					 false);
        
        marker->updatePoseMat(rVec, tVec);
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

void Detect::drawMarker(cv::Mat &img, const Marker &m) {
	// draw outline
	Point2fVec markerPts = m.getPoints();
	const int numPts = (int)markerPts.size();
	cv::Scalar white(255, 255, 255, 255);
	for (int i = 0; i < numPts; i++) {
		cv::line(img, markerPts[i], markerPts[(i + 1) % numPts], white);
	}
    
	// draw centroid
	cv::Scalar green(0, 255, 0, 255);
	cv::Point cross1(2, 2);
	cv::Point cross2(2, -2);
	cv::Point c = m.getCentroid();
	cv::line(img, c - cross1, c + cross1, green);
	cv::line(img, c + cross2, c - cross2, green);
    
	// draw perimeter
	cv::Scalar blue(0, 0, 255, 255);
	cv::circle(img, c, m.getPerimeterRadius(), blue);
}

void Detect::calcProjMat(float viewW, float viewH) {
    printf("ocv_ar::Detect - calculating projection matrix for view size %dx%d\n",
           (int)viewW, (int)viewH);
    
    const bool invert = false;
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
    
    arParamDecompMat(cparam, icpara, trans);
    
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
    
    if (!invert)
    {
        projMat[13] = -projMat[13];
        projMat[1]  = -projMat[1];
        projMat[5]  = -projMat[5];
        projMat[9]  = -projMat[9];
    }
}