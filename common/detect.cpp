#include "detect.h"

#include "tools.h"

using namespace ocv_ar;

#pragma mark public methods

Detect::Detect() {
    prepared = false;
    inputFrameCvtType = -1;
    outFrameProcLvl = DEFAULT;
    
    inFrameOrigGray = NULL;
    inFrame = NULL;
    outFrame = NULL;
    
    downsampleSizeW = downsampleSizeH = 0;
    
#if !defined(OCV_AR_CONF_DOWNSAMPLE) && defined(OCV_AR_CONF_RESIZE_W) && defined(OCV_AR_CONF_RESIZE_H)
    downsampleSizeW = OCV_AR_CONF_RESIZE_W;
    downsampleSizeH = OCV_AR_CONF_RESIZE_H;
#endif
}

Detect::~Detect() {
    if (inFrameOrigGray) delete inFrameOrigGray;
    if (inFrame) delete inFrame;
    if (outFrame) delete outFrame;
}

void Detect::prepare(int frameW, int frameH, int frameChan, int cvtType) {
    assert(frameW > 0 && frameH > 0 && (frameChan == 1 || frameChan == 3 || frameChan == 4));
    
    // alloc mem for orig. sized image in grayscale format
    if (inFrameOrigGray) delete inFrameOrigGray;
    inFrameOrigGray = new cv::Mat(frameH, frameW, CV_8UC1);
    
    // alloc mem for downsampled image in grayscale format
#ifdef OCV_AR_CONF_DOWNSAMPLE
    int frac = pow(2, OCV_AR_CONF_DOWNSAMPLE);
    int downW = frameW / frac;
    int downH = frameH / frac;

    if (inFrame) delete inFrame;
    
    inFrame = new cv::Mat(downH, downW, CV_8UC1);
#elif defined(OCV_AR_CONF_RESIZE_W) && defined(OCV_AR_CONF_RESIZE_H)
    assert(downsampleSizeW <= frameW && downsampleSizeH <= frameH);
    if (!inFrame) {
        inFrame = new cv::Mat(downsampleSizeH, downsampleSizeW, CV_8UC1);
    }
#else
#error Either OCV_AR_CONF_DOWNSAMPLE or OCV_AR_CONF_RESIZE_W/H must be defined.
#endif
    
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
}

void Detect::setCamIntrinsics(cv::Mat &camIntrinsics) {
    
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
                          *inFrame,
                          255,
                          cv::ADAPTIVE_THRESH_MEAN_C, // ADAPTIVE_THRESH_GAUSSIAN_C
                          cv::THRESH_BINARY_INV,
                          OCV_AR_CONF_THRESH_BLOCK_SIZE,
                          OCV_AR_CONF_THRESH_C);
    
    setOutputFrameOnCurProcLevel(THRESH, inFrame);
}

//void Detect::threshPostProc() {
//    
//}

void Detect::findContours() {
	// find contours
	ContourVec allContours;
	cv::findContours(*inFrame, allContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE); // CV_RETR_LIST or CV_RETR_EXTERNAL

	// filter out contours consisting of
	// less than <minContourPointsAllowed> points
	curContours.clear();
	for (ContourVec::iterator it = allContours.begin();
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
    
	possibleMarkers.clear();
	PointVec  approxCurve;
    
	for (ContourVec::iterator it = curContours.begin();
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

    printf("Num. marker candidates: %lu\n", possibleMarkers.size());
    
    discardDuplicateMarkers(possibleMarkers);
    
    printf("Num. marker candidates without duplicates: %lu\n", possibleMarkers.size());
    
	// draw markers if necessary
	if (outFrameProcLvl == POSS_MARKERS) {
		inFrame->copyTo(*outFrame);
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

void Detect::checkMarkerCandidates() {
    
}

void Detect::estimatePositions() {
    
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
            if (dist <= maxDuplDistSquared && cur->getPerimeterRadius() >= other->getPerimeterRadius()) {
                printf("Will remove duplicate! dist = %f, r1 = %f, r2 = %f \n",
                       dist, cur->getPerimeterRadius(), other->getPerimeterRadius());
                
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

int Detect::readMarkerCode(cv::Mat &img, int *validRot) {
    return 0;
}

bool Detect::checkMarkerCode(const cv::Mat &m, int dir) const {
    return false;
}

int Detect::markerCodeToId(const cv::Mat &m, int dir) const {
    return 0;
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
	const int numPts = markerPts.size();
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