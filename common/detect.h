#ifndef OCV_AR_DETECT_H
#define OCV_AR_DETECT_H

#include <vector>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "conf.h"
#include "types.h"
#include "marker.h"
#include "ident.h"
#include "ident_7bit.h"

using namespace std;

namespace ocv_ar {

class Detect {
public:
    Detect(IdentificatorType identType);
    ~Detect();
    
    void prepare(int frameW, int frameH, int frameChan, int cvtType = -1);
    
    void setCamIntrinsics(cv::Mat &camIntrinsics);
    
    void setFrameOutputLevel(FrameProcLevel level);
    
    void setInputFrame(cv::Mat *frame);
    
    void setIdentificator(IdentificatorType identType);
    
    IdentificatorType getIdentificator() const { return ident ? ident->getType() : NONE; }
    
    void processFrame();
    
    cv::Mat *getOutputFrame() const;
    
    vector<Marker> getMarkers() const { return foundMarkers; }
    
    bool isPrepared() const { return prepared; }
    
private:
    void preprocess();
    
    void performThreshold();
    
//    void threshPostProc();
    
    void findContours();
    
    void findMarkerCandidates();
    
    void identifyMarkers();
    
    void discardDuplicateMarkers(vector<Marker> &markerList);
    
    void estimatePositions();
    
    void setOutputFrameOnCurProcLevel(FrameProcLevel curLvl, cv::Mat *srcFrame);
    
//    int readMarkerCode(cv::Mat &img, int *validRot);
//    
//    bool checkMarkerCode(const cv::Mat &m, int dir) const;
//    int markerCodeToId(const cv::Mat &m, int dir) const;
    
    void drawMarker(cv::Mat &img, const Marker &m);
    
    
    bool prepared;
    
    int inputFrameCvtType;
    
    FrameProcLevel outFrameProcLvl;

    cv::Mat *inFrameOrigGray;   // input frame with original size, grayscale
    cv::Mat *inFrame;           // input frame downsampled, grayscale
    cv::Mat *procFrame;         // temporary frame during processing, grayscale/binary
    cv::Mat *outFrame;          // output frame for debugging, grayscale
    
    ContourVec curContours;
    
    int downsampleSizeW;
    int downsampleSizeH;
    
    vector<Marker> possibleMarkers;
    vector<Marker> foundMarkers;
    
    IdentificatorBase *ident;
    
    int normMarkerSize;
	Point2fVec normMarkerCoord2D;	// standard coordinates for a normalized rectangular marker in 2D
	Point3fVec normMarkerCoord3D;	// standard coordinates for a normalized rectangular marker in 3D
};

}

#endif