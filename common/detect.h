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
    Detect(IdentificatorType identType, float markerSizeM = 0.0f, FlipMode flip = FLIP_NONE);
    ~Detect();
    
    void prepare(int frameW, int frameH, int frameChan, int cvtType = -1);
    
    void setCamIntrinsics(const cv::Mat &camMat, const cv::Mat &distCoeff);
    
    void setFrameOutputLevel(FrameProcLevel level);
    
    void setInputFrame(cv::Mat *frame);
    
    void setIdentificator(IdentificatorType identType);
    
    IdentificatorType getIdentificator() const { return ident ? ident->getType() : NONE; }
    
    void processFrame();
    
    cv::Mat *getOutputFrame() const;
    
    vector<Marker *> getMarkers() const { return foundMarkers; }
    
    float *getProjMat(float viewW, float viewH);
    
    float getMarkerScale() const { return markerScale; }
    
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
    
    void drawMarker(cv::Mat &img, const Marker &m);
    
    void calcProjMat(float viewW, float viewH);
    
    
    bool prepared;
    
    int inputFrameCvtType;
    
    FrameProcLevel outFrameProcLvl;

    cv::Mat *inFrameOrigGray;   // input frame with original size, grayscale
    cv::Mat *inFrame;           // input frame downsampled, grayscale
    cv::Mat *procFrame;         // temporary frame during processing, grayscale/binary
    cv::Mat *outFrame;          // output frame for debugging, grayscale
    
    ContourVec curContours;
    
    int inputFrameW;
    int inputFrameH;
    int downsampleSizeW;
    int downsampleSizeH;
    
    vector<Marker> possibleMarkers;
    vector<Marker *> foundMarkers;   // holds pointers to correct markers in <possibleMarkers>
    
    IdentificatorBase *ident;
    
    int normMarkerSize;
	Point2fVec normMarkerCoord2D;	// standard coordinates for a normalized rectangular marker in 2D
	Point3fVec normMarkerCoord3D;	// standard coordinates for a normalized rectangular marker in 3D
    
    cv::Mat camMat;
    cv::Mat distCoeff;
    
    float markerScale;
    
    FlipMode flipProj;
    float projMat[16];
    cv::Size projMatUsedSize;
};

}

#endif