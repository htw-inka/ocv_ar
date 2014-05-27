#ifndef OCV_AR_DETECT_H
#define OCV_AR_DETECT_H

#include <vector>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "conf.h"
#include "types.h"
#include "marker.h"

using namespace std;

namespace ocv_ar {

class Detect {
public:
    Detect();
    ~Detect();
    
    void prepare(int frameW, int frameH, int frameChan, int cvtType = -1);
    
    void setCamIntrinsics(cv::Mat &camIntrinsics);
    
    void setFrameOutputLevel(FrameProcLevel level);
    
    void setInputFrame(cv::Mat *frame);
    
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
    
    void checkMarkerCandidates();
    
    void discardDuplicateMarkers(vector<Marker> &markerList);
    
    void estimatePositions();
    
    void setOutputFrameOnCurProcLevel(FrameProcLevel curLvl, cv::Mat *srcFrame);
    
    int readMarkerCode(cv::Mat &img, int *validRot);
    
    bool checkMarkerCode(const cv::Mat &m, int dir) const;
    int markerCodeToId(const cv::Mat &m, int dir) const;
    
    void drawMarker(cv::Mat &img, const Marker &m);
    
    
    bool prepared;
    
    int inputFrameCvtType;
    
    FrameProcLevel outFrameProcLvl;

    cv::Mat *inFrameOrigGray;
    cv::Mat *inFrame;
    cv::Mat *outFrame;
    
    ContourVec curContours;
    
    int downsampleSizeW;
    int downsampleSizeH;
    
    vector<Marker> possibleMarkers;
    vector<Marker> foundMarkers;  // maps marker id -> marker object
};

}

#endif