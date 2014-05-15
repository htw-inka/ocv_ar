#ifndef OCV_AR_DETECT_H
#define OCV_AR_DETECT_H

#include <set>

#include "conf.h"
#include "types.h"
#include "marker.h"

using namespace std;

namespace ocv_ar {

class Detect {
public:
    
    void setCamIntrinsics(cv::Mat &camIntrinsics);
    
    void setFrameOutputLevel(FrameProcLevel level);
    
    void setInputFrame(cv::Mat *frame);
    cv::Mat *getOutputFrame() const { return outFrame; }
    
    void processFrame();
    
    set<Marker *> getMarkers() const { return markers; };
    
private:
    void preprocess();
    
    void performThreshold();
    
    void threshPostProc();
    
    void findContours();
    
    void findMarkerCandidates();
    
    void checkMarkerCandidates();
    
    void discardDuplicateMarkers();
    
    void estimatePositions();
    
    void setOutputFrameOnReqProcLevel(FrameProcLevel reqLvl, cv::Mat *srcFrame = NULL);
    
    int readMarkerCode(cv::Mat &img, int *validRot);
    
    bool checkMarkerCode(const cv::Mat &m, int dir) const;
    int markerCodeToId(const cv::Mat &m, int dir) const;
    
    void drawMarker(cv::Mat &img, const Marker &m);
    
    
    cv::Mat *inFrame;
    cv::Mat *outFrame;
    
    set<Marker *> markers;
};

}

#endif