#ifndef OCV_AR_TYPES_H
#define OCV_AR_TYPES_H

#include <vector>

#include <opencv2/opencv.hpp>

namespace ocv_ar {
    typedef enum _FrameProcLevel {
        DEFAULT = -1,
        PREPROC,
        THRESH,
        CONTOURS,
        POSS_MARKERS,        // possible marker candidates
        DETECTED_MARKERS
    } FrameProcLevel;
    
    typedef enum _IdentificatorType {
        NONE = -1,
        CODE_7BIT,
        CODE_8BIT,
        TEMPLATE
    } IdentificatorType;
    
    typedef std::vector<cv::Point> PointVec;
    typedef std::vector<cv::Point2f> Point2fVec;
    typedef std::vector<cv::Point3f> Point3fVec;
    typedef std::vector<PointVec> ContourVec;
}

#endif