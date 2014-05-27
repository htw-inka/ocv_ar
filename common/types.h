#ifndef OCV_AR_TYPES_H
#define OCV_AR_TYPES_H

#include <vector>

namespace ocv_ar {
    typedef enum _FrameProcLevel {
        DEFAULT = -1,
        PREPROC,
        THRESH,
        CONTOURS
    } FrameProcLevel;
    
    typedef std::vector<cv::Point> PointVec;
    typedef std::vector<cv::Point2f> Point2fVec;
    typedef std::vector<cv::Point3f> Point3fVec;
    typedef std::vector<PointVec> ContourVec;
}

#endif