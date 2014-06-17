#ifndef OCV_AR_IDENT_H
#define OCV_AR_IDENT_H

#include <opencv2/opencv.hpp>

#include "marker.h"
#include "types.h"

namespace ocv_ar {

class IdentificatorBase {
public:
    IdentificatorBase(int markerSize) : reqMarkerSize(markerSize) {};
    
    virtual ~IdentificatorBase() {};
    
    virtual bool readMarkerCode(const cv::Mat &area, Marker &marker) = 0;
    
    virtual int getRequiredMarkerSize() const { return reqMarkerSize; }
    
    static IdentificatorType getType() { return type; }
    
protected:
    virtual bool checkMarkerCode(const cv::Mat &m, int dir) const = 0;
    virtual int markerCodeToId(const cv::Mat &m, int dir) const = 0;
    
    
    static IdentificatorType type;
    
    int reqMarkerSize;
};
    
}

#endif
