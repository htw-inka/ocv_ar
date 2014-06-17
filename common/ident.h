#ifndef OCV_AR_IDENT_H
#define OCV_AR_IDENT_H

#include <opencv2/opencv.hpp>

#include "marker.h"
#include "types.h"

namespace ocv_ar {

class IdentificatorBase {
public:
    IdentificatorBase(IdentificatorType t, int markerSize) : reqMarkerSize(markerSize),
                                                             type(t)
    {};
    
    virtual ~IdentificatorBase() {};
    
    virtual bool readMarkerCode(const cv::Mat &area, Marker &marker) = 0;
    
    virtual int getRequiredMarkerSize() const { return reqMarkerSize; }
    
    IdentificatorType getType() { return type; }
    
protected:
    void setFoundPropertiesForMarker(Marker &marker, int id, int rot);
    
    
    IdentificatorType type;
    int reqMarkerSize;
};
    
}

#endif
