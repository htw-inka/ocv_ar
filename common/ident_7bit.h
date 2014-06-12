#ifndef OCV_AR_IDENT_7BIT_H
#define OCV_AR_IDENT_7BIT_H

#include <opencv2/opencv.hpp>

#include "marker.h"
#include "types.h"
#include "ident.h"

namespace ocv_ar {

class Identificator7BitCode : public IdentificatorBase {
public:
    Identificator7BitCode() : IdentificatorBase(7 * OCV_AR_CONF_MARKER_CODE_PX_PER_FIELD),
                              markerCellSize(OCV_AR_CONF_MARKER_CODE_PX_PER_FIELD),
                              minSetMarkerPixels(OCV_AR_CONF_MARKER_CODE_PX_PER_FIELD * OCV_AR_CONF_MARKER_CODE_PX_PER_FIELD / 2)
                            {};
    
    
    virtual bool readMarkerCode(cv::Mat &area, Marker &marker);
    
protected:
    virtual bool checkMarkerCode(const cv::Mat &m, int dir) const;
    virtual int markerCodeToId(const cv::Mat &m, int dir) const;
    
private:
    int markerCellSize;
    int minSetMarkerPixels;
};

}

#endif