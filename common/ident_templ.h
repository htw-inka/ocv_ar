#ifndef OCV_AR_IDENT_TEMPL_H
#define OCV_AR_IDENT_TEMPL_H

#include <opencv2/opencv.hpp>

#include "marker.h"
#include "types.h"
#include "ident.h"

namespace ocv_ar {

class IdentificatorTemplMatch : public IdentificatorBase {
public:
    IdentificatorTemplMatch() : IdentificatorBase(8 * OCV_AR_CONF_MARKER_CODE_PX_PER_FIELD),
                                borderSize(OCV_AR_CONF_MARKER_CODE_PX_PER_FIELD)
    {
        templSize = reqMarkerSize - 2 * borderSize;
    };
    
    virtual bool readMarkerCode(const cv::Mat &area, Marker &marker);
    
    void addTemplateImg(int id, const cv::Mat &img, bool stripBorder = true, bool binarize = false);
    
protected:
    virtual bool checkMarkerCode(const cv::Mat &m, int dir) const;
    virtual int markerCodeToId(const cv::Mat &m, int dir) const;
    
private:
    int borderSize;
    int templSize;
    TemplateMap templates;
};

}

#endif