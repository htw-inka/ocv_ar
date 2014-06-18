#ifndef OCV_AR_IDENT_TEMPL_H
#define OCV_AR_IDENT_TEMPL_H

#include <opencv2/opencv.hpp>

#include "marker.h"
#include "types.h"
#include "ident.h"

namespace ocv_ar {

class IdentificatorTemplMatch : public IdentificatorBase {
public:
    IdentificatorTemplMatch();
    
    virtual ~IdentificatorTemplMatch();
    
    virtual bool readMarkerCode(const cv::Mat &area, Marker &marker);
    
    void addTemplateImg(int id, const cv::Mat &img, bool stripBorder = true, bool binarize = false);
    
private:
    bool checkTemplateRotations(const cv::Mat &marker, const cv::Mat *templRotations, int *validRot);
    bool checkBorder(const cv::Mat &img, int dir, int off);
    
    
    int borderSize;
    int templSize;
    int templSizeSq;
    int minSetMarkerPixels;
    TemplateMap templates;
};

}

#endif