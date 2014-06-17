#include "ident_templ.h"

using namespace ocv_ar;

#pragma mark public methods

bool IdentificatorTemplMatch::readMarkerCode(const cv::Mat &area, Marker &marker) {
    // check the border first
    
    
    return false;
}

void IdentificatorTemplMatch::addTemplateImg(int id, const cv::Mat &img, bool stripBorder, bool binarize) {
    assert(id >= 0 && img.rows == img.cols && img.type() == CV_8UC1);   // must be quadratic and grayscale
    
    cv::Mat templ(templSize, templSize, CV_8UC1);
    
    // strip border
    if (stripBorder) {
        assert(img.rows == reqMarkerSize);  // "templ" will then be of size "reqMarkerSize - 2 * borderSize"
        cv::Rect roi(borderSize, borderSize, img.cols - borderSize, img.rows - borderSize);
        templ = img(roi);
    } else {
        assert(img.rows == templSize);
        img.copyTo(templ);
    }
    
    // binarize, if necessary
    if (binarize) {
        cv::threshold(templ, templ, 127, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    }
    
    // add to map
    TemplateMapPair templPair(id, templ);
    templates.insert(templPair);
}

#pragma mark protected methods

bool IdentificatorTemplMatch::checkMarkerCode(const cv::Mat &m, int dir) const {
    return false;
}

int IdentificatorTemplMatch::markerCodeToId(const cv::Mat &m, int dir) const {
    return -1;
}