#include "ident_7bit.h"

using namespace ocv_ar;

IdentificatorType IdentificatorBase::type = CODE_7BIT;

bool Identificator7BitCode::readMarkerCode(cv::Mat &area, Marker &marker) {
    return false;
}

bool Identificator7BitCode::checkMarkerCode(const cv::Mat &m, int dir) const {
    return false;
}

int Identificator7BitCode::markerCodeToId(const cv::Mat &m, int dir) const {
    return 0;
}