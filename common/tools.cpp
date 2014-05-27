#include "tools.h"

using namespace ocv_ar;

float Tools::distSquared(cv::Point2f p1, cv::Point2f p2) {
	const float dX = p1.x - p2.x;
	const float dY = p1.y - p2.y;
    
	return dX * dX + dY * dY;
}