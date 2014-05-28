#include "marker.h"

using namespace ocv_ar;

#pragma mark public methods

Marker::Marker(PointVec &pts) {
	for (int i = 0; i < 4; i++) {
		points.push_back(cv::Point2f(pts[i].x, pts[i].y));
	}
    
	init();
}


Marker::Marker(Point2fVec &pts) {
	points.assign(pts.begin(), pts.begin() + 4);
    
	init();
}

void Marker::rotatePoints(int rot) {
	rotate(points.begin(), points.begin() + 4 - rot, points.end());
}

void Marker::updatePoseMat(const cv::Mat &r, const cv::Mat &t) {
    
}

void Marker::sortPoints() {
	// Sort the points in anti-clockwise order
	cv::Point v1 = points[1] - points[0];
	cv::Point v2 = points[2] - points[0];
    
	// if the third point is in the left side,
	// sort in anti-clockwise order
	if ((v1.x * v2.y) - (v1.y * v2.x) < 0.0) {
		swap(points[1], points[3]);
	}
}

void Marker::calcShapeProperties() {
	centroid = 0.25f * (points[0] + points[1] + points[2] + points[3]);
	float maxDist = numeric_limits<float>::min();
	for (Point2fVec::iterator it = points.begin();
		 it != points.end();
		 ++it)
	{
		float d = cv::norm(centroid - *it);
		maxDist = max(maxDist, d);
	}
    
	perimeterRad = maxDist;
}

#pragma mark private methods

void Marker::init() {
    id = -1;
    
	rVec.create(3, 1, CV_32F);
	tVec.create(3, 1, CV_32F);
    
	sortPoints();
	calcShapeProperties();
}