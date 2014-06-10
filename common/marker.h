#ifndef OCV_AR_MARKER_H
#define OCV_AR_MARKER_H

#include <vector>

#include <opencv2/opencv.hpp>

#include "conf.h"
#include "types.h"
#include "marker.h"

using namespace std;

namespace ocv_ar {

class Marker {
public:
    Marker(PointVec &pts);
    Marker(Point2fVec &pts);
    
    void setId(int newId) { id = newId; };
    int getId() const { return id; };
    
    Point2fVec getPoints() const { return points; }
    void addPoint(cv::Point2f p) { points.push_back(p); }
    void clearPoints() { points.clear(); }
    void rotatePoints(int rot);
    
    cv::Point2f getCentroid() const { return centroid; }
    float getPerimeterRadius() const { return perimeterRad; }
    
    const cv::Mat &getRVec() const { return rVec; };
    const cv::Mat &getTVec() const { return tVec; };

    void updatePoseMat(const cv::Mat &r, const cv::Mat &t);
    
//    const glm::mat4 &getPoseMat() const { return poseMat; };
    
    const float *getPoseMatPtr() const { return poseMat; };
    
    void sortPoints();
    void calcShapeProperties();
    
private:
    void init();
    
    int id;
    
    Point2fVec points;
    
    cv::Point2f centroid;
    float perimeterRad;
    
    cv::Mat rVec;
    cv::Mat tVec;
    
    float poseMat[16];  // 4x4 matrix with model-view-transformation
//    glm::mat4 poseMat;      // 4x4 matrix with model-view-transformation
};

}

#endif