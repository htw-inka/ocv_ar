/**
 * ocv_ar - OpenCV based Augmented Reality library
 *
 * Marker class to describe single found markers in an image -- header file.
 *
 * Authors: Markus Konrad <konrad@htw-berlin.de>, Alexander Godoba, June 2014.
 * INKA Research Group, HTW Berlin - http://inka.htw-berlin.de/
 *
 * This file contains code and inspiration from ArUco library developed at the
 * Ava group of the Univeristy of Cordoba (Spain).
 * See http://sourceforge.net/projects/aruco/
 *
 * See LICENSE for license.
 */

#ifndef OCV_AR_MARKER_H
#define OCV_AR_MARKER_H

#include <vector>
//#include <list>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "types.h"

using namespace std;

namespace ocv_ar {

/**
 * Marker class that describes the properties of a possible or valid marker in
 * an image.
 */
class Marker {
public:
    /**
     * Constructor to create a marker with the corner points <pts>, submitted as
     * PointVec.
     */
    Marker(PointVec &pts);
    
    /**
     * Constructor to create a marker with the corner points <pts>, submitted as
     * Point2fVec.
     */
    Marker(Point2fVec &pts);
    
    /**
     * Copy constructor
     */
    Marker(const Marker &other);
    
    /**
     * Deconstructor.
     */
    ~Marker();
    
    /**
     * Set the marker ID to <newId>.
     */
    void setId(int newId) { id = newId; }
    
    /**
     * Return the marker ID.
     */
    int getId() const { return id; }
    
    /**
     * Updates <detectMs> to "now".
     */
    void updateDetectionTime();
    
    /**
     * Return the timestamp for the last detection time in milliseconds.
     */
    double getDetectionTimeMs() const { return detectMs; }
    
    /**
     * Return the corner points of this marker.
     */
    Point2fVec getPoints() const { return points; }
    
    /**
     * Set the points in <pVec>.
     */
    void setPoints(const Point2fVec &pVec) { points.assign(pVec.begin(), pVec.end()); }
    
    /**
     * Add a point <p> to the corner points.
     */
    void addPoint(cv::Point2f p) { points.push_back(p); }
    
    /**
     * Clear the corner points.
     */
    void clearPoints() { points.clear(); }
    
    /**
     * Rotate the corner points in the vector <rot> times.
     */
    void rotatePoints(int rot);
    
    /**
     * Return the controid calculated from the corner points.
     */
    cv::Point2f getCentroid() const { return centroid; }
    
    /**
     * Return the perimeter radius calculated from the corner points.
     */
    float getPerimeterRadius() const { return perimeterRad; }
    
    /**
     * Return the 3D pose rotation vector.
     */
    const cv::Mat &getRVec() const { return rVec; };
    
    /**
     * Return the 3D pose translation vector.
     */
    const cv::Mat &getTVec() const { return tVec; };

    /**
     * Update the 3D pose by rotation vector <r> and translation vector <t>.
     */
    void updatePoseMat(const cv::Mat &r, const cv::Mat &t);
    
    /**
     * Update the 3D pose for tracking, which means the T and R vectors from
     * <other> will be copied and interpolated with previous T and R vectors
     * to achieve a smooth transition between marker poses.
     */
    void updateForTracking(const Marker &other);
    
    /**
     * Return the 4x4 OpenGL 3D pose model-view matrix as pointer to
     * the internal float[16].
     */
    const float *getPoseMatPtr() const { return poseMat; };
    
private:
    /**
     * Common initialize method for the marker class.
     */
    void init();
    
    /**
     * Sort the points in anti-clockwise order.
     */
    void sortPoints();
    
    /**
     * Calculate the shape properties from the corner points like centroid
     * and perimeter radius.
     */
    void calcShapeProperties();
    
    /**
     * Update vector history arrays for smoothing (<tVecHist> and <rVecHist>) with
     * elements from <r> and <t> (3-component vectors each).
     */
    void pushVecsToHistory(const float *r, const float *t);
    
    /**
     * Updates <r> and <t> with new values calulated by taking into account the former
     * pose vector values in <rVecHist> and <tVecHist>. <r> and <t> are 3-component
     * vectors each.
     */
    void calcSmoothPoseVecs(float *r, float *t);
    
    /**
     * Calculate the 3D pose matrix from translation and rotation vectors <rVec> and
     * <tVec>.
     */
    void calcPoseMat();
    
    
    int id;                 // marker ID
    
    double detectMs;        // timestamp of last detection in milliseconds
    
    Point2fVec points;      // corner points
    
    cv::Point2f centroid;   // centroid formed by the corner points
    float perimeterRad;     // perimenter radius formed by the corner points
    
    cv::Mat rVec;           // 3D pose rotation vector
    cv::Mat tVec;           // 3D pose translation vector
    
    int pushedHistVecs;     // number of vectors in <tVecHist> and <rVecHist>
    
    float *tVecHist;        // marker position history with N * 3 elements for smoothing effect
    float *rVecHist;        // marker rotation history with N * 3 elements (euler vectors) for smoothing effect
    
    float poseMat[16];      // OpenGL 4x4 matrix with model-view-transformation
};

}

#endif