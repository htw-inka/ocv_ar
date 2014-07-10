/**
 * ocv_ar - OpenCV based Augmented Reality library
 *
 * Detection core header file.
 *
 * Author: Markus Konrad <konrad@htw-berlin.de>, June 2014.
 * INKA Research Group, HTW Berlin - http://inka.htw-berlin.de/
 *
 * This file contains code and inspiration from ArUco library developed at the
 * Ava group of the Univeristy of Cordoba (Spain).
 * See http://sourceforge.net/projects/aruco/
 *
 * See LICENSE for license.
 */

#ifndef OCV_AR_DETECT_H
#define OCV_AR_DETECT_H

#include <vector>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "conf.h"
#include "types.h"
#include "marker.h"
#include "ident.h"
#include "ident_7x7.h"
#include "ident_templ.h"

using namespace std;

namespace ocv_ar {

/**
 * Marker detection class. Handles initialization of the detection pipeline and then
 * allows to perform the marker detection steps for each input frame that is given.
 * The found *possible* markers will be tried to identify according to a identificator
 * type. Results will be saved in a <foundMarkers> vector. Also calculates the pose for each
 * found valid marker.
 */
class Detect {
public:
    /**
     * Initialize a marker detector with identificator type <identType>, a real world
     * marker size in meters <markerSizeM> and a flip mode for the marker pose
     * estimation <flip>.
     *
     * Note:
    *  - You need to call <prepare()> before you can start processing frames!
     * - You need to call <setCamIntrinsics()> before each marker's pose can be estimated.
     */
    Detect(IdentificatorType identType, float markerSizeM = 0.0f, FlipMode flip = FLIP_NONE);
    
    /**
     * Deconstructor.
     */
    ~Detect();
    
    /**
     * Prepare for input frames of size <frameW> x <frameH> with <frameChan>
     * color channels. Specify a conversion type <cvtType> for grayscale conversion of
     * the input frames. If <cvtType> is -1, the conversion type will be guessed.
     */
    void prepare(int frameW, int frameH, int frameChan, int cvtType = -1);
    
    /**
     * Return the preparation state of the detector.
     */
    bool isPrepared() const { return prepared; }
    
    /**
     * Set the camera intrinsics for pose estimation by submitting the camera matrix
     * <camMat> and (optionally empty) distortion coefficients <distCoeff>.
     */
    void setCamIntrinsics(const cv::Mat &camMat, const cv::Mat &distCoeff);
    
    /**
     * Set a frame output level to display the results of each single step in the
     * detection pipeline for debugging purposes. See <types.h> for possible levels.
     */
    void setFrameOutputLevel(FrameProcLevel level);
    
    /**
     * Set an identificator type. See <types.h> for possible identificators.
     */
    void setIdentificatorType(IdentificatorType identType);
    
    /**
     * Get the current identificator type. See <types.h> for possible identificators.
     */
    IdentificatorType getIdentificatorType() const { return ident ? ident->getType() : IDENT_TYPE_NONE; }
    
    /**
     * Get the current identificator object.
     */
    IdentificatorBase *getIdentificator() const { return ident; }
    
    /**
     * Set an input frame for processing.
     * <frame> is a weak ref and will be copied internally if it is a color image
     * and/or if <doNotCopyGrayscaleImg> is false.
     * Otherwise, <frame> will be handled as reference but will not be modified
     * (it is const).
     *
     * Note: You need to call <prepare()> before you can start processing frames!
     */
    void setInputFrame(const cv::Mat *frame, bool doNotCopyGrayscaleImg = false);
    
    /**
     * Process the input frame to detect and identify markers.
     */
    void processFrame();
    
    /**
     * After detection and identification of the markers, estimate their 3D pose.
     */
    void estimateMarkersPoses();
    
    /**
     * Get the output frame according to the "frame output level" set via
     * <setFrameOutputLevel()>. If this level is set to PROC_LEVEL_DEFAULT, it will
     * return NULL, otherwise it will return a weak pointer.
     */
    cv::Mat *getOutputFrame() const;
    
    /**
     * Return a vector with weak pointers to the found markers.
     */
    vector<Marker *> getMarkers() const { return foundMarkers; }
    
    /**
     * Return a 4x4 OpenGL projection matrix that can be used to display the found
     * markers. The projection matrix will be calculated depending on the OpenGL view
     * size <viewW> x <viewH>.
     * A pointer to a float[16] array will be returned.
     */
    float *getProjMat(float viewW, float viewH);
    
    /**
     * Return the set real world marker size in meters.
     */
    float getMarkerScale() const { return markerScale; }
    
private:
    /**
     * Detection step 1 - frame preprocessing: Downscaling, color conversion.
     */
    void preprocess();

    /**
     * Detection step 2 - thresholding.
     */
    void performThreshold();
    
//    void threshPostProc();
    
    /**
     * Detection step 3 - finding contours.
     */
    void findContours();
    
    /**
     * Detection step 4 - finding marker candidates from contours.
     */
    void findMarkerCandidates();
    
    /**
     * Detection step 5 - identifiying markers.
     */
    void identifyMarkers();
    
    /**
     * Discard duplicate markers after detection step 4.
     */
    void discardDuplicateMarkers(const vector<Marker> &inputMarkers, vector<Marker> &filteredMarkers);
    
    /**
     * Helper function to copy the image from <srcFrame> to <outFrame> if the
     * frame processing level <curLvl> matches.
     */
    void setOutputFrameOnCurProcLevel(FrameProcLevel curLvl, cv::Mat *srcFrame);
    
    /**
     * Helper function to draw a marker <m> to the image <img>. Optionally draw the
     * marker id (<drawId>).
     */
    void drawMarker(cv::Mat &img, const Marker &m, bool drawId);
    
    /**
     * Calculates the OpenGL projection matrix for a view of size <viewW> x <viewH>.
     * See <getProjMat()>.
     */
    void calcProjMat(float viewW, float viewH);
    
    
    bool prepared;                  // detector is prepared (<prepare()> called)?
    
    int inputFrameCvtType;          // color conversion type for input frames
    
    FrameProcLevel outFrameProcLvl; // frame output processing level

    const cv::Mat *inFrameRef;
    cv::Mat *inFrameOrigGray;   // input frame with original size, grayscale
    cv::Mat *inFrame;           // input frame downsampled, grayscale
    cv::Mat *procFrame;         // temporary frame during processing, grayscale/binary
    cv::Mat *outFrame;          // output frame for debugging, grayscale
    
    ContourVec curContours;     // vector with extracted contours
    
    int inputFrameW;            // original input frame width
    int inputFrameH;            // original input frame height
    int downsampleSizeW;        // frame width after preprocessing
    int downsampleSizeH;        // frame height after preprocessing
    
    vector<Marker> possibleMarkers; // possible markers in the current frame (i.e. all squares found in the image)
    vector<Marker *> foundMarkers;  // holds pointers to all valid markers in <possibleMarkers>
    
    IdentificatorBase *ident;   // marker identificator object
    
    int normMarkerSize;
	Point2fVec normMarkerCoord2D;	// standard coordinates for a normalized rectangular marker in 2D
	Point3fVec normMarkerCoord3D;	// standard coordinates for a normalized rectangular marker in 3D
    
    cv::Mat camMat;             // cam intrinsics: camera matrix
    cv::Mat distCoeff;          // cam intrinsics: distortion coefficients (may be empty)
    
    float markerScale;          // real world marker size in meters
    
    FlipMode flipProj;          // flip mode for the projection matrix
    float projMat[16];          // 4x4 OpenGL projection matrix
    cv::Size projMatUsedSize;   // the view size for which <projMat> was calculated
};

}

#endif
