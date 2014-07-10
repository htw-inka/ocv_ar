/**
 * ocv_ar - OpenCV based Augmented Reality library
 *
 * Configuration file for compile-time settings.
 *
 * Author: Markus Konrad <konrad@htw-berlin.de>, June 2014.
 * INKA Research Group, HTW Berlin - http://inka.htw-berlin.de/
 *
 * See LICENSE for license.
 */

#ifndef OCV_AR_CONF_H
#define OCV_AR_CONF_H

/** Downsampling **/

// perform cv::pyrDown N times
#define OCV_AR_CONF_DOWNSAMPLE  1
// or: use cv::resize
//#define OCV_AR_CONF_RESIZE_W 640
//#define OCV_AR_CONF_RESIZE_H 360

/** thresholding **/
/* see http://docs.opencv.org/modules/imgproc/doc/miscellaneous_transformations.html#adaptivethreshold */

#define OCV_AR_CONF_THRESH_BLOCK_SIZE   7
#define OCV_AR_CONF_THRESH_C            9.0f

/** detection **/

#define OCV_AR_CONF_FIND_CONTOUR_TYPE   CV_RETR_EXTERNAL    // is faster but can cause problems with low contrast images; try CV_RETR_LIST then
#define OCV_AR_CONF_MIN_CONTOUR_PTS     4       // 4 because we need to find squares
#define OCV_AR_CONF_MIN_CONTOUR_LENGTH  15.0f   // in pixels
#define OCV_AR_CONF_MAX_DUPLICATE_DIST  2.0f    // in pixels

/** pose estimation **/

#define OCV_AR_CONF_DEFAULT_MARKER_SIZE_REAL    0.05f   // "real world" marker size in meters

#define OCV_AR_CONF_REFINE_CORNERS_ITER 0       // num. of iterations to refine corners. set 0 to disable

#define OCV_AR_CONF_PROJMAT_NEAR_PLANE  0.01f   // opengl projection matrix near plane
#define OCV_AR_CONF_PROJMAT_FAR_PLANE   100.0f  // opengl projection matrix far plane

#define OCV_AR_CONF_SMOOTHING_HIST_SIZE 5

/** marker identification **/

#define OCV_AR_CONF_MARKER_CODE_PX_PER_FIELD    8       // length in pixels that defines a single marker cell
#define OCV_AR_CONF_TEMPL_MATCH_MAX_ERROR_RATE  0.3f    // max. error rate for template matching

/** tracking **/

#define OCV_AR_CONF_TRACKER_MARKER_TIMEOUT_MS   750.0   // tracker timeout for lost markers in millisec.

#endif