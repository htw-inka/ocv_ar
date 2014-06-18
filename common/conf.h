#ifndef OCV_AR_CONF_H
#define OCV_AR_CONF_H

/** Downsampling **/

// perform cv::pyrDown N times
#define OCV_AR_CONF_DOWNSAMPLE  1
// or: use cv::resize
//#define OCV_AR_CONF_RESIZE_W 640
//#define OCV_AR_CONF_RESIZE_H 360

/** thresholding **/

#define OCV_AR_CONF_THRESH_BLOCK_SIZE   5
#define OCV_AR_CONF_THRESH_C            9.0f

/** detection **/

#define OCV_AR_CONF_MIN_CONTOUR_PTS     4
#define OCV_AR_CONF_MIN_CONTOUR_LENGTH  30.0f
#define OCV_AR_CONF_MAX_DUPLICATE_DIST  2.0f

/** pose estimation **/

#define OCV_AR_CONF_DEFAULT_MARKER_SIZE_REAL    0.05f   // "real world" marker size in meters

#define OCV_AR_CONF_REFINE_CORNERS_ITER 0

#define OCV_AR_CONF_PROJMAT_NEAR_PLANE  0.01f
#define OCV_AR_CONF_PROJMAT_FAR_PLANE   100.0f

/** marker identification **/

#define OCV_AR_CONF_MARKER_CODE_PX_PER_FIELD 8
#define OCV_AR_CONF_TEMPL_MATCH_MAX_ERROR_RATE   0.30f

#endif