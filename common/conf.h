#ifndef OCV_AR_CONF_H
#define OCV_AR_CONF_H

#define OCV_AR_CONF_PLATFORM_IOS
//#define #define OCV_AR_CONF_PLATFORM_ANDROID

/** Downsampling **/

// perform cv::pyrDown N times
#define OCV_AR_CONF_DOWNSAMPLE  1
// or: use cv::resize
//#define OCV_AR_CONF_RESIZE_W 640
//#define OCV_AR_CONF_RESIZE_H 360

#define OCV_AR_CONF_THRESH_BLOCK_SIZE   5
#define OCV_AR_CONF_THRESH_C            9.0f

#define OCV_AR_CONF_MIN_CONTOUR_PTS     4
#define OCV_AR_CONF_MIN_CONTOUR_LENGTH  30.0f
#define OCV_AR_CONF_MAX_DUPLICATE_DIST  2.0f

#define OCV_AR_CONF_MARKER_SIZE_REAL    0.05f   // "real world" marker size in meters

#define OCV_AR_CONF_REFINE_CORNERS_ITER 0

#define OCV_AR_CONF_MARKER_CODE_PX_PER_FIELD 8

#endif