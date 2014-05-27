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


#endif