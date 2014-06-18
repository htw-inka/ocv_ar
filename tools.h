#ifndef OCV_AR_TOOLS_H
#define OCV_AR_TOOLS_H

#include <opencv2/opencv.hpp>

namespace ocv_ar {

class Tools {
public:
    static float distSquared(cv::Point2f p1, cv::Point2f p2);
    static void matRot90CW(cv::Mat &m); // fast clock-wise rotation of <m> by 90°
    
    static float norm( float a, float b, float c );
    static float dot( float a1, float a2, float a3,
                     float b1, float b2, float b3 );
    static int  arParamDecompMat( float source[3][4], float cpara[3][4], float trans[3][4] );
};
    
}

#endif