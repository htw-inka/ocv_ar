/**
 * ocv_ar - OpenCV based Augmented Reality library
 *
 * Helper function class "Tools" -- implementation file.
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

#include "tools.h"

using namespace ocv_ar;

float Tools::distSquared(cv::Point2f p1, cv::Point2f p2) {
	const float dX = p1.x - p2.x;
	const float dY = p1.y - p2.y;
    
	return dX * dX + dY * dY;
}

void Tools::matRot90CW(cv::Mat &m) {
    cv::transpose(m, m);
    cv::flip(m, m, 1);
}

/* BEGIN code from ArUco lib */

float Tools::norm( float a, float b, float c )
{
    return( sqrt( a*a + b*b + c*c ) );
}

float Tools::dot( float a1, float a2, float a3,
                 float b1, float b2, float b3 )
{
    return( a1 * b1 + a2 * b2 + a3 * b3 );
}

int Tools::arParamDecompMat( float source[3][4], float cpara[3][4], float trans[3][4] )
{
    int       r, c;
    float    Cpara[3][4];
    float    rem1, rem2, rem3;
    
    if ( source[2][3] >= 0 )
    {
        for ( r = 0; r < 3; r++ )
        {
            for ( c = 0; c < 4; c++ )
            {
                Cpara[r][c] = source[r][c];
            }
        }
    }
    else
    {
        for ( r = 0; r < 3; r++ )
        {
            for ( c = 0; c < 4; c++ )
            {
                Cpara[r][c] = -(source[r][c]);
            }
        }
    }
    
    for ( r = 0; r < 3; r++ )
    {
        for ( c = 0; c < 4; c++ )
        {
            cpara[r][c] = 0.0;
        }
    }
    cpara[2][2] = norm( Cpara[2][0], Cpara[2][1], Cpara[2][2] );
    trans[2][0] = Cpara[2][0] / cpara[2][2];
    trans[2][1] = Cpara[2][1] / cpara[2][2];
    trans[2][2] = Cpara[2][2] / cpara[2][2];
    trans[2][3] = Cpara[2][3] / cpara[2][2];
    
    cpara[1][2] = dot( trans[2][0], trans[2][1], trans[2][2],
                      Cpara[1][0], Cpara[1][1], Cpara[1][2] );
    rem1 = Cpara[1][0] - cpara[1][2] * trans[2][0];
    rem2 = Cpara[1][1] - cpara[1][2] * trans[2][1];
    rem3 = Cpara[1][2] - cpara[1][2] * trans[2][2];
    cpara[1][1] = norm( rem1, rem2, rem3 );
    trans[1][0] = rem1 / cpara[1][1];
    trans[1][1] = rem2 / cpara[1][1];
    trans[1][2] = rem3 / cpara[1][1];
    
    cpara[0][2] = dot( trans[2][0], trans[2][1], trans[2][2],
                      Cpara[0][0], Cpara[0][1], Cpara[0][2] );
    cpara[0][1] = dot( trans[1][0], trans[1][1], trans[1][2],
                      Cpara[0][0], Cpara[0][1], Cpara[0][2] );
    rem1 = Cpara[0][0] - cpara[0][1]*trans[1][0] - cpara[0][2]*trans[2][0];
    rem2 = Cpara[0][1] - cpara[0][1]*trans[1][1] - cpara[0][2]*trans[2][1];
    rem3 = Cpara[0][2] - cpara[0][1]*trans[1][2] - cpara[0][2]*trans[2][2];
    cpara[0][0] = norm( rem1, rem2, rem3 );
    trans[0][0] = rem1 / cpara[0][0];
    trans[0][1] = rem2 / cpara[0][0];
    trans[0][2] = rem3 / cpara[0][0];
    
    trans[1][3] = (Cpara[1][3] - cpara[1][2]*trans[2][3]) / cpara[1][1];
    trans[0][3] = (Cpara[0][3] - cpara[0][1]*trans[1][3]
                   - cpara[0][2]*trans[2][3]) / cpara[0][0];
    
    for ( r = 0; r < 3; r++ )
    {
        for ( c = 0; c < 3; c++ )
        {
            cpara[r][c] /= cpara[2][2];
        }
    }
    
    return 0;
}
/* END code from ArUco lib */