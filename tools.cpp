/**
 * ocv_ar - OpenCV based Augmented Reality library
 *
 * Helper function class "Tools" -- implementation file.
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

#include <cmath>
#include <ctime>

#include "tools.h"

#define VEC3_NORM(v) (sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]))
#define VEC4_NORM(v) (sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]))

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

float Tools::getAverageAngle(float *angles, int count) {
    double x = 0;
    double y = 0;
    
    // sum all values
    for (int i = 0; i < count; i++) {
        x += cos(angles[i]);
        y += sin(angles[i]);
    }
    
    // devide by the amount of values
    x /= count;
    y /= count;
    
    if (x > 0) {
        return atan(y / x);
    } else if (x < 0) {
        return M_PI + atan(y / x);
    }
    
    // this will just be used when x = 0
    return (y > 0) ? (M_PI / 2.0f) : -(M_PI / 2.0f);
}

void Tools::rotVecToQuat(const float r[3], float q[4]) {
    float norm = VEC3_NORM(r);
    float halfAng = norm / 2.0f;
    q[0] = r[0] / norm * sinf(halfAng);
    q[1] = r[1] / norm * sinf(halfAng);
    q[2] = r[2] / norm * sinf(halfAng);
    q[3] = cosf(halfAng);
}

void Tools::quatToRotVec(float q[4], float r[3]) {
    if (q[3] > 1.0f) {
        float norm = VEC4_NORM(q);
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;
    }
    
    float ang = 2.0f * acosf(q[3]);
    float s = sqrtf(1.0f - q[3] * q[3]) * ang;
    
    if (s < 0.001f) {
        r[0] = q[0];
        r[1] = q[1];
        r[2] = q[2];
    } else {
        r[0] = q[0] / s;
        r[1] = q[1] / s;
        r[2] = q[2] / s;
    }
}

float Tools::quatDot(const float q1[4], const float q2[4]) {
    return q1[0] * q2[0]
         + q1[1] * q2[1]
         + q1[2] * q2[2]
         + q1[3] * q2[3];
}

//void Tools::getAvgRotVec(const float *rotVecs, int numRotVecs, float *avgRotVec) {
//    float avgTheta = 0.0f;
//    float avgDirVec[] = {0.0f, 0.0f, 0.0f};
//    for (int i = 0; i < numRotVecs; ++i) {
//        float v1 = rotVecs[i * 3];
//        float v2 = rotVecs[i * 3 + 1];
//        float v3 = rotVecs[i * 3 + 2];
//        avgTheta += sqrtf(v1 * v1 + v2 * v2 + v3 * v3);
//        avgDirVec[0] += v1;
//        avgDirVec[1] += v2;
//        avgDirVec[2] += v3;
//    }
//    
//    avgTheta /= numRotVecs;
//    avgDirVec[0] /= numRotVecs;
//    avgDirVec[1] /= numRotVecs;
//    avgDirVec[2] /= numRotVecs;
//    
//    avgRotVec[0] = avgTheta * avgDirVec[0];
//    avgRotVec[1] = avgTheta * avgDirVec[1];
//    avgRotVec[2] = avgTheta * avgDirVec[2];
//}

void Tools::printFloatMat(const float *m, int rows, int cols) {
    assert(m);
    
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            printf("%f ", m[y * cols + x]);
        }
        printf("\n");
    }
}

//void Tools::composeModelViewMatrix(float* camPos, float* camRot, float* matrixColumns) {
//    matrixColumns[0] =  cos(camRot[2]) * cos(camRot[1]) - sin(camRot[2]) * sin(camRot[0]) * sin(camRot[1]);
//    matrixColumns[1] = -( sin(camRot[2]) * cos(camRot[1]) + cos(camRot[2]) * sin(camRot[0]) * sin(camRot[1]));
//    matrixColumns[2] =  cos(camRot[0]) * sin(camRot[1]);
//    matrixColumns[3] = 0;
//    
//    matrixColumns[4] =  sin(camRot[2]) * cos(camRot[0]);
//    matrixColumns[5] =  cos(camRot[2]) * cos(camRot[0]);
//    matrixColumns[6] =  -sin(camRot[0]);
//    matrixColumns[7] =  0;
//    
//    matrixColumns[8] =  cos(camRot[2]) * sin(camRot[1]) + sin(camRot[2]) * sin(camRot[0]) * cos(camRot[1]);
//    matrixColumns[9] =  sin(camRot[2]) * sin(camRot[1]) - cos(camRot[2]) * sin(camRot[0]) * cos(camRot[1]);
//    matrixColumns[10] = -cos(camRot[0]) * cos(camRot[1]);
//    matrixColumns[11] = 0;
//    
//    matrixColumns[12] = -(camPos[0] * matrixColumns[0] + camPos[1] * matrixColumns[4] + camPos[2] * matrixColumns[8]),
//    matrixColumns[13] = (camPos[0] * matrixColumns[1] + camPos[1] * matrixColumns[5] + camPos[2] * matrixColumns[9]),
//    matrixColumns[14] = -(camPos[0] * matrixColumns[2] + camPos[1] * matrixColumns[6] + camPos[2] * matrixColumns[10]),
//    matrixColumns[15] = 1;
//}

double Tools::nowMs() {
    return ((double)clock() / CLOCKS_PER_SEC) * 1000.0;
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