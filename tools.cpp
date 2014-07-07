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
    float x = 0;
    float y = 0;
    
    // sum all values
    for (int i = 0; i < count; i++) {
        x += cosf(angles[i]);
        y += sinf(angles[i]);
    }
    
    // devide by the amount of values
    x /= (float)count;
    y /= (float)count;
    
    return atan2f(y, x);
}


void Tools::rotVecToEuler(const float r[3], float eu[3]) {
    float ang = VEC3_NORM(r);
    float s = sinf(ang);
    float c = cosf(ang);
    float t = 1.0f - c;
    
    float x = r[0] / ang;
    float y = r[1] / ang;
    float z = r[2] / ang;
    
    float singCheck = x * y * t + z * s;
    if (fabsf(singCheck) > 0.998f) { // north or south pole singularity detected
        float sign = singCheck > 0.0f ? 1.0f : -1.0f;
		eu[0] = sign * 2.0f * atan2f(x * sinf(ang / 2.0f), cosf(ang / 2.0f));
		eu[1] = sign * M_PI / 2.0f;
		eu[2] = 0.0f;
	} else {
        eu[0] = atan2f(y * s - x * z * t, 1.0f - (y*y + z*z) * t);
        eu[1] = asinf(x * y * t + z * s);
        eu[2] = atan2f(x * s - y * z * t, 1.0f - (x*x + z*z) * t);
    }
}

void Tools::eulerToRotVec(const float eu[3], float r[3]) {
    float euXhalf = eu[0] / 2.0f;
    float euYhalf = eu[1] / 2.0f;
    float euZhalf = eu[2] / 2.0f;
    
    float c1 = cosf(euXhalf);
	float s1 = sinf(euXhalf);
	float c2 = cosf(euYhalf);
	float s2 = sinf(euYhalf);
	float c3 = cosf(euZhalf);
	float s3 = sinf(euZhalf);
    
    float c1c2 = c1 * c2;
	float s1s2 = s1 * s2;
	float w = c1c2 * c3 - s1s2 * s3;
	r[0] =c1c2 * s3 + s1s2 * c3;
	r[1] =s1 * c2 * c3 + c1 * s2 * s3;
	r[2] =c1 * s2 * c3 - s1 * c2 * s3;
	float ang = 2.0f * acosf(w);
    
    float norm = VEC3_NORM(r);
	if (norm < 0.001f) { // when all euler angles are zero angle =0 so
		// we can set axis to anything to avoid divide by zero
		r[0] = 1.0f;
		r[1] = r[2] = 0.0f;
	} else {
		float magn = ang / sqrtf(norm);
    	r[0] *= magn;
    	r[1] *= magn;
    	r[2] *= magn;
    }
}

void Tools::printFloatMat(const float *m, int rows, int cols) {
    assert(m);
    
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            printf("%f ", m[y * cols + x]);
        }
        printf("\n");
    }
}

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