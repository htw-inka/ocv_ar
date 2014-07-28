/**
 * ocv_ar - OpenCV based Augmented Reality library
 *
 * Marker identification for ArUco style 7x7 markers -- implementation file.
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

#include "ident_7x7.h"

#include "tools.h"

using namespace ocv_ar;

static unsigned char possibleBitcodes[4][5] = {
	{ 1,0,0,0,0 },
	{ 1,0,1,1,1 },
	{ 0,1,0,0,1 },
	{ 0,1,1,1,0 }
};

bool Identificator7x7::readMarkerCode(const cv::Mat &area, Marker &marker) {
    assert(area.rows == area.cols && area.rows == reqMarkerSize && area.type() == CV_8UC1);   // must be quadratic and grayscale
    
    // bitMatrix will contain the read marker code raw bits
    cv::Mat bitMatrix = cv::Mat::zeros(5, 5, CV_8UC1);  // 5 because in the 7 bit marker, 1 bit on each side must be 0
    
    // for each marker cell, count the number of "ones" (white pixels)
    // and check if we can already dismiss the marker (border cell is mainly white)
    // or otherwise set the according bitmatrix bit
    for (int y = 0; y < 7; y++) {
    	for (int x = 0; x < 7; x++) {
    		int cellX = x * markerCellSize;
    		int cellY = y * markerCellSize;
    		cv::Mat cell = area(cv::Rect(cellX, cellY, markerCellSize, markerCellSize));
            
    		int nonZ = cv::countNonZero(cell);
            
    		// Check number of non zero pixels
    		if (nonZ > minSetMarkerPixels) {
    			if (y == 0 || y == 6 || x == 0 || x == 6) {	// border must be black!
    				return false;
    			} else {	// set to "1" for this cell
    				bitMatrix.at<uchar>(x - 1, y - 1) = 1;
    			}
    		}
    	}
    }
    
    // check marker code for all possible 4 rotations
//    printf("---\n");
    for (int rot = 0; rot < 4; rot++) {
    	if (checkMarkerCode(bitMatrix)) { // found a valid marker code!
            // set the id and rotate the corner points
            int id = markerCodeToId(bitMatrix);
            marker.setId(id);
            marker.rotatePoints(rot);

            return true;
    	}
        
        Tools::matRot90CW(bitMatrix);   // rotate the matrix
    }
//    printf("---\n");
    
    return false;
}

bool Identificator7x7::checkMarkerCode(const cv::Mat &m) const {
	// go through all bitcode rows in the read matrix
	for (int r = 0; r < m.rows; r++) {
		// select read code row
		const unsigned char *readCode = m.ptr<unsigned char>(r);
        
		bool foundCode = false;
        
		// go through all possible bitcodes
        const int numPossibleCodes = sizeof(possibleBitcodes) / sizeof(possibleBitcodes[0]);
		for (int p = 0; p < numPossibleCodes; p++) {
			// select possible code row
			const unsigned char *testCode = possibleBitcodes[p];
            
			// go through all bits in the row depending on direction
			bool invalidBit = false;
			
			for (int i = 0; i < m.cols; i++) {
				if (readCode[i] != testCode[i]) {	// invalid bit found!
					invalidBit = true;
					break;
				}
			}
            
			if (!invalidBit) {	// this bitcode row is valid!
				foundCode = true;	// so check the next row
				break;
			}
		}
        
		if (!foundCode) {
			return false;
		}
	}
    
	return true;

}

int Identificator7x7::markerCodeToId(const cv::Mat &m) const {
	int id = 0;
    
	// go through all bitcode rows in the read matrix
    // the matrix rows must be read "backward" to result in the same IDs
    // as the ArUco lib
	for (int r = m.rows - 1; r >= 0; r--) {
        // get current row
		const unsigned char *row =  m.ptr<unsigned char>(r);
        
		// code from ARuCo:
		id <<= 1;
        if (row[1]) id |= 1;
		id <<= 1;
        if (row[3]) id |= 1;
	}
    
	return id;
}