#include "ident_7bit.h"

using namespace ocv_ar;

static unsigned char possibleBitcodes[4][5] = {
	{ 1,0,0,0,0 },
	{ 1,0,1,1,1 },
	{ 0,1,0,0,1 },
	{ 0,1,1,1,0 }
};

IdentificatorType IdentificatorBase::type = IDENT_TYPE_CODE_7BIT;

bool Identificator7BitCode::readMarkerCode(cv::Mat &area, Marker &marker) {
    // bitMatrix will contain the read marker code raw bits
    cv::Mat bitMatrix = cv::Mat::zeros(5, 5, CV_8UC1);  // 5 because in the 7 bit marker, 1 bit on each side must be 0
    
    // for each marker cell, count the number of "ones" (white pixels)
    // and check if we can alread dismiss the marker (border cell is mainly white)
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
    
    // create transposed bit matrix for rotations
    cv::Mat bitMatrixT;
    cv::transpose(bitMatrix, bitMatrixT);
    
    cv::Mat *curBitMat; // a pointer to either bitMatrix or bitMatrixT
    
    // check marker code for all possible 4 rotations
    for (int rot = 0; rot < 4; rot++) {
    	int dir = -2 * (rot % 2) + 1; // dir is -1 or 1
    	if (rot < 2) {
    		curBitMat = &bitMatrix;
    	} else {
    		curBitMat = &bitMatrixT;
    	}
        
    	if (checkMarkerCode(*curBitMat, dir)) { // found a valid marker code!
            // set the Id and rotate the corner points
    		marker.setId(markerCodeToId(*curBitMat, dir));
            marker.rotatePoints(rot);
            
            return true;
    	}
    }
    
    return false;
}

bool Identificator7BitCode::checkMarkerCode(const cv::Mat &m, int dir) const {
	// set start index depending on reading direction
	int start = (dir > 0) ? 0 : m.cols - 1;
    
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
			bool nextBit = true;
			bool invalidBit = false;
			int i = start;
			int j = 0;
			while (nextBit) {
				if (readCode[i] != testCode[j]) {	// invalid bit found!
					invalidBit = true;
					break;
				}
                
				i += dir;
				if (dir > 0) {
					j = i;
					nextBit = (i < m.cols);
				} else {
					j = start - i;
					nextBit = (i > 0);
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

int Identificator7BitCode::markerCodeToId(const cv::Mat &m, int dir) const {
	int id = 0;
    
	// set start index depending on reading direction
	int start = (dir > 0) ? 0 : m.cols - 1;
    
	// go through all bitcode rows in the read matrix
	int r = start;
	unsigned char u, v;
	bool nextRow = true;
	while (nextRow) {
        // get current row
		const unsigned char *row =  m.ptr<unsigned char>(r);
        
        // select u and v bits
		if (dir > 0) {
			u = row[1];
			v = row[3];
		} else {	// reverse order
			u = row[3];
			v = row[1];
		}
        
		// code from ARuCo:
		id <<= 1;
        if (u) id |= 1;
		id <<= 1;
        if (v) id |= 1;
        
        // next row
		nextRow = (dir > 0) ? (r < m.cols) : (r > 0);
		r += dir;
	}
    
	return id;
}