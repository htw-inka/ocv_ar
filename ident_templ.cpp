/**
 * ocv_ar - OpenCV based Augmented Reality library
 *
 * Marker identification via simple template matching of binary
 * images -- implementation file.
 *
 * Author: Markus Konrad <konrad@htw-berlin.de>, June 2014.
 * INKA Research Group, HTW Berlin - http://inka.htw-berlin.de/
 *
 * See LICENSE for license.
 */

#include "ident_templ.h"

#include "tools.h"

using namespace ocv_ar;

#pragma mark constructors/deconstructor

IdentificatorTemplMatch::IdentificatorTemplMatch() :
    IdentificatorBase(IDENT_TYPE_TEMPL_MATCH, 8 * OCV_AR_CONF_MARKER_CODE_PX_PER_FIELD),
    borderSize(OCV_AR_CONF_MARKER_CODE_PX_PER_FIELD),
    minSetMarkerPixels(OCV_AR_CONF_MARKER_CODE_PX_PER_FIELD * OCV_AR_CONF_MARKER_CODE_PX_PER_FIELD / 2)
{
    // calculate defaults
    templSize = reqMarkerSize - 2 * borderSize;
    templSizeSq = templSize * templSize;
    
    printf("ocv_ar::IdentificatorTemplMatch - req. marker size: %d, templ. size: %d\n",
           reqMarkerSize, templSize);
}

IdentificatorTemplMatch::~IdentificatorTemplMatch() {
    // clear templates map
    for (TemplateMap::iterator it = templates.begin();
         it != templates.end();
         ++it)
    {
        cv::Mat *templArr = it->second;
        delete [] templArr;
    }
}

#pragma mark public methods

bool IdentificatorTemplMatch::readMarkerCode(const cv::Mat &area, Marker &marker) {
    assert(area.rows == area.cols && area.rows == reqMarkerSize && area.type() == CV_8UC1);   // must be quadratic and grayscale
    
    // check the border as "cells" first
    bool borderOk =    checkBorder(area, 0, 0)      // top row
                    && checkBorder(area, 0, 7)      // bottom row
                    && checkBorder(area, 1, 0)      // left column
                    && checkBorder(area, 1, 7);     // right column
    
    if (!borderOk) return false;
    
//    printf("ocv_ar::IdentificatorTemplMatch - border checks ok\n");
    
    // get only the "content" of <area>, i.e. the marker without borders
    int areaContentSize = area.cols - 2 * borderSize;
    cv::Rect roi(borderSize, borderSize, areaContentSize, areaContentSize);
    cv::Mat areaContent(area, roi);
    
    
    // do the template matching for all templates we have
    for (TemplateMap::iterator it = templates.begin();
         it != templates.end();
         ++it)
    {
        int validRot;
        if (checkTemplateRotations(areaContent, it->second, &validRot)) {  // found a matching template!
            marker.setId(it->first);
            marker.rotatePoints(validRot);
            return true;
        }
    }
    
    return false;
}

void IdentificatorTemplMatch::addTemplateImg(int id, const cv::Mat &img, bool stripBorder, bool binarize) {
    assert(id >= 0 && img.rows == img.cols && img.type() == CV_8UC1);   // must be quadratic and grayscale
    
    if (templates.find(id) != templates.end()) {
        printf("ocv_ar::IdentificatorTemplMatch - template not added to library - id %d already exists\n", id);
        
        return;
    }
    
    cv::Mat templ(templSize, templSize, CV_8UC1);
    
    // strip border
    if (stripBorder) {
        const cv::Mat *imgToUse;
        cv::Mat imgResized;
        
        if (img.cols != reqMarkerSize) {    // resize
            cv::resize(img, imgResized, cv::Size(reqMarkerSize, reqMarkerSize));
            imgToUse = &imgResized;
        } else {
            imgToUse = &img;
        }

        // "templ" will then be of size "reqMarkerSize - 2 * borderSize"
        int imgContentSize = imgToUse->cols - 2 * borderSize;
        cv::Rect roi(borderSize, borderSize, imgContentSize, imgContentSize);
        templ = (*imgToUse)(roi);
    } else {
        if (img.cols != templSize) {    // resize
            cv::resize(img, templ, cv::Size(templSize, templSize));
        } else {    // just copy
            img.copyTo(templ);
        }
    }
    
    // binarize, if necessary
    if (binarize) {
        cv::threshold(templ, templ, 127, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    }
    
    // create all 4 possible rotations
    cv::Mat *templArr = new cv::Mat[4];
    templ.copyTo(templArr[0]);  // first: no rotation
    for (int r = 1; r < 4; r++) {   // all other rotations
        Tools::matRot90CW(templ);
        templ.copyTo(templArr[r]);
    }
    
    // add to map
    TemplateMapPair templPair(id, templArr);
    templates.insert(templPair);
    
    printf("ocv_ar::IdentificatorTemplMatch - added template image of size %dx%d for id %d\n",
           img.cols, img.rows, id);
}

void IdentificatorTemplMatch::removeTemplateImg(int id) {
    TemplateMap::iterator it = templates.find(id);
    if (it != templates.end()) {
        // free memory
        cv::Mat *templArr = it->second;
        delete [] templArr;
        
        // remove the map entry
        templates.erase(it);
    }
}

#pragma mark private methods

bool IdentificatorTemplMatch::checkBorder(const cv::Mat &img, int dir, int off) {
    const int numCells = 8;

    // init
    int x, y;
    if (dir == 0) { // horizontal
        x = 0;
        y = off;
    } else {        // vertical
        x = off;
        y = 0;
    }
    
    // walk
    for (int c = 0; c < numCells; c++) {
        // get abs. pixel offsets for the cell
        int cellX = x * borderSize;
        int cellY = y * borderSize;
        
        // select cell
        cv::Mat cell = img(cv::Rect(cellX, cellY, borderSize, borderSize));
        
        // count non zero values
        if (cv::countNonZero(cell) > minSetMarkerPixels) {  // we have "white" cell!
//            printf("ocv_ar::IdentificatorTemplMatch - check border failed for dir %d, offset %d\n", dir, off);
            
            return false;
        }
        
        // increment
        if (dir == 0) { // horizontal
            x++;
        } else {        // vertical
            y++;
        }
    }
    
    return true;
}

bool IdentificatorTemplMatch::checkTemplateRotations(const cv::Mat &marker, const cv::Mat *templRotations, int *validRot) {
    cv::Mat errorMat(marker.rows, marker.cols, CV_8UC1);
    
    float bestResult = OCV_AR_CONF_TEMPL_MATCH_MAX_ERROR_RATE;    // init. with worst possible result
    int bestResultRot = -1;
    
    for (int r = 0; r < 4; r++) {   // check all four rotations
        // do a bitwise or so that we get "1"s where the two images do not fit
        cv::bitwise_xor(marker, templRotations[r], errorMat);
        
        // could the pixels that do not fit
        float errRate = (float)cv::countNonZero(errorMat) / (float)templSizeSq;
        
        if (errRate <= OCV_AR_CONF_TEMPL_MATCH_MAX_ERROR_RATE && errRate < bestResult) {
            // found the best result so far
            bestResult = errRate;
            bestResultRot = r;
        }
    }
    
    if (bestResultRot < 0) {
//        printf("ocv_ar::IdentificatorTemplMatch - no template match\n");
        
        return false;
    }
    

//    printf("ocv_ar::IdentificatorTemplMatch - template match(es) with best result %f, rotation %d\n",
//           bestResult, bestResultRot);
    *validRot = bestResultRot;
    
    return true;
}