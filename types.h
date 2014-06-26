/**
 * ocv_ar - OpenCV based Augmented Reality library
 *
 * Common type definitions.
 *
 * Author: Markus Konrad <konrad@htw-berlin.de>, June 2014.
 * INKA Research Group, HTW Berlin - http://inka.htw-berlin.de/
 *
 * See LICENSE for license.
 */

#ifndef OCV_AR_TYPES_H
#define OCV_AR_TYPES_H

#include <vector>
#include <map>

#include <opencv2/core/core.hpp>

namespace ocv_ar {
    /**
     * Flip mode for projection matrix
     */
    typedef enum _FlipMode {
        FLIP_NONE,
        FLIP_H, // horizontal flip
        FLIP_V  // vertical flip
    } FlipMode;
    
    /**
     * Frame processing levels in the marker detection pipeline
     */
    typedef enum _FrameProcLevel {
        PROC_LEVEL_DEFAULT = -1,
        PROC_LEVEL_PREPROC,
        PROC_LEVEL_THRESH,
        PROC_LEVEL_CONTOURS,
        PROC_LEVEL_POSS_MARKERS,        // possible marker candidates
        PROC_LEVEL_DETECTED_MARKERS
    } FrameProcLevel;
    
    /**
     * Identificator types
     */
    typedef enum _IdentificatorType {
        IDENT_TYPE_NONE = -1,
        IDENT_TYPE_CODE_7X7,
        IDENT_TYPE_TEMPL_MATCH
//        CODE_8BIT,
    } IdentificatorType;
    
    /**
     * Different point vectors
     */
    typedef std::vector<cv::Point> PointVec;
    typedef std::vector<cv::Point2f> Point2fVec;
    typedef std::vector<cv::Point3f> Point3fVec;
    
    /**
     * A vector of a point vector
     */
    typedef std::vector<PointVec> ContourVec;
    
    /**
     * Template map
     */
    typedef std::map<int, cv::Mat*> TemplateMap;    // maps id to array of 4 rotated templates
    typedef std::pair<int, cv::Mat*> TemplateMapPair;
}

#endif