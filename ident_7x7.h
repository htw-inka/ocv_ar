/**
 * ocv_ar - OpenCV based Augmented Reality library
 *
 * Marker identification for ArUco style 7x7 markers -- header file.
 * Note: Although the markers use 7x7 binary fields, only the inner 5x5 fields
 * carry information (i.e. the marker code).
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

#ifndef OCV_AR_IDENT_7BIT_H
#define OCV_AR_IDENT_7BIT_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "conf.h"
#include "ident.h"
#include "types.h"
#include "marker.h"

namespace ocv_ar {
    
/**
 * Marker idenfication for ArUco style 7x7 markers.
 * A possible marker square can be analyzed and - if it is a valid marker - its ID
 * can be calculated from the code embedded in the inner 5x5 binary marker fields.
 */
class Identificator7x7 : public IdentificatorBase {
public:
    /**
     * Constructor. Create a new idenficiator of type IDENT_TYPE_CODE_7BIT.
     */
    Identificator7x7() : IdentificatorBase(IDENT_TYPE_CODE_7X7, 7 * OCV_AR_CONF_MARKER_CODE_PX_PER_FIELD),
                         markerCellSize(OCV_AR_CONF_MARKER_CODE_PX_PER_FIELD),
                         minSetMarkerPixels(OCV_AR_CONF_MARKER_CODE_PX_PER_FIELD * OCV_AR_CONF_MARKER_CODE_PX_PER_FIELD / 2)
                         {};
    
    /**
     * Try to read the marker code from the quadratic image part <area> by trying out all
     * four possbile rotations of the image. If an ID could be read, set the found ID and
     * valid rotation in <marker> and return true, otherwise return false.
     */
    virtual bool readMarkerCode(const cv::Mat &area, Marker &marker);
    
private:
    /**
     * Helper function to check if a marker code is valid when the extracted marker code
     * matrix <m> is read.
     */
    bool checkMarkerCode(const cv::Mat &m) const;
    
    /**
     * Helper function to read the marker code from the extracted marker code
     * matrix <m>.
     */
    int markerCodeToId(const cv::Mat &m) const;
    
    
    int markerCellSize;     // cell size of each marker field in pixels
    int minSetMarkerPixels; // minimum of white marker pixels that must be set if a field should be regarded "1"
};

}

#endif