/**
 * ocv_ar - OpenCV based Augmented Reality library
 *
 * Marker identification base class -- header file.
 *
 * Author: Markus Konrad <konrad@htw-berlin.de>, June 2014.
 * INKA Research Group, HTW Berlin - http://inka.htw-berlin.de/
 *
 * See LICENSE for license.
 */

#ifndef OCV_AR_IDENT_H
#define OCV_AR_IDENT_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "types.h"
#include "marker.h"

namespace ocv_ar {

/**
 * Abstract base class for all identificators providing common methods and a uniform
 * interface.
 */
class IdentificatorBase {
public:
    /**
     * Base constructor to set the identificator type <t> and the required marker size
     * <markerSize> in pixels.
     */
    IdentificatorBase(IdentificatorType t, int markerSize) : type(t),
                                                             reqMarkerSize(markerSize)
    {};
    
    /**
     * Empty virtual deconstructor.
     */
    virtual ~IdentificatorBase() {};
    
    /**
     * Abstract method to read a marker code from the image region <area> and save
     * possible results in <marker>.
     * Returns true if the code could be read, otherwise false.
     */
    virtual bool readMarkerCode(const cv::Mat &area, Marker &marker) = 0;
    
    /**
     * Returns the required marker size.
     */
    int getRequiredMarkerSize() const { return reqMarkerSize; }
    
    /**
     * Returns the idenficator type.
     */
    IdentificatorType getType() { return type; }
    
protected:
    
    IdentificatorType type; // identificator type
    int reqMarkerSize;      // required marker size
};
    
}

#endif
