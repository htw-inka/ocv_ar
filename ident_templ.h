/**
 * ocv_ar - OpenCV based Augmented Reality library
 *
 * Marker identification via simple template matching of binary
 * images -- header file.
 *
 * Author: Markus Konrad <konrad@htw-berlin.de>, June 2014.
 * INKA Research Group, HTW Berlin - http://inka.htw-berlin.de/
 *
 * See LICENSE for license.
 */

#ifndef OCV_AR_IDENT_TEMPL_H
#define OCV_AR_IDENT_TEMPL_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "conf.h"
#include "ident.h"
#include "types.h"
#include "marker.h"

namespace ocv_ar {

/**
 * Marker idenfication via simple template matching of binary images.
 * The binarized image of a possible marker square is matched up with images
 * in a "template library", which contains all valid binary marker images.
 * Note that each marker image requires to have a surrounding border around that
 * has a width of 1/8th of the overall image size.
 */
class IdentificatorTemplMatch : public IdentificatorBase {
public:
    /**
     * Constructor. Create a new idenficiator of type IDENT_TYPE_TEMPL_MATCH.
     */
    IdentificatorTemplMatch();
    
    /**
     * Deconstructor. Will free memory for the images in <templates>.
     */
    virtual ~IdentificatorTemplMatch();
    
    /**
     * Check if <area> may contain be a valid marker (check its black border). Then
     * match the binary image inside the borders against all images in <templates> by
     * performing a binary xor operation for all possible rotations and choosing the
     * best matching rotation beneath a certain threshold OCV_AR_CONF_TEMPL_MATCH_MAX_ERROR_RATE.
     * If a matching template could be read, set the found ID and
     * valid rotation in <marker> and return true, otherwise return false.
     */
    virtual bool readMarkerCode(const cv::Mat &area, Marker &marker);
    
    /**
     * Add a template image <img> (associated with ID <id>) to the template library.
     * If <stripBorder> is true, a border consuming 1/8th of the image width on each side
     * will be cut away. If <binarize> is true, the image will be thresholded using Otsu.
     */
    void addTemplateImg(int id, const cv::Mat &img, bool stripBorder = true, bool binarize = false);
    
    /**
     * Remove template image associated with ID <id> from the template library.
     */
    void removeTemplateImg(int id);
    
private:
    /**
     * Helper function to match the image <marker> against the templates with 4 possible rotations
     * in <templRotations>. Return true if the best result underneath a certain threshold
     * OCV_AR_CONF_TEMPL_MATCH_MAX_ERROR_RATE could be found and will set the correct rotation in
     * <validRot>. Otherwise returns false.
     */
    bool checkTemplateRotations(const cv::Mat &marker, const cv::Mat *templRotations, int *validRot);
    
    /**
     * Check if the one border in <img>, identified by direction <dir> (0 = horizontal, 1 = vertical)
     * and an offset <off> for the second coordinate, is valid (all black). Returns true if this is
     * the case, otherwise false.
     */
    bool checkBorder(const cv::Mat &img, int dir, int off);
    
    
    int borderSize;             // border size in pixels
    int templSize;              // template size in pixels
    int templSizeSq;            // squared template size in pixels
    int minSetMarkerPixels;     // minimum of white marker pixels that must be set if a field should be regarded "1"
    TemplateMap templates;      // template library that maps an int id to an array cv::Mat[4] with all 4 rotations
                                // of a template image
};

}

#endif