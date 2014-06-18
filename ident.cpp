/**
 * ocv_ar - OpenCV based Augmented Reality library
 *
 * Marker identification base class -- implementation file.
 *
 * Author: Markus Konrad <konrad@htw-berlin.de>, June 2014.
 * INKA Research Group, HTW Berlin - http://inka.htw-berlin.de/
 *
 * See LICENSE for license.
 */

#include "ident.h"

using namespace ocv_ar;

void IdentificatorBase::setFoundPropertiesForMarker(Marker &marker, int id, int rot) {
    marker.setId(id);
    marker.rotatePoints(rot);
}