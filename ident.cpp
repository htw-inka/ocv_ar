#include "ident.h"

using namespace ocv_ar;

void IdentificatorBase::setFoundPropertiesForMarker(Marker &marker, int id, int rot) {
    marker.setId(id);
    marker.rotatePoints(rot);
}