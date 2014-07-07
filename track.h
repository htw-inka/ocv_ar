/**
 * ocv_ar - OpenCV based Augmented Reality library
 *
 * Tracker class -- header file.
 *
 * Author: Markus Konrad <konrad@htw-berlin.de>, June 2014.
 * INKA Research Group, HTW Berlin - http://inka.htw-berlin.de/
 *
 * See LICENSE for license.
 */

#ifndef OCV_AR_TRACK_H
#define OCV_AR_TRACK_H

#include <map>
#include <vector>

#include <opencv2/core/core.hpp>

#include "types.h"
#include "detect.h"

namespace ocv_ar {
    
/**
 * Marker map. Holds one marker mapped to a marker id.
 * Note that this means that only unique markers in a
 * scene can be tracked for now!
 */
typedef std::map<int, Marker> MarkerMap;
typedef std::pair<int, Marker> MarkerMapPair;

/**
 * Marker tracker.
 *
 * Note that that only unique markers in a scene can be tracked for now!
 * This means that there should not be two or more markers with the same id
 * in the scene.
 */
class Track {
public:
    Track(Detect *detectorPtr) : detector(detectorPtr),
                                 detectionRunning(false),
                                 newMarkersFresh(false)
    {};
    
    void detect(const cv::Mat *frame);
    
    void update();
    
    void lockMarkers();
    void unlockMarkers();
    
    const MarkerMap *getMarkers() const { return &markers; }
    
private:
    void correctMarkerVertexOrder(std::vector<Marker *> newMarkers);
    
    std::vector<Marker> newMarkers;
    bool newMarkersFresh;
    MarkerMap markers;
    
    bool detectionRunning;
    
    Detect *detector;   // weak ref to Detect object
};

}

#endif