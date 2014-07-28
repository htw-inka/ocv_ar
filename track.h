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
#include "threading.h"

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
    /**
     * Constructor. Pass a pointer to a detector instance <detectorPtr>.
     */
    Track(Detect *detectorPtr) : newMarkersFresh(false), detectionRunning(false),
                                 detector(detectorPtr)
    {
        Threading::init();
    };
    
    /**
     * Detect markers in <frame>.
     */
    void detect(const cv::Mat *frame);
    
    /**
     * Update 3D poses of the detected markers.
     */
    void update();
    
    /**
     * Lock the detected markers data. This is importent when detect(), update()
     * and/or drawing of the markers is done in different threads.
     */
    void lockMarkers();
    
    /**
     * Unlock the detected markers data.
     */
    void unlockMarkers();
    
    /**
     * Return a pointer to the detected markers map.
     * The mapping is marker id -> Marker object.
     */
    const MarkerMap *getMarkers() const { return &markers; }
    
private:
    std::vector<Marker> newMarkers; // vector that holds Marker objects from the
                                    // last time when detect() was called
    bool newMarkersFresh;   // is true if detect() was called before and is set
                            // to false in update()
    MarkerMap markers;      // markers to be tracked. mapping is
                            // marker id -> Marker object
    bool detectionRunning;  // is true while detect() is running
    
    Detect *detector;   // weak ref to Detect object
};

}

#endif