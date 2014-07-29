/**
 * ocv_ar - OpenCV based Augmented Reality library
 *
 * Tracker class -- implementation file.
 *
 * Author: Markus Konrad <konrad@htw-berlin.de>, June 2014.
 * INKA Research Group, HTW Berlin - http://inka.htw-berlin.de/
 *
 * See LICENSE for license.
 */

#include "track.h"

#include "tools.h"

using namespace ocv_ar;

#pragma mark public methods

void Track::detect(const cv::Mat *frame) {
    assert(detector && frame);
    
    if (detectionRunning) return;   // check if detection is already running
    detectionRunning = true;
    
    // set the input frame
    detector->setInputFrame(frame, frame->channels() == 1);
    
    // detect and identify the markers
    detector->processFrame();
    
    lockMarkers();  // lock markers map
    
    // estimate the markers' 3D poses
    detector->estimateMarkersPoses();
    
    // copy the found markers to the <newMarkers> vector
    newMarkers.clear();
    vector<Marker *> foundMarkers = detector->getMarkers();
    for (vector<Marker *>::const_iterator it = foundMarkers.begin();
         it != foundMarkers.end();
         ++it)
    {
        Marker *mrk = *it;
        newMarkers.push_back(Marker(*mrk)); // create new Marker object from <mrk>
    }
    
    newMarkersFresh = true; // just detected, they're fresh!
    
    unlockMarkers();  // lock markers map
    
    // ready for new detection
    detectionRunning = false;
}

void Track::lockMarkers() {
    Threading::mutexLock();
}

void Track::unlockMarkers() {
    Threading::mutexUnlock();
}

void Track::update() {
    lockMarkers();  // lock markers map
    
    // update already existing markers
    double now = Tools::nowMs();
    for (MarkerMap::iterator it = markers.begin();
         it != markers.end();
         )  // no op -- map::erase might be called below
    {
        int existingMrkId = it->first;
        Marker *existingMrk = &it->second;
        
        bool markerUpdated = false;
        
        // try to find a matching marker in the "new markers" vector
        for (vector<Marker>::const_iterator newMrkIt = newMarkers.begin();
             newMrkIt != newMarkers.end();
             ++newMrkIt)
        {
            if (existingMrkId == newMrkIt->getId()) { // we found a matching marker
//                printf("ocv_ar::Track - updating marker %d\n", existingMrkId);
                
                // update the existing marker with the information of the "new" marker
                existingMrk->updateForTracking(*newMrkIt);
                
                // update detection time to "now"
                existingMrk->updateDetectionTime();
                
                // set status
                markerUpdated = true;
                break;
            }
        }
        
        // check if this marker was detected this time
        // and if not, if it already time out
        if (!markerUpdated && now - existingMrk->getDetectionTimeMs() > OCV_AR_CONF_TRACKER_MARKER_TIMEOUT_MS) {
            // if so, remove it!
//            printf("ocv_ar::Track - lost marker %d\n", existingMrkId);
            markers.erase(it++);    // safe map item delete
        } else {
            ++it;
        }
    }
    
    // add new markers the first time they were detected
    if (newMarkersFresh) {
        for (vector<Marker>::const_iterator newMrkIt = newMarkers.begin();
             newMrkIt != newMarkers.end();
             ++newMrkIt)
        {
            MarkerMapPair newMrkPair(newMrkIt->getId(), *newMrkIt);
            markers.insert(newMrkPair);
//            printf("ocv_ar::Track - added new marker %d\n", newMrkIt->getId());
        }
    }
    
    newMarkersFresh = false;
    
//    printf("ocv_ar::Track - tracking %lu markers now\n", markers.size());
    
    unlockMarkers();    // unlock markers map
}
