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

void Track::detect(const cv::Mat *frame) {
    assert(detector && frame);
    detector->setInputFrame(frame, frame->channels() == 1);
    
    detector->processFrame();
}

void Track::lockMarkers() {
    while (markersLocked) {};
    markersLocked = true;
}

void Track::unlockMarkers() {
    markersLocked = false;
}

void Track::update() {
    // get the vector of detected markers
    vector<Marker *> newMarkers = detector->getMarkers();
    
    // update already existing markers
    lockMarkers();
    double now = Tools::nowMs();
    for (MarkerMap::iterator it = markers.begin();
         it != markers.end();
         )
    {
        int existingMrkId = it->first;
        Marker *existingMrk = &it->second;
        
        bool markerUpdated = false;
        
        // try to find a matching marker in the "new markers" vector
        for (vector<Marker *>::const_iterator newMrkIt = newMarkers.begin();
             newMrkIt != newMarkers.end();
             )
        {
            const Marker *newMrk = *newMrkIt;
            if (existingMrkId == newMrk->getId()) { // we found a matching marker
//                printf("ocv_ar::Track - updating marker %d\n", existingMrkId);
                
                // update the existing marker with the information of the "new" marker
                existingMrk->updatePoseMat(newMrk->getRVec(), newMrk->getTVec(), true);
                
                // update detection time to "now"
                existingMrk->updateDetectionTime();
                
                // delete this marker from the "new markers" vector
                newMrkIt = newMarkers.erase(newMrkIt);
                
                // set status
                markerUpdated = true;
                break;
            } else {
                ++newMrkIt; // advance
            }
        }
        
        // check if this marker was detected this time and if not, if it already time out
        if (!markerUpdated && now - existingMrk->getDetectionTimeMs() > OCV_AR_CONF_TRACKER_MARKER_TIMEOUT_MS) {
            // if so, remove it!
            printf("ocv_ar::Track - lost marker %d\n", existingMrkId);
            markers.erase(it++);    // safe map item delete
        } else {
            ++it;
        }
    }
    
    // add new markers
    for (vector<Marker *>::const_iterator newMrkIt = newMarkers.begin();
         newMrkIt != newMarkers.end();
         ++newMrkIt)
    {
        const Marker *newMrk = *newMrkIt;
        MarkerMapPair newMrkPair(newMrk->getId(), Marker(*newMrk));
        markers.insert(newMrkPair);
        printf("ocv_ar::Track - added new marker %d\n", newMrk->getId());
    }
    
    unlockMarkers();
}