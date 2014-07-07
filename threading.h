/**
 * ocv_ar - OpenCV based Augmented Reality library
 *
 * Helper function class for threading -- header file.
 *
 * Author: Markus Konrad <konrad@htw-berlin.de>, July 2014.
 * INKA Research Group, HTW Berlin - http://inka.htw-berlin.de/
 *
 * See LICENSE for license.
 */

#ifndef OCV_AR_THREADING_H
#define OCV_AR_THREADING_H

#include <pthread.h>

namespace ocv_ar {

/**
 * Simple threading class that mainly manages a mutex variable.
 * Threading is necessary in the tracker, when its update() method
 * is called from a different thread than the detect() method.
 */
class Threading {
public:
    /**
     * Initialize the mutex.
     */
    static void init();
    
    /**
     * Lock the mutex.
     */
    static void mutexLock();
    
    /**
     * Unlock the mutex.
     */
    static void mutexUnlock();
    
private:
    static pthread_mutex_t mutex;   // pthread mutex variable
};

}

#endif