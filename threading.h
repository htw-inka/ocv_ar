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

class Threading {
public:
    static void init();
    
    static void mutexLock();
    static void mutexUnlock();
    
private:
    static pthread_mutex_t mutex;
};

}

#endif