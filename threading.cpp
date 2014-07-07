/**
 * ocv_ar - OpenCV based Augmented Reality library
 *
 * Helper function class for threading -- implementation file.
 *
 * Author: Markus Konrad <konrad@htw-berlin.de>, July 2014.
 * INKA Research Group, HTW Berlin - http://inka.htw-berlin.de/
 *
 * See LICENSE for license.
 */

#include "threading.h"

using namespace ocv_ar;

pthread_mutex_t Threading::mutex;

void Threading::init() {
    pthread_mutex_init(&mutex, NULL);
}

void Threading::mutexLock() {
    pthread_mutex_lock(&mutex);
}

void Threading::mutexUnlock() {
    pthread_mutex_unlock(&mutex);
}