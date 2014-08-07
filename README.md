# ocv_ar - OpenCV based Augmented Reality library

*Markus Konrad <konrad@htw-berlin.de>, June 2014*

*INKA Research Group / Project MINERVA, HTW Berlin - http://inka.htw-berlin.de/inka/projekte/minerva/*

This project implements a simple but powerful Augmented Reality solution based on [OpenCV](http://opencv.org/). It is based on the explainations from the book *Mastering OpenCV with Practical Computer Vision Projects* (Baggio et al., 2012) and borrows some ideas and code from [ArUco](http://sourceforge.net/projects/aruco/). It is written in C++ and has no dependencies besides the already mentioned OpenCV.

## Features

* fast, lightweight and portable library in standard C++
* depends only on OpenCV library
* different marker identification possibilities:
 * ArUco-style 7x7 code markers
 * custom (binary) markers via template matching
* marker tracking with marker pose interpolation for smooth marker motions (see [this example project](https://github.com/htw-inka/ocv_ar-examples/tree/master/examples/ios/OcvARBasicNativeCam))
* well documented code and example projects (see below)

## Usage and example projects

The library usage is best explained with the well documented example projects, which are available for different plattforms in a separate [github repository 'ocv_ar-examples'](https://github.com/htw-inka/ocv_ar-examples).

The library is also part of [ocv_ar-cocos2d](https://github.com/htw-inka/ocv_ar-cocos2d), a mobile augmented reality framework for iOS that uses [Cocos2D](http://www.cocos2d-swift.org/) for 3D visualization and user interaction.

## Acknoledgements and Contributions

Thanks to Alexander Godoba for the 3D pose smoothing functions (especially the 3D rotation interpolation functions).
