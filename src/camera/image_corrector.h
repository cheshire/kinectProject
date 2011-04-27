/*
 * image_aligner.h
 *
 * The Image Aligner will takes a depth image and an RGB image and attempts to
 * align them so that each (x,y) coordinate matches.
 *
 *  Created on: 06/04/2011
 *      Author: Laurence Rouesnel
 */

#ifndef IMAGE_ALIGNER_H_
#define IMAGE_ALIGNER_H_

#include <cv.h>
#include <gflags/gflags.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "camera/image.h"
#include "camera/camera_perspective.h"

namespace camera {

class ImageCorrector {
public:
  void undistort(Image& frame);
  
  void align(Image& frame);
};

}

#endif /* IMAGE_ALIGNER_H_ */
