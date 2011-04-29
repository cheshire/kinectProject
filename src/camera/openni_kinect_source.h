/*
 * openni_kinect_source.h
 *
 *  Created on: 28/04/2011
 *      Author: laurence
 */

#ifndef OPENNI_KINECT_SOURCE_H_
#define OPENNI_KINECT_SOURCE_H_

#include <XnOpenNI.h>
#include <XnCppWrapper.h>

#include "camera/image.h"
#include "camera/image_source.h"

namespace camera {

class OpenNIKinectSource : public ImageSource {
  bool high_resolution_mode_;

  cv::Mat current_rgb;
  cv::Mat current_depth;

  xn::Context context;
  xn::DepthGenerator depth_generator;
  xn::ImageGenerator rgb_generator;

  CameraPerspective *rgb_perspective;
  CameraPerspective *depth_perspective;

  void estimate_calibration();

public:
  OpenNIKinectSource();
  virtual ~OpenNIKinectSource();

  CameraResponse get_image(Image *frame);

  bool high_resolution_mode() const { return high_resolution_mode_; }
  void high_resolution_mode(bool set) { high_resolution_mode_ = set; }
};
}

#endif /* OPENNI_KINECT_SOURCE_H_ */
