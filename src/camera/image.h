#ifndef RGB_DEPTH_FRAME_H_
#define RGB_DEPTH_FRAME_H_

#include <ctime>
#include <cv.h>

#include "camera/camera_perspective.h"

struct Image {

  Image() : aligned(false), undistorted(false) {}

  cv::Mat rgb;
  camera::CameraPerspective *rgb_perspective;
  cv::Mat depth;
  camera::CameraPerspective *depth_perspective;
  cv::Mat raw_depth;
  time_t timestamp;

  bool aligned;
  bool undistorted;

  cv::Mat mapped_depth;
  cv::Mat mapped_rgb;

  cv::Mat1b mask;
  cv::Mat3b masked_rgb;
  cv::Mat1f masked_depth;
};

#endif
