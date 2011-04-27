#ifndef RGB_DEPTH_FRAME_H_
#define RGB_DEPTH_FRAME_H_

#include <ctime>
#include <cv.h>

#include "camera/camera_perspective.h"

struct Image {
  cv::Size size() {
    return rgb.size();
  }

  cv::Mat rgb;
  camera::CameraPerspective *rgb_perspective;
  cv::Mat depth;
  camera::CameraPerspective *depth_perspective;
  cv::Mat raw_depth;
  time_t timestamp;

  cv::Mat mapped_depth;
  cv::Mat mapped_rgb;
};

#endif
