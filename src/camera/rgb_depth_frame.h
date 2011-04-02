#ifndef RGB_DEPTH_FRAME_H_
#define RGB_DEPTH_FRAME_H_

#include <ctime>
#include <cv.h>

using namespace cv;

struct RgbDepthFrame {
  RgbDepthFrame() :
    rgb_image(Size(640, 480), CV_8UC3),
    depth_image(Size(640, 480), CV_16UC1) {}

  Mat rgb_image;
  Mat depth_image;
  time_t timestamp;
};

#endif
