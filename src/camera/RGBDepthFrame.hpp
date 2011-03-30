#include <ctime>
#include "cv.h"

using namespace cv;

struct RGBDepthFrame {
  RGBDepthFrame() :
    rgbImage(Size(640, 480), CV_8UC3),
    depthImage(Size(640, 480), CV_16UC1) {}
  Mat rgbImage;
  Mat depthImage;
  time_t timestamp;
};
