#ifndef FAKE_KINECT_H
#define FAKE_KINECT_H

#include <cv.h>
#include <highgui.h>
#include <string>

#include "camera/abstract_rgb_depth_camera.h"

using namespace cv;

namespace camera {

class FakeKinect : AbstractRgbDepthCamera {
public:
  FakeKinect(const string &rgb_filename, const string &depth_filename);

  bool get_rgb_depth_frame(RgbDepthFrame *frame);

private:
  int frame_count;

  string rgb_filename_base;
  string depth_filename_base;
};
}

#endif
