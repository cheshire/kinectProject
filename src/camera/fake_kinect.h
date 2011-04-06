#ifndef FAKE_KINECT_H
#define FAKE_KINECT_H

#include <cv.h>
#include <highgui.h>
#include <string>
#include <time.h>
#include <vector>

#include "camera/abstract_rgb_depth_camera.h"

using namespace std;
using namespace cv;

namespace camera {

class FakeKinect : public AbstractRgbDepthCamera {
public:
  FakeKinect(const string &directory);

  bool get_rgb_depth_frame(RgbDepthFrame *frame);

private:
  void initialize();

  bool initialized;
  int frame_count;
  int total_frame_count;
  time_t last_time;

  string directory;

  vector<Mat> rgb_frames;
  vector<Mat> depth_frames;
};
}

#endif
