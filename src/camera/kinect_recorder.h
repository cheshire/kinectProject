#ifndef KINECT_RECORDER_H
#define KINECT_RECORDER_H

#include <cv.h>
#include <highgui.h>
#include <string>

#include "camera/kinect_device.h"

using namespace cv;
namespace camera {

class KinectRecorder {
public:
  KinectRecorder();
  KinectRecorder(const string &rgb_filename_base,
      const string &depth_filename_base);

  void record(const RgbDepthFrame &frame);

private:
  int frame_count;

  string rgb_filename_base;
  string depth_filename_base;
};
}
#endif // FAKE_KINECT_H
