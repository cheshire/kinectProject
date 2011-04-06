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

  void record(const RgbDepthFrame &frame);
private:
  int frame_count;
};
}
#endif // FAKE_KINECT_H
