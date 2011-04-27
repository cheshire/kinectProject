#ifndef KINECT_RECORDER_H
#define KINECT_RECORDER_H

#include <cv.h>
#include <highgui.h>
#include <string>

#include "camera/kinect_source.h"

using namespace cv;
namespace camera {

class FileWriter {
public:
  FileWriter();

  void record(const Image &frame);
private:
  int frame_count;
};
}
#endif // FAKE_KINECT_H
