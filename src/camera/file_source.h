#ifndef FAKE_KINECT_H
#define FAKE_KINECT_H

#include <cv.h>
#include <highgui.h>
#include <string>
#include <time.h>
#include <vector>

#include "camera/image_source.h"

using namespace std;
using namespace cv;

namespace camera {

class FileSource : public ImageSource {
public:
  FileSource(const string &directory);

  CameraResponse get_image( Image* frame);

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
