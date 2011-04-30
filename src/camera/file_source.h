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
  
  // Get the rgb filename for the given count.
  string get_rgb_filename(int count);
  
  // Get the depth filename for the given count.
  string get_depth_filename(int count);
  
  bool initialized;
  
  // Count of the current frame shown by the FileSource.
  int frame_count;
  
  // Number of the total frames found.
  int total_frame_count;

  string directory;
};
}

#endif
