#include <assert.h>
#include <iostream>

#include "kinect_recorder.h"

#include "camera/constants.h"

namespace camera {

KinectRecorder::KinectRecorder()
    : frame_count(0) {
}

void KinectRecorder::record(const RgbDepthFrame &frame) {
  string rgb_filename =
      cv::format(FILENAME_FORMAT, RGB_FILENAME_BASE, frame_count, RGB_FILENAME_EXTENSION);
  string depth_filename =
      cv::format(FILENAME_FORMAT, DEPTH_FILENAME_BASE, frame_count, DEPTH_FILENAME_EXTENSION);

  std::cout << "writing " << frame_count << std::endl;

  imwrite(rgb_filename.c_str(), frame.rgb_image);

  FileStorage fs(depth_filename.c_str(), FileStorage::WRITE);
  fs << "depth" << frame.depth_image;

  frame_count++;
}
}
