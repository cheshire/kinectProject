#include <assert.h>
#include <iostream>
#include <sstream>

#include "kinect_recorder.h"

namespace camera {

KinectRecorder::KinectRecorder()
    : frame_count(0), rgb_filename_base("rgb"),
    depth_filename_base("depth") {
}

KinectRecorder::KinectRecorder(const string &rgb_filename_base,
    const string &depth_filename_base)
    : frame_count(0),
    rgb_filename_base(rgb_filename_base),
    depth_filename_base(depth_filename_base) {
}

void KinectRecorder::record(const RgbDepthFrame &frame) {
  stringstream rgb_filename, depth_filename;
  rgb_filename << rgb_filename_base << "-" << frame_count << ".png";
  depth_filename << depth_filename_base << "-" << frame_count << ".depth";

  std::cout << "writing " << frame_count << std::endl;

  imwrite(rgb_filename.str().c_str(), frame.rgb_image);

  FileStorage fs(depth_filename.str().c_str(), FileStorage::WRITE);
  fs << "depth" << frame.depth_image;

  frame_count++;
}
}
