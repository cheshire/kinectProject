#include <sstream>

#include "fake_kinect.h"

namespace camera {

FakeKinect::FakeKinect(const string &rgb_filename,
    const string &depth_filename) 
    : frame_count(0), rgb_filename_base(rgb_filename),
    depth_filename_base(depth_filename) {}

bool FakeKinect::get_rgb_depth_frame(RgbDepthFrame *frame) {
  stringstream rgb_filename, depth_filename;

  rgb_filename << rgb_filename_base << "-" << frame_count << ".png";
  depth_filename << depth_filename_base << "-" << frame_count << ".depth";

  Mat rgb = imread(rgb_filename.str().c_str());

  if (rgb.data == NULL) {
    return false;
  }

  rgb.copyTo(frame->rgb_image);

  FileStorage fs(depth_filename.str().c_str(), FileStorage::READ);
  Mat depth;

  fs["depth"] >> depth;
  depth.copyTo(frame->depth_image);

  frame_count++;
  return true;
}

}
