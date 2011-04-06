#include "fake_kinect.h"

#include <iostream>

#include "camera/constants.h"

#define KINECT_FRAME_LATENCY 0.009

namespace camera {

FakeKinect::FakeKinect(const string &directory)
    : initialized(false), frame_count(0), last_time(time(NULL)), directory(directory) {
  std::cout << "Creating fake kinect." << std::endl;
}

void FakeKinect::initialize() {
  std::cout << "FakeKinect: Initializing" << std::endl;
  int count = 0;
  while(1) {
    string rgb_filename = cv::format(PATH_FORMAT, directory.c_str(),
        RGB_FILENAME_BASE, count, RGB_FILENAME_EXTENSION);

    string depth_filename = cv::format(PATH_FORMAT, directory.c_str(),
          DEPTH_FILENAME_BASE, count, DEPTH_FILENAME_EXTENSION);

    std::cout << "Attempting to read RGB from " << rgb_filename << std::endl;
    Mat rgb = imread(rgb_filename.c_str());

    if (rgb.data == NULL) {
      break;
    }

    rgb_frames.push_back(rgb);

    std::cout << "Attempting to read Depth from " << depth_filename << std::endl;

    FileStorage fs(depth_filename.c_str(), FileStorage::READ);
    Mat depth = Mat();

    fs["depth"] >> (depth);
    depth_frames.push_back(depth);
    count++;
  }

  total_frame_count = count;
  initialized = true;
}

bool FakeKinect::get_rgb_depth_frame(RgbDepthFrame *frame) {
  if (!initialized) {
    initialize();
  }

  if (difftime(time(NULL), last_time) < KINECT_FRAME_LATENCY) {
    // Add an artifical delay that matches the kinect (from experimental
    // evidence).
    return false;
  }
  last_time = time(NULL);

  std::cout << "FakeKinect: Getting Frame" << std::endl;

  rgb_frames[frame_count].copyTo(frame->rgb_image);
  depth_frames[frame_count].copyTo(frame->depth_image);

  frame_count = (frame_count + 1) % total_frame_count;
  return true;
}

}
