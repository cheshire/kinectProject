#include "file_source.h"

#include <iostream>
#include <unistd.h>

#include "camera/constants.h"
#include "camera/image_source.h"

using namespace std;

namespace {
  
  // Delay between the frames in milliseconds
  const double KINECT_FRAME_LATENCY = 100;
}

namespace camera {

FileSource::FileSource(const string &directory)
    : initialized(false), frame_count(0), last_time(time(NULL)), directory(directory) {
  cout << "Creating fake kinect." << endl;
}

void FileSource::initialize() {
  cout << "FakeKinect: Initializing" << endl;
  int count = 0;
  while(1) {
    string rgb_filename = cv::format(PATH_FORMAT, directory.c_str(),
        RGB_FILENAME_BASE, count, RGB_FILENAME_EXTENSION);

    string depth_filename = cv::format(PATH_FORMAT, directory.c_str(),
          DEPTH_FILENAME_BASE, count, DEPTH_FILENAME_EXTENSION);

    cout << "Attempting to read RGB from " << rgb_filename << endl;
    Mat rgb = imread(rgb_filename.c_str());
    if (rgb.data == NULL) {
      break;
    }

    rgb_frames.push_back(rgb);

    cout << "Attempting to read Depth from " << depth_filename << endl;

    FileStorage fs(depth_filename.c_str(), FileStorage::READ);
    Mat depth = Mat();

    fs["depth"] >> (depth);
    depth_frames.push_back(depth);
    count++;
  }
  
  total_frame_count = count;
  
  initialized = true;
}

CameraResponse FileSource::get_image(Image *frame) {
  if (!initialized) {
    initialize();
  }
  
  if (total_frame_count == 0){
    
    // No frames were recorded, abort.
    return NO_FRAMES;
  }

  usleep(KINECT_FRAME_LATENCY * 1000);
  
  last_time = time(NULL);

  cout << "FakeKinect: Getting Frame" << endl;

  rgb_frames[frame_count].copyTo(frame->rgb);
  depth_frames[frame_count].copyTo(frame->depth);

  frame_count = (frame_count + 1) % total_frame_count;
  return OK;
}

}
