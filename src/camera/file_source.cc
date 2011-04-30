#include "file_source.h"

#include <iostream>
#include <unistd.h>
#include <glog/logging.h>

#include "camera/constants.h"
#include "camera/image_source.h"

using namespace std;

namespace {
  
  // Delay between the frames in milliseconds
  const double KINECT_FRAME_LATENCY = 100;    
}

namespace camera {

FileSource::FileSource(const string &directory) :
        initialized(false),
        frame_count(0),
        total_frame_count(0),
        directory(directory) {
          
  LOG(INFO) << "Creating fake kinect.";
}

/**
 * Figure out the number of recorded frames and write
 * it to the total_frame_count.
 */
void FileSource::initialize() {
  LOG(INFO) << "FakeKinect: Initializing";
  while(1) {
    string rgb_filename = get_rgb_filename(total_frame_count);
    string depth_filename = get_depth_filename(total_frame_count);
    
    // Check that both files exist.
    cout << rgb_filename << endl;
    cout << depth_filename << endl;
    if(access(rgb_filename.c_str(), F_OK) != -1 && access(depth_filename.c_str(), F_OK) != -1) {
      
      total_frame_count++;
    } else {
        break;
    }
  }
    
  initialized = true;
}

/**
 * Read & show the image from HDD.
 */
CameraResponse FileSource::get_image(Image *frame) {
  if (!initialized) {
    initialize();
  }
  cout << "Total frame count is " << total_frame_count << endl;
  if (total_frame_count == 0){
    
    // No frames were recorded, abort.
    return NO_FRAMES;
  }

  usleep(KINECT_FRAME_LATENCY * 1000);
  
  string rgb_filename = get_rgb_filename(frame_count);
  string depth_filename = get_depth_filename(frame_count);
  
  LOG(INFO) << "Attempting to read RGB from " << rgb_filename;
  
  Mat rgb = imread(rgb_filename.c_str());
  if (rgb.data == NULL) {
    
    // Something is wrong, aborting.
    return BROKEN_IMAGE;
  }

  LOG(INFO) << "Attempting to read Depth from " << depth_filename;
  FileStorage fs(depth_filename.c_str(), FileStorage::READ);
  Mat depth = Mat();
  fs["depth"] >> (depth);
  
  frame_count++;
  frame->mapped_rgb = frame->rgb = rgb;
  frame->mapped_depth = frame->depth = depth;

  // TODO(laurencer): encode values in the files somehow.
  frame->aligned = true;
  frame->undistorted = true;

  frame_count = (frame_count + 1) % total_frame_count;
  return OK;
}

/**
 * Get the filename of the image file. 
 */
string FileSource::get_rgb_filename(int count){
  return cv::format(PATH_FORMAT,
                    directory.c_str(),
                    RGB_FILENAME_BASE,
                    count,
                    RGB_FILENAME_EXTENSION);
}

/**
 * Get the filename of the depth file.
 */
string FileSource::get_depth_filename(int count){
  return cv::format(PATH_FORMAT,
                    directory.c_str(),
                    DEPTH_FILENAME_BASE,
                    count,
                    DEPTH_FILENAME_EXTENSION);
}

}
