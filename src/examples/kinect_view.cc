#include <cmath>
#include <cv.hpp>
#include <cxcore.h>
#include <highgui.h>
#include <iostream>
#include <libfreenect.hpp>
#include <string>
#include <vector>
#include <time.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include "camera/abstract_rgb_depth_camera.h"
#include "camera/fake_kinect.h"
#include "camera/kinect_factory.h"
#include "camera/kinect_recorder.h"
#include "camera/rgb_depth_frame.h"
#include "camera/image_corrector.h"

DEFINE_string(fake_kinect_data, "", "directory containing files for FakeKinect");

using namespace camera;
using namespace std;

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  bool running(true);
  RgbDepthFrame frame;
  Mat temp_depth;
  KinectFactory factory;
  ImageCorrector image_corrector;
  KinectRecorder *recorder = NULL;
  vector<RgbDepthFrame*> *frames = new vector<RgbDepthFrame*>;
  AbstractRgbDepthCamera *device;

  LOG(INFO) << "Creating KinectDevice";
  if (FLAGS_fake_kinect_data.empty()) {
    device = factory.create_kinect();
  } else {
    device = factory.create_kinect(FLAGS_fake_kinect_data);
  }
  LOG(INFO) << "Successfully created KinectDevice";

  LOG(INFO) << "Creating Windows";
  namedWindow("rgb", CV_WINDOW_AUTOSIZE);
  namedWindow("depth", CV_WINDOW_AUTOSIZE);

  while(running) {
    while (true){
      CameraResponse r = device->get_rgb_depth_frame(&frame);
      if (r == WAIT){
        continue;
      } else if (r == NO_FRAMES){
        LOG(ERROR) << "No files found in the given directory, aborting";
        exit(0);
      } else if (r == OK){
        break;
      }
    }
        
    LOG(INFO) << "undistoring";
    image_corrector.undistort(frame);
    LOG(INFO) << "aligning";
    image_corrector.align(frame);
    
    cv::imshow("rgb", frame.rgb_image);

    frame.depth_image.convertTo(temp_depth, CV_8UC1, 150);//255.0/2048.0);

    cv::imshow("depth", temp_depth);

    if (recorder != NULL) {
      RgbDepthFrame *new_frame = new RgbDepthFrame();
      frame.rgb_image.copyTo(new_frame->rgb_image);
      frame.depth_image.copyTo(new_frame->depth_image);
      frames->push_back(new_frame);
    }

    char k = cvWaitKey(10);
    switch (k) {
      case 27:
        cvDestroyWindow("rgb");
        cvDestroyWindow("depth");
        running = false;
        if (recorder != NULL) {
          delete recorder;
        }
        break;
      case 119:
        if (recorder == NULL) {
          recorder = new KinectRecorder();
          cout << "starting to record" << endl;
        }
        break;
      case 101:
        if (recorder != NULL) {
          for (int i = 0, size = frames->size(); i < size; i ++) {
            recorder->record(*(*frames)[i]);
            delete (*frames)[i];
          }
          delete recorder;
          recorder = NULL;
          cout << "finished recording" << endl;
        }
        break;
    }
  }

  return 0;
}
