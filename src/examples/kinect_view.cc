#include <cmath>
#include <cv.hpp>
#include <cxcore.h>
#include <highgui.h>
#include <iostream>
#include <libfreenect.hpp>
#include <string>
#include <vector>
#include <gflags/gflags.h>
#include <time.h>

#include "camera/abstract_rgb_depth_camera.h"
#include "camera/fake_kinect.h"
#include "camera/kinect_factory.h"
#include "camera/kinect_recorder.h"
#include "camera/rgb_depth_frame.h"

DEFINE_string(fake_kinect_data, "", "directory containing files for FakeKinect");

using namespace camera;
using namespace cv;
using namespace std;

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  bool running(true);
  RgbDepthFrame frame;
  Mat temp_depth;
  KinectFactory factory;
  KinectRecorder *recorder = NULL;
  vector<RgbDepthFrame*> *frames;
  AbstractRgbDepthCamera *device;

  cout << "Creating KinectDevice" << endl;
  if (FLAGS_fake_kinect_data.empty()) {
    device = factory.create_kinect();
  } else {
    device = factory.create_kinect(FLAGS_fake_kinect_data);
  }
  cout << "Successfully created KinectDevice" << endl;

  cout << "Creating Windows" << endl;
  namedWindow("rgb", CV_WINDOW_AUTOSIZE);
  namedWindow("depth", CV_WINDOW_AUTOSIZE);
  cout << "Created Windows" << endl;

  while(running) {
    while (!device->get_rgb_depth_frame(&frame))
    {}
    cv::imshow("rgb", frame.rgb_image);

    frame.depth_image.convertTo(temp_depth, CV_8UC1, 255.0/2048.0);

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
