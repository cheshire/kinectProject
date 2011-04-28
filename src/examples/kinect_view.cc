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

#include "camera/image_source.h"
#include "camera/file_source.h"
#include "camera/kinect_factory.h"
#include "camera/file_writer.h"
#include "camera/image.h"
#include "camera/image_corrector.h"
#include "camera/background_filter.h"

#include "mesh/mesh_viewer.h"
#include "mesh/point_cloud.h"

DEFINE_string(fake_kinect_data, "", "directory containing files for FakeKinect");

using namespace camera;
using namespace mesh;
using namespace std;

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  LOG(INFO) << "Initialized Google Addins.";

  bool running(true);
  Image frame;
  Mat temp_depth, temp_rgb, mask;
  KinectFactory factory;
  ImageCorrector image_corrector;
  ImageSource *device;

  LOG(INFO) << "Creating KinectDevice";
  if (FLAGS_fake_kinect_data.empty()) {
    device = factory.create_kinect();
  } else {
    device = factory.create_kinect(FLAGS_fake_kinect_data);
  }
  LOG(INFO) << "Successfully created KinectDevice";

  LOG(INFO) << "Creating Windows";
  namedWindow("rgb", CV_WINDOW_AUTOSIZE);
  namedWindow("rgb_raw", CV_WINDOW_AUTOSIZE);
  namedWindow("depth", CV_WINDOW_AUTOSIZE);
  namedWindow("depth_raw", CV_WINDOW_AUTOSIZE);



  while(running) {
    while (true){
      CameraResponse r = device->get_image(&frame);
      if (r == WAIT){
        continue;
      } else if (r == NO_FRAMES){
        LOG(ERROR) << "No files found in the given directory, aborting";
        exit(0);
      } else if (r == OK){
        break;
      }
    }
    cv::imshow("rgb_raw", frame.rgb);
    frame.depth.convertTo(temp_depth, CV_8UC1, 50);
    cv::imshow("depth_raw", temp_depth);

    image_corrector.undistort(frame);
    image_corrector.align(frame);
    
    frame.depth.convertTo(temp_depth, CV_8UC1, 50);
    mask = cv::Mat::zeros(frame.depth.size(), CV_8UC1);



    cv::MatIterator_<char> mask_it = mask.begin<char>(),
        mask_it_end = mask.end<char>();
    cv::MatIterator_<float> depth_it = frame.mapped_depth.begin<float>(),
        depth_it_end = frame.mapped_depth.end<float>();

    for (; mask_it != mask_it_end && depth_it != depth_it_end; ++mask_it, ++depth_it) {
      if ((*depth_it) > 1.0 || (*depth_it) < 0.4) {
        *mask_it = saturate_cast<char>(0);
      } else {
        *mask_it = saturate_cast<char>(1);
      }
    }


    // Apply a threshold.
    temp_rgb = cv::Mat::zeros(frame.rgb.size(), CV_8UC3);
    frame.mapped_rgb.copyTo(temp_rgb, mask);
    cv::imshow("rgb", temp_rgb);

    mask *= 255;
    cv::imshow("depth", mask);

    BackgroundFilter filter(frame);
    filter.filter(frame, mask, temp_depth, mask);


    //cv::imshow("depth", temp_depth);

    PointCloud pc;
    pc.add_image(frame, cv::Mat1b(frame.rgb.size()));

    char k = cvWaitKey(10);
    switch (k) {
      case 27:
        cvDestroyWindow("rgb");
        cvDestroyWindow("rgb_raw");
        cvDestroyWindow("depth");
        running = false;
        break;
      case 119:
        break;
      case 101:
        break;
    }
  }

  return 0;
}
