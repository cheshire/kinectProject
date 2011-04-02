#include <cmath>
#include <cv.hpp>
#include <cxcore.h>
#include <highgui.h>
#include <libfreenect.hpp>

#include "camera/kinect_device.h"
#include "camera/kinect_factory.h"
#include "camera/rgb_depth_frame.h"

using namespace cv;
using namespace camera;

int main() {
  bool running(true);
  RgbDepthFrame frame;
  KinectFactory factory;
  KinectDevice *device = factory.create_kinect();

  namedWindow("rgb", CV_WINDOW_AUTOSIZE);
  namedWindow("depth", CV_WINDOW_AUTOSIZE);

  while(running) {
    device->get_rgb_depth_frame(&frame);
    cv::imshow("rgb", frame.rgb_image);
    cv::imshow("depth", frame.depth_image);

    char k = cvWaitKey(10);
    switch (k) {
      case 27:
        delete device;
        cvDestroyWindow("rgb");
        cvDestroyWindow("depth");
        running = false;
        break;
    }
  }

  return 0;
}
