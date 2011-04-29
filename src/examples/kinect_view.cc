#include <cmath>
#include <cv.hpp>
#include <cxcore.h>
#include <highgui.h>
#include <iostream>
#include <libfreenect.hpp>
#include <string>
#include <vector>
#include <stack>
#include <time.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <math.h>

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

struct EllipticalContour {
  double contour_area;
  double error;
  cv::Point2f circle_center;
  vector<cv::Point> contour;
};

struct EllipticalContourComparison {
  bool operator ()(EllipticalContour const &first, EllipticalContour const &second) {
    return first.error < second.error;
  }
};

bool create_elliptical_contour(EllipticalContour &result, int min_size = 400) {
  result.contour_area = cv::contourArea(cv::Mat(result.contour));

  // If there are less than 8 contours an ellipse cannot be fitted.
  if (result.contour_area < min_size || result.contour.size() < 6) {
    return false;
  }

  cv::RotatedRect ellipse = cv::fitEllipse(cv::Mat(result.contour));
  result.circle_center = ellipse.center;

  // Calculate the difference between the area of the ellipse and the area of
  // the contour. This should give us an approximate measure of ellipseness.
  //
  // The error is divided by the area so it is a proportion (otherwise a small area
  // would always have a smaller error than a larger area).
  result.error = abs(ellipse.size.width * ellipse.size.height * 3.14 - result.contour_area)
      / result.contour_area;

  return true;
}


void find_circles(cv::Mat &rgb_image, vector<EllipticalContour> &results, int blur_amount = 7,
    int canny_threshold1 = 125, int canny_threshold2 = 200, int canny_aperture_size = 3) {
  cv::Mat gray, edges;
  cv::cvtColor(rgb_image, gray, CV_BGR2GRAY);
  cv::GaussianBlur(gray, gray, Size(blur_amount, blur_amount), 2, 2);

  // Find edges and contours.
  vector<vector<cv::Point> > contours;
  cv::Canny(gray, edges, canny_threshold1, canny_threshold2, canny_aperture_size, false);
  cv::findContours(edges, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

  cv::Mat temp = rgb_image.clone();
  cv::Scalar color( rand()&255, rand()&255, rand()&255);
  cv::drawContours( temp, contours, -1, color, CV_FILLED);
  cv::imshow("depth", temp);

  for (size_t i = 0, len = contours.size(); i < len; i++) {
    EllipticalContour result;
    result.contour = contours[i];

    if (create_elliptical_contour(result)) {
      results.push_back(result);
    }
  }

  sort(results.begin(), results.end(), EllipticalContourComparison());
}

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
  FileWriter writer;
  bool recording = false;
  vector<Image*> recorded_images;

  LOG(INFO) << "Creating KinectDevice";
  if (FLAGS_fake_kinect_data.empty()) {
    device = factory.create_openni_kinect();
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

    if (recording) {
      Image *img = new Image();
      img->rgb = frame.rgb.clone();
      img->depth = frame.depth.clone();
      recorded_images.push_back(img);
    }

    //RGB Experiments
    frame.mapped_rgb.copyTo(temp_rgb);

    vector<EllipticalContour> contours;
    find_circles(frame.mapped_rgb, contours);

    LOG(INFO) << "Found " << contours.size() << " ellipses.";

    for (size_t i = 0; i < 5 && i < contours.size(); i++) {
      cv::Scalar color( rand()&255, rand()&255, rand()&255);
      vector<vector<cv::Point> > single_contour;
      single_contour.push_back(contours[i].contour);
      cv::drawContours( temp_rgb, single_contour, -1, color, CV_FILLED);
      LOG(INFO) << "Contour: size->" << contours[i].contour_area << " error->" << contours[i].error;
    }

  // Apply a threshold.
    cv::imshow("rgb", temp_rgb);

    //cv::imshow("depth", frame.mapped_rgb);

    char k = cvWaitKey(10);
    switch (k) {
      case 27:
        cvDestroyWindow("rgb");
        cvDestroyWindow("rgb_raw");
        cvDestroyWindow("depth");
        cvDestroyWindow("depth_raw");
        running = false;
        break;
      case 119:
        LOG(INFO) << "Recording Started.";
        recording = true;
        break;
      case 101:
        if (!recording) {
          break;
        }
        LOG(INFO) << "Recording Stopped.";
        LOG(INFO) << "Writing to disk.";
        for (size_t i = 0, len = recorded_images.size(); i < len; i++) {
          writer.record(*recorded_images[i]);
          delete recorded_images[i];
        }
        recorded_images.clear();
        LOG(INFO) << "Finished writing.";
        recording = false;
        break;
    }
  }

  return 0;
}
