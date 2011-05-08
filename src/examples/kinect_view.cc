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

#include <unistd.h>
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
DEFINE_int32(chessboard_width, 6, "Number of internal chessboard corners in width");
DEFINE_int32(chessboard_height, 6, "Number of internal chessboard corners in height");
DEFINE_bool(detect_circles, false, "Run the circle-detection algorithm and report on results");
DEFINE_bool(show_gui, true, "Show the images currently processed");

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

/**
 * If we can find chessboard corners in the image,
 * we will write the homography. Otherwise, return 0.
 * 
 * @param img Undistorted image. 
 * 
 * @return Whether homography was found
 */
int find_homography(Mat img){
  bool homography_found = false;
  
  // Let's see whether we can find chessboard.
  vector<cv::Point2f> chessboard_corners;
  bool found = cv::findChessboardCorners(
    img,
    cv::Size(FLAGS_chessboard_width,
      FLAGS_chessboard_height
    ),
    chessboard_corners
  );
  
  // Drawing is optional -- good for debugging
  cv::drawChessboardCorners(img, 
    cv::Size(FLAGS_chessboard_width,
      FLAGS_chessboard_height
    ),
    cv::Mat(chessboard_corners),
    found
  );
  
  // We define our coordinate center to be
  // the center. We define size of the chessboard
  // square to be the unit length.
  
  
  // Apparently the only way to do manual data entry into Mat
  // is to declare it first as Matx and then convert to Mat.
  
  // TODO(george) - generalize away from 9 corners?
  float _dst_points[2][9] = {{-1, 0, 1, -1, 0, 1, -1, 0, 1},                                                                                                 
                    {1, 1, 1, 0, 0, 0, -1, -1, -1}};  
                    
  // TODO(george) - just write out everything as vectorS!!
  cv::Mat dst_points = cv::Mat(2, 9, CV_32F, _dst_points);

  cv::Mat homography;
  homography = cv::findHomography(chessboard_corners, );
  cout << homography << endl;
  
  
/*  cv::transpose(src_points_, src_points);
  cout << chessboard_corners << endl;
  cout << "Original array is " << endl << Mat(chessboard_corners) << endl;
  cout << "Transposed array is " << endl << src_points << endl;
  
  cout << "(0, 1) element in original array: " << endl;
  cout << Mat(chessboard_corners).at<float>(0, 1) << endl;
  
  cout << "(0, 1) element in transposed array: " << endl;
  cout << src_points.at<float>(0, 1) << endl;*/
  LOG(INFO) << "Found chessboard is " << found;
  
  return homography_found;
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

  if (FLAGS_show_gui) {
    namedWindow("rgb", CV_WINDOW_AUTOSIZE);
    namedWindow("rgb_raw", CV_WINDOW_AUTOSIZE);
    namedWindow("depth", CV_WINDOW_AUTOSIZE);
    namedWindow("depth_raw", CV_WINDOW_AUTOSIZE);
  }

  while(running) {
    while (true){
      CameraResponse r = device->get_image(&frame);
      if (r == WAIT){
        usleep(200 * 1000);
        continue;
      } else if (r == NO_FRAMES) {
        LOG(ERROR) << "No files found in the given directory, aborting";
        exit(0);
      } else if (r == BROKEN_IMAGE){
        LOG(ERROR) << "Image can not be read, aborting";
        exit(0);
      } else if (r == OK){
        break;
      }
    }
        
    
    frame.depth.convertTo(temp_depth, CV_8UC1, 50);
    if (FLAGS_show_gui) {
      cv::imshow("rgb_raw", frame.rgb);
      cv::imshow("depth_raw", temp_depth);
    }

    image_corrector.undistort(frame);
    image_corrector.align(frame);
    
    find_homography(frame.mapped_rgb);
    
    frame.depth.convertTo(temp_depth, CV_8UC1, 50);
    mask = cv::Mat::zeros(frame.depth.size(), CV_8UC1);
    
    if (FLAGS_show_gui) {
      cv::imshow("rgb", frame.mapped_rgb);
      cv::imshow("depth", frame.mapped_depth);
    }

    if (recording) {
      Image *img = new Image();
      img->rgb = frame.rgb.clone();
      img->depth = frame.depth.clone();
      recorded_images.push_back(img);
    }        

    // Run the contour detection algorithm and detect
    // two black circles on lazy susan.
    if (FLAGS_detect_circles) {
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
    }

    char k = cvWaitKey(10);
    switch (k) {
      case 27:
        cvDestroyWindow("rgb");
        cvDestroyWindow("rgb_raw");
        cvDestroyWindow("depth");
        cvDestroyWindow("depth_raw");
        running = false;
        break;
      case 119: // w
        LOG(INFO) << "Recording Started.";
        recording = true;
        break;
      case 101: // e
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
