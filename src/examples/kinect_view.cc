#include <cmath>
#include <unistd.h>
#include <iostream>

#include <string>
#include <vector>
#include <stack>
#include <time.h>

#include <libfreenect.hpp>

#include <cv.hpp>
#include <cxcore.h>
#include <highgui.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "camera/image_source.h"
#include "camera/file_source.h"
#include "camera/kinect_factory.h"
#include "camera/file_writer.h"
#include "camera/image.h"
#include "camera/image_corrector.h"
#include "camera/background_filter.h"

#include "orientation_detector/orientation_detector.h"

#include "mesh/mesh_viewer.h"
#include "mesh/point_cloud.h"

DEFINE_string(fake_kinect_data, "", "directory containing files for FakeKinect");
DEFINE_int32(chessboard_width, 3, "Number of internal chessboard corners in width");
DEFINE_int32(chessboard_height, 3, "Number of internal chessboard corners in height");
DEFINE_bool(show_gui, true, "Show the images currently processed");

DEFINE_double(canny_threshold1, 100, "Min threshold for Canny function");
DEFINE_double(canny_threshold2, 200, "Max threshold for Canny function");
DEFINE_int32(canny_aperture_size, 25, "Size of the aperture for Canny function");
DEFINE_int32(blur_amount, 1, "Amount of blur to apply");

using namespace camera;
using namespace std;

namespace { 
  cv::Scalar MY_COLOR(100);
}

// Forward declarations, see below for comments.
int find_homography(const cv::Mat img,
                    cv::Mat visualisation,
                    cv::Mat& homography);

Image get_frame(ImageSource *device, ImageCorrector *corrector);


struct EllipticalContour {
  double contour_area;
  double error;
  cv::Point2f circle_center;
  vector<cv::Point> contour;
};

struct EllipticalContourComparison {
  bool operator ()(EllipticalContour const &first,
                    EllipticalContour const &second) {
    if (first.error == -1) {
      return false;
    }
    if (second.error == -1) {
      return true;
    }
    return (first.error < second.error);
  }
};


bool create_elliptical_contour(EllipticalContour& result,
                              int min_size = 800, // 500
                              double min_circularity = 0.1) { // 0.55
  result.contour_area = cv::contourArea(cv::Mat(result.contour));

  // If there are less than 8 contours an ellipse cannot be fitted.
  
  // was .size() < 6
  if (result.contour_area < min_size || result.contour.size() < 1) {
    return false;
  }

  float circumference = cv::arcLength(cv::Mat(result.contour), true);
  result.error = (circumference == 0) ? -1 : (float) (4 * M_PI * result.contour_area / pow(circumference, 2));

  cv::RotatedRect ellipse = cv::fitEllipse(cv::Mat(result.contour));
  result.circle_center = ellipse.center;

  return result.error > min_circularity;
}

/*
 * New idea - find the biggest ellipse in rgb space,
 * find circle of interest (ellipse with the same ratio taking some 
 * set proportion of the area (OR we can use the homography matrix
 * and positions of the centers of the circles to do that)

 */

vector<EllipticalContour> find_ellipses(const cv::Mat rgb_image,
  Mat& visualisation,
  int blur_amount = 1,
  int canny_threshold1 = 300,
  int canny_threshold2 = 350, 
  int canny_aperture_size = 7){
  
  vector<EllipticalContour> results;
  
  cv::Mat gray, edges;
//   cv::cvtColor(rgb_image, gray, CV_BGR2GRAY);
//   cv::GaussianBlur(gray, gray, Size(blur_amount, blur_amount), 2, 2);

  // Find edges and contours.
  vector<vector<cv::Point> > contours;
  
  
  rgb_image.copyTo(gray);
  cv::GaussianBlur(gray, gray, Size(blur_amount, blur_amount), 2, 2);
  
  cv::Canny(gray,edges, canny_threshold1, canny_threshold2,
            canny_aperture_size, false);     
  
  cv::findContours(edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);  

  for (size_t i = 0, len = contours.size(); i < len; i++) {
    EllipticalContour result;
    result.contour = contours[i];
    bool contour_created = create_elliptical_contour(result);
    
    if (contour_created) {
      LOG(INFO) << "Contour created";
      cv::drawContours(gray, contours, i, cv::Scalar(255), CV_FILLED);

      results.push_back(result);
    }
  }
  
  cv::drawContours(gray, contours, -1, cv::Scalar(120), CV_FILLED);
 
  gray.copyTo(visualisation);

  sort(results.begin(), results.end(), EllipticalContourComparison());
  return results;  
}


/**
 * Try to find ellipse of the turntable.
 */
bool find_turntable_ellipse(const cv::Mat depth_image,
                            cv::Mat& visualisation){
    // depth_image - Mat1f
    // rgb_image - Mat3b
    Mat1b converted_matrix;
    Mat(depth_image * 255).convertTo(converted_matrix, CV_8U);

//     vector<Mat> merged_frames;
//     for (int i=0; i<3; i++) {
//       merged_frames.push_back(float_matrix);
//     }
//     Mat merged_depth_image;
//     cv::merge(merged_frames, merged_depth_image);
    
    find_ellipses(converted_matrix, visualisation,
      FLAGS_blur_amount, 
      (int)FLAGS_canny_threshold1,
      (int)FLAGS_canny_threshold2,
      (int)FLAGS_canny_aperture_size
    );
    
    return true;
}


int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
    
  bool running(true);
  bool homography_found(false);
  bool recording(false);
  
  Image frame;
  Mat homography;  
  Mat visualisation;
  Mat depth_visualisation;
  
  float center_depth = 0;

  KinectFactory factory;
  ImageCorrector *image_corrector;
  ImageSource *device;
  FileWriter writer;
  BackgroundFilter filter;
  orientationdetector::OrientationDetector orientation_detector;
  orientation_detector.initialize();
  
  vector<Image*> recorded_images;
  float orientation_angle;
    
  LOG(INFO) << "Creating KinectDevice";
  if (FLAGS_fake_kinect_data.empty()) {
    device = factory.create_openni_kinect();
  } else {
    device = factory.create_kinect(FLAGS_fake_kinect_data);
  }
  
  LOG(INFO) << "Successfully created KinectDevice";

  if (FLAGS_show_gui) {
    cv::namedWindow("rgb", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("depth", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("visualisation", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("depth_visualisation", CV_WINDOW_AUTOSIZE);
  }

  std::cout << "Chessboard calibration is required to figure out the "
        << "lazy susan orientation. Camera should be facing at empty" 
        << " turntable with the chessboard on it."<< endl;

  while(running) {
    
    frame = get_frame(device, image_corrector);
    if (recording) {      
      Image *img = new Image();
      img->rgb = frame.rgb.clone();
      img->depth = frame.depth.clone();
      writer.record(*img);
      delete img;
    }
    
    // Any visualisations on top of rgb image.
    frame.mapped_rgb.copyTo(visualisation);
    frame.mapped_depth.copyTo(depth_visualisation);
    
    find_turntable_ellipse(frame.mapped_depth, depth_visualisation);
    
    if (!homography_found) {
      frame.masked_rgb = frame.mapped_rgb;
      frame.masked_depth = frame.mapped_depth;
      homography_found = find_homography(frame.mapped_rgb,
                                         visualisation, homography);
      if (homography_found) {
        cout << "Homography was found, chessboard calibration is no" 
            << " longer required. Please place an object on lazy susan"
            << " and press any key to continue" << endl;
        cout << homography << endl;
        cvWaitKey(0);
      }
    }
  
    if (homography_found) {
      if (center_depth == 0) {
        // If we don't have a depth for the chessboard, calculate it.
        Point center = orientation_detector.get_chessboard_centre_in_image(homography);

        center_depth = frame.mapped_depth.at<float>(center.y, center.x);
        frame.masked_rgb = frame.mapped_rgb;
        frame.masked_depth = frame.mapped_depth;
      } else {
        // If we do have a depth for the chessboard, filter it.
        filter.filter(frame, center_depth - 0.2, center_depth + 0.2);
      }
      bool status = orientation_detector.find_orientation_angle(homography,
        frame.masked_rgb,
        visualisation,
        orientation_angle
      );
      if (!status) {
        // Can't procede without orientation angle.
//         LOG(INFO) << "Skipping incorrect frame";
      } else {
//         LOG(INFO) << "ANGLE: " << orientation_angle;
      }

    }

    if (FLAGS_show_gui) {
      cv::imshow("rgb", frame.masked_rgb);
      cv::imshow("depth", frame.mapped_depth);
      cv::imshow("visualisation", visualisation);
      cv::imshow("depth_visualisation", depth_visualisation);
    }

    char k = cvWaitKey(10);
    switch (k) {
      case 27: // Esc
        cvDestroyWindow("rgb");
        cvDestroyWindow("depth");
        cvDestroyWindow("visualisation");
        running = false;
        break;
      case 119: // w
        LOG(INFO) << "Recording Started.";
        recording = true;
        break;
      case 101: // e
        recording = false;
        break;
    }
  }

  return 0;
}

/**
 * Get the frame from the device provided.
 * 
 * @param device Device to get the data from.
 * @param corrector Corrector object which aligns/undistorts
 * rgb_image and depth.
 * 
 * @return undistorted and aligned frame.
 */
Image get_frame(ImageSource *device, ImageCorrector *corrector){
  Image frame;
  
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
  
  corrector->undistort(frame);
  corrector->align(frame);
  
  return frame;
}

/**
 * Finds the homography matrix between the camera plane and the
 * chessboard plane.
 * 
 * @param img Undistorted image. 
 * @param homography Output param to write homography into.
 * @param visualisation Canvas to draw any visualisations upon.
 * 
 * @return Whether homography was found
 */
int find_homography(const cv::Mat img,
                    cv::Mat visualisation,
                    cv::Mat& homography){
  vector<Point2f> dst_points;
  
  // Let's see whether we can find chessboard.
  vector<cv::Point2f> chessboard_corners;
  bool found = cv::findChessboardCorners(
    img,
    cv::Size(FLAGS_chessboard_width,
      FLAGS_chessboard_height
    ),
    chessboard_corners
  );
  
  if (!found) {
    
    // No chessboard -> no homography.
    return false;
  }
  
  cv::drawChessboardCorners(visualisation, 
    cv::Size(FLAGS_chessboard_width,
      FLAGS_chessboard_height
    ),
    cv::Mat(chessboard_corners),
    found
  );

  if (!(FLAGS_chessboard_height % 2 == 1 
        && FLAGS_chessboard_width % 2 == 1)){
    LOG(ERROR) << "Chessboard with odd number of internal"
               << "corners is required; aborting";
    exit(1);
  }
  
  // Coordinates of the points on the lazy susan plane.
  // center of the chessboard is defined to be (0, 0), 
  // size of the cell is defined as a unit length.
  for (int y=(FLAGS_chessboard_height/2); y >= -(FLAGS_chessboard_height/2); y--) {
    for (int x=-(FLAGS_chessboard_width/2); x<=(FLAGS_chessboard_width/2); x++) { 
      
      // Point2f(x, y), though in matrix you specify
      // the column first. Amount of sense it makes: none.
      dst_points.push_back(Point2f(x, y));
    }
  }
  
  // Homography FROM camera plane (src) TO lazy susan plane (dst).
  homography = cv::findHomography(Mat(chessboard_corners), // src
                                  Mat(dst_points) 
  );
  return true;
}
