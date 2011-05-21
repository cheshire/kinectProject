#define _USE_MATH_DEFINES

#include <cv.h>
#include <cmath>
#include <math.h>
#include <highgui.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <vector>

#include "orientation_detector/orientation_detector.h"

using namespace cv;
using namespace orientationdetector;

namespace {
  cv::Scalar RANDOM_COLOR(255, 255, 0);
    
  /**
   * Comparison operator so that contours can be sorted
   * according to the error.
   */
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
}

OrientationDetector::OrientationDetector(){
  prev_angle = -1;
  num_jumps = 0;
}

void OrientationDetector::initialize(){
  int *value = new int(0);  
  
  prev_angle = -1;
  num_jumps = 0;
  
  namedWindow("angle", CV_WINDOW_AUTOSIZE);
  createTrackbar("t_angle", "angle", value, (int) (2*M_PI*1000));
}

Point OrientationDetector::get_chessboard_centre_in_image(const Mat homography) {
  Mat inv_h = homography.inv();
  int cx = (int)(inv_h.at<double>(0,2) / inv_h.at<double>(2,2));
  int cy = (int)(inv_h.at<double>(1,2) / inv_h.at<double>(2,2));
  return Point(cx, cy);
}

bool OrientationDetector::find_orientation_angle(
  const Mat homography,
  const Mat rgb_image,
  Mat visualisation,
  float& orientation_angle
){  
  // Run the contour detection algorithm and detect
  // two black circles on lazy susan.           
  vector<EllipticalContour> contours = find_circles(rgb_image,
    visualisation
  );
  
  // (Up to) two angles corresponding (up to) two circles.
  float orientation_angle_[2];

//   LOG(INFO) << "Found " << contours.size() << " ellipses.";
    
  // If we have more then 2 ellipses - something is wrong.
  // If we don't see any - something is wrong as well.
  if (contours.size() == 0 || contours.size() > 2) {
    LOG(ERROR) << "Incorrect number of ellipses detected - "
                << contours.size() << " => orientation detection"
                << " is impossible.";
    return false;
  }
  for (unsigned i=0; i<contours.size(); i++) {    
    vector<Point3f> h_center; // One-element output vector.
    
    convertPointsHomogeneous(
      // Circle center in camera plane coordinates.
      Mat(one_element_vector<Point2f>(contours.at(i).circle_center)),
      
      // Output
      h_center);
    
    // Circle center in camera plane homogeneous coordinates.
    Point3d d_h_center = h_center.at(0);
        
    // Center in turntable homogeneous coordinates.
    Point3f center_h = Mat(homography * Mat(d_h_center)).at<Point3d>(0, 0);
    

    // One element output vector
    vector<Point2f> center_temp;

    // Converting back from homogeneous coordinates.
    convertPointsHomogeneous(
      Mat(one_element_vector<Point3f>(center_h)),
      center_temp);
    
    // Circle center in turntable coordinates.
    Point2f center = center_temp.at(0);


    if (center.x == 0) {
      orientation_angle_[i] = M_PI/2;
    } else {
      orientation_angle_[i] = atan(center.y/center.x);
    }
  }
  
  /* 
   * Note - from the property of atan the found angle lies in the
   * range -pi/2 to +pi/2. If everything went right, two found angles
   * should be (almost) equal.
   */
  if (contours.size() == 2) {
    // If everything went right, two found angles should be (almost) exactly
    
/*    LOG(INFO) << "Two circles were seen, angles are "
              << orientation_angle_[0] << " and " << orientation_angle_[1];*/
    orientation_angle = (orientation_angle_[0] + orientation_angle_[1]) / 2;
  } else {
    orientation_angle = orientation_angle_[0];
  }
  
  float corrected_angle;
  bool status = correct_rotation_angle(orientation_angle, corrected_angle);
  
  if (!status) {
    return false;
  }
  
  setTrackbarPos("t_angle", "angle", (int) ((corrected_angle) * 1000));      
  return true;
}

bool OrientationDetector::correct_rotation_angle(
  float orientation_angle,
  float &corrected_angle
){  
  if (prev_angle == -1) {
    // Prev_angle was not initialized.
    prev_angle = orientation_angle + M_PI / 2;
  }
    
  // Move the angle from range [-PI/2, PI/2] to [0, PI].
  corrected_angle = orientation_angle + M_PI / 2;
  
  // Account for the abrupt jumps in atan function - extends
  // the domain to [0, 2PI].
  corrected_angle = fmod((corrected_angle + M_PI * num_jumps),
                         2 * M_PI);  
  
  // Let's find whether "jump" occurs here.
  float delta = fabs(corrected_angle - prev_angle);  
  if (delta > M_PI / 4 && delta < M_PI / 2) {
    
    // This data point is retarded, let's just skip it.
    // Usually there are 2 or 3 such points per the whole
    // rotation.
    return false;
  } else {
    if (delta > M_PI / 2) {
      
      // Jump occurs between this angle and the previous one.
      corrected_angle = fmod(corrected_angle + M_PI, M_PI * 2);      
      num_jumps++;
    }

    prev_angle = corrected_angle;
  }
  
  return true;
}

bool OrientationDetector::create_elliptical_contour(EllipticalContour& result,
                                                    int min_size,
                                                    double min_circularity) {
  result.contour_area = cv::contourArea(cv::Mat(result.contour));

  // If there are less than 8 contours an ellipse cannot be fitted.
  if (result.contour_area < min_size || result.contour.size() < 6) {
    return false;
  }

  float circumference = cv::arcLength(cv::Mat(result.contour), true);
  result.error = (circumference == 0) ? -1 : (float) (4 * M_PI * result.contour_area / pow(circumference, 2));

  cv::RotatedRect ellipse = cv::fitEllipse(cv::Mat(result.contour));
  result.circle_center = ellipse.center;

  return result.error > min_circularity;
}

vector<EllipticalContour> OrientationDetector::find_circles(const cv::Mat rgb_image,
  Mat visualisation,
  int blur_amount,
  int canny_threshold1,
  int canny_threshold2, 
  int canny_aperture_size){
  
  vector<EllipticalContour> results;
  cv::Mat gray, edges, temp;
  cv::cvtColor(rgb_image, temp, CV_BGR2GRAY);
  cv::threshold(temp, gray, 60, 255, THRESH_BINARY);
  cv::erode(gray, gray, Mat());
  // Find edges and contours.
  vector<vector<cv::Point> > contours;
  cv::findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

  for (size_t i = 0, len = contours.size(); i < len; i++) {
    EllipticalContour result;
    result.contour = contours[i];
    bool contour_created = create_elliptical_contour(result);

    if (contour_created) {
      cv::drawContours(visualisation, contours, i, RANDOM_COLOR, CV_FILLED);
      
      results.push_back(result);
    }        
  }

  sort(results.begin(), results.end(), EllipticalContourComparison());
  return results;  
}

template <typename T>
/**
 * Create and return a vector containing obj.
 */
vector<T> OrientationDetector::one_element_vector(T obj){
  vector<T> temp;
  temp.push_back(obj);
  return temp;
}
