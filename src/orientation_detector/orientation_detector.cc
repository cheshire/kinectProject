#include <cv.h>
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
      return first.error < second.error;
    }
  };
}

void OrientationDetector::initialize(){
 
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

  LOG(INFO) << "Found " << contours.size() << " ellipses.";
    
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
    
    LOG(INFO) << "Two circles were seen, angles are "
              << orientation_angle_[0] << " and " << orientation_angle_[1];
    orientation_angle = (orientation_angle_[0] + orientation_angle_[1]) / 2;
  } else {
    orientation_angle = orientation_angle_[0];
  }
  return true;
}

bool OrientationDetector::create_elliptical_contour(EllipticalContour& result,
                                                    int min_size) {
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
  result.error = abs(ellipse.size.width * ellipse.size.height * M_PI
      - result.contour_area) / result.contour_area;

  return true;
}

vector<EllipticalContour> OrientationDetector::find_circles(const cv::Mat rgb_image,
  Mat visualisation,
  int blur_amount,
  int canny_threshold1,
  int canny_threshold2, 
  int canny_aperture_size){
  
  vector<EllipticalContour> results;
  cv::Mat gray, edges;
  cv::cvtColor(rgb_image, gray, CV_BGR2GRAY);
  cv::GaussianBlur(gray, gray, Size(blur_amount, blur_amount), 2, 2);

  // Find edges and contours.
  vector<vector<cv::Point> > contours;
  cv::Canny(gray, edges, canny_threshold1, canny_threshold2,
            canny_aperture_size, false);
  cv::findContours(edges, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);  

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