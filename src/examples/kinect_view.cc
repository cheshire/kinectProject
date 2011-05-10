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
DEFINE_int32(chessboard_width, 3, "Number of internal chessboard corners in width");
DEFINE_int32(chessboard_height, 3, "Number of internal chessboard corners in height");
DEFINE_bool(show_gui, true, "Show the images currently processed");

using namespace camera;
using namespace std;

cv::Scalar RANDOM_COLOR(255, 255, 0);

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

template <typename T>
/**
 * Create and return a vector containing obj.
 */
vector<T> one_element_vector(T obj){
  vector<T> temp;
  temp.push_back(obj);
  return temp;
}


/**
 * Tries to fit an ellipse onto a contour, if an ellipse
 * was found sets it's center and the possible error to struct.
 * 
 * @return Whether the elliptical contour was found.
 */
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
  result.error = abs(ellipse.size.width * ellipse.size.height * M_PI
      - result.contour_area) / result.contour_area;

  return true;
}


/**
 * Find ellipses in the given image.
 * 
 * @param rgb_image Image to find the contours in.
 * @param visualisation Canvas to draw any visualisations upon.
 * 
 * @return vector of found contours.
 */
vector<EllipticalContour> find_circles(const cv::Mat rgb_image,
                  cv::Mat visualisation,
                  int blur_amount = 7,
                  int canny_threshold1 = 125,
                  int canny_threshold2 = 200,
                  int canny_aperture_size = 3) {
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

/**
 * If we can find chessboard corners in the image,
 * we will write the homography. Otherwise, return 0.
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
 * Get the orientation angle of the turntable.
 * 
 * @param homography Homography matrix for the translation
 * from/to LS plane.
 * @param rgb_image Image of the turntable.
 * @param visualisation Anything we might want to visualise.
 * @param orientation_angle Output param to write orientation angle
 * into. It is measured in the range of -\pi to +\pi. We can detect
 * full rotation from an abrupt jump from ~ \pi to ~ -\pi.
 * 
 * @return Whether the angle was found.
 */
bool find_orientation_angle(
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


int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  bool running(true);
  bool homography_found(false);
  Mat homography;
  Image frame;
  Mat visualisation;
  KinectFactory factory;
  ImageCorrector *image_corrector;
  ImageSource *device;
  FileWriter writer;
  bool recording = false;
  float orientation_angle;
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
    namedWindow("depth", CV_WINDOW_AUTOSIZE);
    namedWindow("visualisation", CV_WINDOW_AUTOSIZE);
  }

  cout << "Chessboard calibration is required to figure out the "
        << "lazy susan orientation. Camera should be facing at empty" 
        << " turntable with the chessboard on it."<< endl;    
  while(running) {
    
    frame = get_frame(device, image_corrector);
    if (recording) {
      Image *img = new Image();
      img->rgb = frame.rgb.clone();
      img->depth = frame.depth.clone();
      recorded_images.push_back(img);
    }
    
    // any visualisations on top of rgb image.
    frame.mapped_rgb.copyTo(visualisation); 
    
    if (!homography_found) {
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
      bool status = find_orientation_angle(homography,
        frame.mapped_rgb,
        visualisation,
        orientation_angle
      );
      if (!status) {
        // Can't procede without orientation angle.
        LOG(INFO) << "Skipping incorrect frame";
      } else {
        LOG(INFO) << "ANGLE: " << orientation_angle;
      }
    }

    if (FLAGS_show_gui) {
      cv::imshow("rgb", frame.mapped_rgb);
      cv::imshow("depth", frame.mapped_depth);
      cv::imshow("visualisation", visualisation);
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
