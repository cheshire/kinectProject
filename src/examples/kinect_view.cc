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

using namespace camera;
using namespace std;

// Forward declarations, see below for comments.
int find_homography(const cv::Mat img,
                    cv::Mat visualisation,
                    cv::Mat& homography);

Image get_frame(ImageSource *device, ImageCorrector *corrector);

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
  
  KinectFactory factory;
  ImageCorrector *image_corrector;
  ImageSource *device;
  FileWriter writer;
  orientationdetector::OrientationDetector orientation_detector;
  
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
      writer.record(*img);
      delete img;
    }
    
    // Any visualisations on top of rgb image.
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
      bool status = orientation_detector.find_orientation_angle(homography,
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
