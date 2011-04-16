/*
 * image_corrector.cc
 *
 * Calibration algorithm from
 * https://github.com/nburrus/nestk/blob/master/ntk/camera/calibration.cpp
 * and https://github.com/nburrus/nestk/blob/master/ntk/camera/rgbd_processor.cpp
 * was used.
 *
 *  Created on: 06/04/2011
 *      Author: Laurence Rouesnel
 */

#include "image_corrector.h"

#include <glog/logging.h>
#include <gflags/gflags.h>
#include <cmath>
#include <cv.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>

using namespace std;

DEFINE_string(distortion_data, "distortion.xml", "The path of the distortion correction data");

namespace camera {

namespace {

/* From https://github.com/nburrus/nestk/blob/master/ntk/geometry/pose_3d.cpp */
/**
 * @param x
 * @param y
 * @param depth 
 * 
 * @return 3-vector representing the point coordinates out of x-, y- pixel coordinates
 * and depth.
 */
cv::Vec3f unproject_from_image(int x, int y, float depth) {
  return cv::Vec3f(x * depth, y * depth, depth);
}

/* From https://github.com/nburrus/nestk/blob/master/ntk/geometry/pose_3d.cpp */
cv::Point3f project_to_image(const cv::Point3f& p, Eigen::Projective3d projective_transform) {
  
  // Translate the cvPoint to an EigenVector.
  Eigen::Vector4d ep(p.x, p.y, p.z, 1);
  
  // Project transform will be set for the rgb camera.
  ep = projective_transform * ep;
  
  ep(0) /= ep(2);
  ep(1) /= ep(2);

  return cv::Point3f(ep(0), ep(1), ep(2));
}

// From https://github.com/nburrus/nestk/blob/master/ntk/geometry/pose_3d.cpp
Eigen::Isometry3d intrinsics_transform(CameraCalibrationData& data) {
  Eigen::Isometry3d m;
  m.setIdentity();
  m(0,0) = data.focal_x;
  m(0,2) = data.image_center_x;
  m(1,1) = data.focal_y;
  m(1,2) = data.image_center_y;
  return m;
}

void convert_depth_matrix_to_meters(cv::Mat& mat) {
  cv::Mat temp;
  mat.convertTo(temp, CV_32FC1, -0.0030711016, 3.3309495161);
  cv::pow(cv::abs(temp), -1.0, mat);
}

inline bool is_yx_in_range(const cv::Mat& image, int y, int x){
  return (x >= 0) && (y >= 0) && (x < image.cols) && (y < image.rows);
}

} // End namespace

ImageCorrector::ImageCorrector() {
  load_calibration_data();
}

// https://github.com/nburrus/nestk/blob/master/ntk/camera/calibration.cpp
void ImageCorrector::load_calibration_data() {
  FileStorage fs(FLAGS_distortion_data, FileStorage::READ);

  // Load the matricies from the calibration file.
  fs["rgb_intrinsics"] >> calibration_data.rgb_intrinsics;
  fs["rgb_distortion"] >> calibration_data.rgb_distortion;
  fs["depth_intrinsics"] >> calibration_data.depth_intrinsics;
  fs["depth_distortion"] >> calibration_data.depth_distortion;

  // Create the undistortion map.
  cv::initUndistortRectifyMap(
      calibration_data.rgb_intrinsics,
      calibration_data.rgb_distortion,
      cv::Mat(),
      calibration_data.rgb_intrinsics,
      cv::Size(640, 480),
      CV_16SC2,
      rgb_undistort_map_1,
      rgb_undistort_map_2
      );

  cv::initUndistortRectifyMap(
      calibration_data.depth_intrinsics,
      calibration_data.depth_distortion,
      cv::Mat(),
      calibration_data.depth_intrinsics,
      cv::Size(640, 480),
      CV_16SC2,
      depth_undistort_map_1,
      depth_undistort_map_2
      );

  // Load the transformations for aligning the images.
  // See https://github.com/nburrus/nestk/blob/master/ntk/geometry/pose_3d.cpp
  cv::Mat r,t;
  fs["R"] >> r;
  fs["T"] >> t;

  cv::Mat1d to_gl_base(3,3);
  cv::setIdentity(to_gl_base);

  to_gl_base(1, 1) = -1;
  to_gl_base(2, 2) = -1;

  rgb_camera.r = to_gl_base.inv() * r.inv() * to_gl_base;
  rgb_camera.t = to_gl_base * (-t);

  Eigen::Isometry3d camera_transform;
  camera_transform.setIdentity();

  // Convert rotation_matrix to an Eigen::Matrix
  Eigen::Matrix3d rotation_matrix;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      rotation_matrix(r, c) = rgb_camera.r.at<double>(r, c);
    }
  }

  // Apply the translation and rotation matricies to the
  // camera transform matrix.
  camera_transform.prerotate(rotation_matrix);
  camera_transform.pretranslate(Eigen::Vector3d(
      rgb_camera.t.at<double>(0),
      rgb_camera.t.at<double>(1),
      rgb_camera.t.at<double>(2)));


  // Compute the 'projective transform'
  Eigen::Isometry3d to_opencv = Eigen::Isometry3d::Identity();
  to_opencv(1,1) = to_opencv(2,2) = -1;
  Eigen::Projective3d projection = Eigen::Projective3d::Identity();
  Eigen::Isometry3d intrinsics = intrinsics_transform(rgb_camera);

  rgb_camera.projective_transform
    = intrinsics * projection * to_opencv * camera_transform;
}

/**
 * Aligns the RGB image with the depth image in place.
 */
void ImageCorrector::align(RgbDepthFrame& frame) {
  cv::Mat mapped_colour = cv::Mat::zeros(Size(640, 480), CV_8UC3);
  cv::Mat mapped_depth = cv::Mat::zeros(Size(640, 480), CV_16UC1);
  
  for (int i = 0; i < frame.depth_image.rows; i++) {
    for (int j = 0; j < frame.depth_image.cols; j++) {
      
      // SO... <double> is definitely wrong - it gives results around
      // 0, or 1e-233
      // int seems to give weird results as well - 
      // 53347118
      float depth = frame.depth_image.at<float>(i,j);
      LOG(INFO) << cv::format("Depth is %e", depth);
      
      // TODO(george) -
      /*a) why frame.depth image is all integral?
       * 
       */
//       cout << frame.depth_image << endl;
      
      cv::Point3f p = unproject_from_image(i, j, depth);
      cv::Point3f prgb = project_to_image(p, rgb_camera.projective_transform);

      int x = floor(prgb.x + 0.5);
      int y = floor(prgb.y + 0.5);
      
//       LOG(INFO) << cv::format("depth is %d", depth);
//       LOG(INFO) << cv::format("y is %d, x is %d", y, x);
      
      if (y < frame.rgb_image.rows && x < frame.rgb_image.cols && y >= 0 && x >= 0) {
        mapped_colour.at<cv::Vec3b>(i, j)
          = frame.rgb_image.at<cv::Vec3b>(y, x);
          
        mapped_depth.at<double>(y, x) = prgb.z;
      }
    }
  }
  
  mapped_colour.copyTo(frame.rgb_image);
  mapped_depth.copyTo(frame.depth_image);
}



/**
 * Undistorts the image in place (this affects the field of view).
 *
 * Note: this does not align the RGB and Depth Images.
 */
void ImageCorrector::undistort(RgbDepthFrame& frame) {
  cv::Mat depth_m, rgb;

  // 1. Convert depth matrix to meters.
  // This does an in-place conversion.
  convert_depth_matrix_to_meters(frame.depth_image);
  
  // 2. Undistort the images using the calibration data.
  cv::remap(frame.rgb_image,
      rgb,
      rgb_undistort_map_1,
      rgb_undistort_map_2,
      CV_INTER_LINEAR);

  cv::remap(frame.depth_image,
        depth_m,
        depth_undistort_map_1,
        depth_undistort_map_2,
        CV_INTER_LINEAR);

  // 3. Copy the undistorted images back to the frame.
  depth_m.copyTo(frame.depth_image);
  rgb.copyTo(frame.rgb_image);
}

}
