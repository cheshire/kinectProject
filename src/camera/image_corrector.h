/*
 * image_aligner.h
 *
 * The Image Aligner will takes a depth image and an RGB image and attempts to
 * align them so that each (x,y) coordinate matches.
 *
 *  Created on: 06/04/2011
 *      Author: Laurence Rouesnel
 */

#ifndef IMAGE_ALIGNER_H_
#define IMAGE_ALIGNER_H_

#include <cv.h>
#include <gflags/gflags.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "camera/rgb_depth_frame.h"

namespace camera {

struct CameraCalibrationData {
	double focal_x;
	double focal_y;
	double image_center_x;
	double image_center_y;

	// These will only be set for the RGB camera.
	cv::Mat r;
	cv::Mat t;

	Eigen::Projective3d projective_transform;
};

class ImageCorrector {
public:
  ImageCorrector();

  void undistort(RgbDepthFrame& frame);
  
  void align(RgbDepthFrame& frame);
private:
  void load_calibration_data();

  CameraCalibrationData rgb_camera, depth_camera;

  struct {
    cv::Mat rgb_intrinsics;
    cv::Mat rgb_distortion;
    cv::Mat depth_intrinsics;
    cv::Mat depth_distortion;
  } calibration_data;

  cv::Mat rgb_undistort_map_1;
  cv::Mat rgb_undistort_map_2;

  cv::Mat depth_undistort_map_1;
  cv::Mat depth_undistort_map_2;
};

}

#endif /* IMAGE_ALIGNER_H_ */
