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
namespace camera {

namespace {

inline bool is_yx_in_range(const cv::Mat& image, int y, int x){
  return (x >= 0) && (y >= 0) && (x < image.cols) && (y < image.rows);
}

} // End namespace

/**
 * Aligns the RGB image with the depth image in place.
 */
void ImageCorrector::align(Image& image) {
  cv::Mat &mapped_colour = image.mapped_rgb;
  mapped_colour = cv::Mat3b(image.depth.size());
  cv::Mat &mapped_depth = image.mapped_depth;
  mapped_depth = cv::Mat1f(image.rgb.size());
  mapped_depth = 0.f;

  // Note that i corresponds to a row and j to a column - this is the reverse
  // of what you would expect (row, col) instead of (col, row).
  for (int i = 0; i < image.depth.rows; i++)
  for (int j = 0; j < image.depth.cols; j++) {
    if (!is_yx_in_range(image.depth, i, j)) {
      continue;
    }

    float depth = image.depth.at<float>(i,j);

    cv::Point3f p = image.depth_perspective->unproject_from_image(j, i, depth);
    cv::Point3f prgb = image.rgb_perspective->project_to_image(p);

    int x = floor(prgb.x + 0.5);
    int y = floor(prgb.y + 0.5);

    if (is_yx_in_range(image.rgb, y, x)) {
      mapped_colour.at<cv::Vec3b>(i, j)
        = image.rgb.at<cv::Vec3b>(y, x);
      mapped_depth.at<float>(y, x) = prgb.z;
    }
  }
}



/**
 * Undistorts the image in place (this affects the field of view).
 *
 * Note: this does not align the RGB and Depth Images.
 */
void ImageCorrector::undistort(Image& frame) {
  cv::Mat depth_m, rgb;

  // 1. Undistort the images using the calibration data.
  cv::remap(frame.rgb,
      rgb,
      frame.rgb_perspective->undistort_map_1,
      frame.rgb_perspective->undistort_map_2,
      CV_INTER_LINEAR);

  cv::remap(frame.depth,
        depth_m,
        frame.depth_perspective->undistort_map_1,
        frame.depth_perspective->undistort_map_2,
        CV_INTER_LINEAR);

  // 2. Copy the undistorted images back to the frame.
  depth_m.copyTo(frame.depth);
  rgb.copyTo(frame.rgb);
}


}
