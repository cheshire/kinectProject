/*
 * background_filter.cc
 *
 *  Created on: 27/04/2011
 *      Author: laurence
 */

#include "background_filter.h"

#include <glog/logging.h>

namespace camera {

BackgroundFilter::BackgroundFilter(const Image &image) : blur(cv::Size(75,75)),
    threshold(100) {
  cv::GaussianBlur(image.mapped_rgb, previous_rgb, blur, 0, 0);
  cv::GaussianBlur(image.depth, previous_depth, blur, 0, 0);

}

void BackgroundFilter::filter(const Image &image, cv::Mat &static_background,
      cv::Mat &dynamic_background, cv::Mat &object) {
  cv::Mat temp_rgb = image.mapped_rgb.clone(),
      temp_depth = image.depth.clone(),
      temp_rgb_thres,
      temp_depth_thres;


  cv::GaussianBlur(image.mapped_rgb, temp_rgb, cv::Size(5,5), 0, 0);
  cv::GaussianBlur(image.depth, temp_depth, cv::Size(5,5), 0, 0);

  if (previous_rgb.type() != temp_rgb.type()) {
    LOG(INFO) << "skipping";
    temp_rgb.copyTo(previous_rgb);
    temp_depth.copyTo(previous_depth);
    return;
  }

  cv::absdiff(previous_rgb, temp_rgb, temp_rgb);
  cv::absdiff(previous_depth, temp_depth, temp_depth);

  cv::threshold(temp_depth, static_background, threshold, 1, cv::THRESH_BINARY);

  cv::GaussianBlur(image.depth, dynamic_background, cv::Size(5,5), 0, 0);
}

BackgroundFilter::~BackgroundFilter() {
  // TODO Auto-generated destructor stub
}

}
