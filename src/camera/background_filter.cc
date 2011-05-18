/*
 * background_filter.cc
 *
 *  Created on: 27/04/2011
 *      Author: laurence
 */

#include "background_filter.h"

#include <glog/logging.h>

namespace camera {

BackgroundFilter::BackgroundFilter() {
}

void BackgroundFilter::filter(Image &image, double start_depth, double end_depth) {
  cv::Mat1f depth = image.mapped_depth;
  image.mask.create(image.mapped_depth.size());

  for (int r = 0; r < depth.rows; r++) {
    for (int c = 0; c < depth.cols; c++) {
      int value = 0;
      if (depth(r,c) < end_depth && depth(r,c) > start_depth) {
        value = 255;
      }
      image.mask(r,c) = value;
    }
  }

  // Try and fill in the main region by detecting the outer contours;
  // this should negate the noise to an extent - although it only works
  // if the center region's depth is continuous.
  cv::Mat temp = image.mask.clone();
  vector< vector<cv::Point> > contours;
  cv::findContours(temp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  drawContours(image.mask, contours, -1, cv::Scalar(255), CV_FILLED);

  image.masked_rgb.create(image.mapped_depth.size());
  image.masked_depth.create(image.mapped_depth.size());

  for (int r = 0; r < depth.rows; r++) {
    for (int c = 0; c < depth.cols; c++) {
      if (image.mask(r, c) == 255) {
        image.masked_rgb(r, c) = image.rgb.at<cv::Vec3b>(r, c);
        image.masked_depth(r, c) = image.mapped_depth.at<float>(r, c);
      } else {
        image.masked_rgb(r, c) = cv::Vec3b();
        image.masked_depth(r, c) = 0.0f;
      }
    }
  }
}

BackgroundFilter::~BackgroundFilter() {
  // TODO Auto-generated destructor stub
}

}
