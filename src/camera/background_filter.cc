/*
 * background_filter.cc
 *
 *  Created on: 27/04/2011
 *      Author: laurence
 */

#include "background_filter.h"

#include <glog/logging.h>

namespace camera {

BackgroundFilter::BackgroundFilter() : range_initialized(false) {
}

void BackgroundFilter::filter(Image &image, double start_depth, double end_depth) {
  cv::Mat1f depth = image.mapped_depth;
  image.mask.create(image.mapped_depth.size());

  for (int r = 0; r < depth.rows; r++) {
    for (int c = 0; c < depth.cols; c++) {
      int value = 0;
      if ((depth(r,c) < end_depth && depth(r,c) > start_depth) || depth(r,c) == 0) {
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

void BackgroundFilter::apply_mask(Image &image, cv::Mat1b mask, bool use_existing_mask) {
  image.masked_rgb.create(image.mapped_depth.size());
  image.masked_depth.create(image.mapped_depth.size());
  for (int r = 0; r < image.mapped_depth.rows; r++) {
    for (int c = 0; c < image.mapped_depth.cols; c++) {
      if (mask(r, c) == 255) {
        if (use_existing_mask) {
          image.masked_rgb(r, c) = image.masked_rgb.at<cv::Vec3b>(r, c);
          image.masked_depth(r, c) = image.masked_depth.at<float>(r, c);
        } else {
          image.masked_rgb(r, c) = image.rgb.at<cv::Vec3b>(r, c);
          image.masked_depth(r, c) = image.mapped_depth.at<float>(r, c);
        }
      } else {
        image.masked_rgb(r, c) = cv::Vec3b();
        image.masked_depth(r, c) = 0.0f;
      }
    }
  }
}

cv::Mat1b BackgroundFilter::filter_object(Image &image) {
  cv::Mat1f depth;// = image.mapped_depth;
  cv::Mat1b mask(image.mapped_depth.size());
  cv::GaussianBlur(image.mapped_depth, depth, cv::Size(5,5), 2, 2);

  if (!range_initialized) {
    return mask;
  }

  for (int r = 0; r < depth.rows; r++) {
    for (int c = 0; c < depth.cols; c++) {
      int value = 0;
      if (depth(r, c) != 0 && (depth(r,c) < depth_min(r, c) || depth(r,c) > depth_max(r, c))) {
        value = 255;
      }
      if (depth_max(r,c) - depth_min(r, c) > 0.1) {
        value = 255;
      }
      mask(r,c) = value;
    }
  }

  // Try and fill in the main region by detecting the outer contours;
  // this should negate the noise to an extent - although it only works
  // if the center region's depth is continuous.
  cv::Mat temp = mask.clone();
  vector< vector<cv::Point> > contours;
  cv::findContours(temp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  for (int i = 0; i < contours.size(); i++) {
    vector<cv::Point> contour = contours.at(i);
    if (cv::contourArea(cv::Mat(contour)) < 50) {
      drawContours(temp, contours, i, cv::Scalar(0), CV_FILLED);
    }
  }


  return mask;
}

void BackgroundFilter::add_calibration_image(const Image &image) {
  cv::Mat1f blurred_depth;// = image.mapped_depth;
  cv::GaussianBlur(image.mapped_depth, blurred_depth, cv::Size(5,5), 2, 2);

  if (!range_initialized) {
    depth_max = blurred_depth.clone();
    depth_min = blurred_depth.clone();
    range_initialized = true;
    return;
  }

  for (int r = 0; r < blurred_depth.rows; r++) {
    for (int c = 0; c < blurred_depth.cols; c++) {
      float depth = blurred_depth(r, c);
      if (depth == 0) {
        continue;
      }
      if (depth > depth_max(r, c)) {
        depth_max(r, c) = depth;
      }
      if (depth < depth_min(r, c)) {
        depth_min(r, c) = depth;
      }
    }
  }
}

BackgroundFilter::~BackgroundFilter() {
  // TODO Auto-generated destructor stub
}

}
