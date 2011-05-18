/*
 * background_filter.h
 *
 *  Created on: 27/04/2011
 *      Author: laurence
 */

#ifndef BACKGROUND_FILTER_H_
#define BACKGROUND_FILTER_H_

#include <cv.h>

#include "camera/image.h"

namespace camera {

class BackgroundFilter {
  cv::Mat1f depth_max;
  cv::Mat1f depth_min;
  bool range_initialized;
public:
  BackgroundFilter();
  virtual ~BackgroundFilter();

  /** Masks all depth and rgb data in an image before the start_depth and after the end_depth.
   *
   * @param image image to be filtered.
   * @param start_depth all pixels before this depth (m) are filtered.
   * @param end_depth all pixels after this depth (m) are filtered.
   *
   */
  void filter(Image &image, double start_depth, double end_depth);

  cv::Mat1b filter_object(Image &image);

  void apply_mask(Image &image, cv::Mat1b mask, bool use_existing_mask);

  /**
   * Adds the image as a calibration image used for range filtering.
   */
  void add_calibration_image(const Image &image);

};

}

#endif /* BACKGROUND_FILTER_H_ */
