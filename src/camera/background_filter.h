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

  cv::Mat previous_depth;
  cv::Mat previous_rgb;
public:
  int threshold;
  cv::Size blur;
  BackgroundFilter(const Image &image);
  virtual ~BackgroundFilter();

  void filter(const Image &image, cv::Mat &static_background,
      cv::Mat &dynamic_background, cv::Mat &object);

};

}

#endif /* BACKGROUND_FILTER_H_ */
