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

};

}

#endif /* BACKGROUND_FILTER_H_ */
