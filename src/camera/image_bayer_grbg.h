/*
 * image_bayer_grbg.h
 *
 *  Created on: 28/04/2011
 *      Author: laurence
 */

#ifndef IMAGE_BAYER_GRBG_H_
#define IMAGE_BAYER_GRBG_H_

#include <XnCppWrapper.h>

namespace camera
{
class ImageBayerGRBG
{
public:
  typedef enum
  {
    Bilinear = 0,
    EdgeAware,
    EdgeAwareWeighted
  } DebayeringMethod;

  ImageBayerGRBG (DebayeringMethod method)
    : debayering_method_(method) {}

  void fillRGB(xn::ImageMetaData& xn_image,
                  unsigned width, unsigned height,
                  unsigned char* rgb_buffer,
                  unsigned rgb_line_step = 0);

protected:
  DebayeringMethod debayering_method_;
};

}

#endif /* IMAGE_BAYER_GRBG_H_ */
