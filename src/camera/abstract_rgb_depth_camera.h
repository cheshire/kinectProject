#ifndef ABSTRACT_RGB_DEPTH_CAMERA_H_
#define ABSTRACT_RGB_DEPTH_CAMERA_H_

#include "rgb_depth_frame.h"

class AbstractRGBDepthCamera {
public:
  // Generates an RGBDepthFrame if a new frame is available and copies
  // it to &frame; otherwise leaves &frame as is.
  //
  // Returns true if a new frame is available and has been copied;
  // otherwise returns false if a new frame was not generated.
  virtual bool get_rgb_depth_frame(RGBDepthFrame &frame) = 0;
};

#endif
