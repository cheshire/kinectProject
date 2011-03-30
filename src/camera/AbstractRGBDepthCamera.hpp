#include "RGBDepthFrame.hpp"

class AbstractRGBDepthCamera {
public:
  // Generates an RGBDepthFrame if a new frame is available and copies
  // it to &frame; otherwise leaves &frame as is.
  //
  // Returns true if a new frame is available and has been copied;
  // otherwise returns false if a new frame was not generated.
  virtual bool getRGBDepthFrame(RGBDepthFrame &frame);
};
