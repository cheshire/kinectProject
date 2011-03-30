#include "libfreenect.hpp"
#include "util/Mutex.hpp"
#include "AbstractRGBDepthCamera.hpp"

using namespace cv;

class KinectAdapter : public AbstractRGBDepthCamera, public Freenect::FreenectDevice {
public:
  KinectAdapter(freenect_context *ctx, int index);

  bool getRGBDepthFrame(RGBDepthFrame &frame);

private:
  void videoCallback(void *rgb, uint32_t timestamp);
  void depthCallback(void *depth, uint32_t timestamp);

  Mat depth_mat;
  Mat rgb_mat;
  Mat own_mat;

  Mutex rgb_mutex;
  Mutex depth_mutex;

  bool new_depth_frame;
  bool new_rgb_frame;

  std::vector<uint8_t> buffer_depth;
  std::vector<uint8_t> buffer_rgb;
  std::vector<uint16_t> gamma;

};
