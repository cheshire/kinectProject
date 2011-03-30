#include "libfreenect.hpp"
#include "util/Mutex.hpp"
#include "AbstractRGBDepthCamera.hpp"

using namespace cv;

namespace camera {

class KinectAdapter : public AbstractRGBDepthCamera {
public:
  KinectAdapter(int index = 0);
  ~KinectAdapter();

  bool getRGBDepthFrame(RGBDepthFrame &frame);

private:
  Freenect::Freenect context;
  Freenect::FreenectDevice *device;
};

}
