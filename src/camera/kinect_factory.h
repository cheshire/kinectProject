#ifndef KINECT_FACTORY_H
#define KINECT_FACTORY_H

#include <libfreenect.hpp>

#include "camera/fake_kinect.h"
#include "camera/kinect_device.h"

namespace camera {

class KinectFactory
{
  public:
    KinectDevice *create_kinect(int index = 0);
    FakeKinect  *create_kinect(const string &directory);
  protected:
    Freenect::Freenect context;
  private:
};
}
#endif // KINECT_FACTORY_H
