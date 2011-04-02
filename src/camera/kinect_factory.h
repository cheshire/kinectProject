#ifndef KINECT_FACTORY_H
#define KINECT_FACTORY_H

#include <libfreenect.hpp>

#include "camera/kinect_device.h"

namespace camera {

class KinectFactory
{
  public:
    KinectFactory();
    ~KinectFactory();

    KinectDevice *create_kinect(int index = 0);
  protected:
    Freenect::Freenect context;
  private:
};
}
#endif // KINECT_FACTORY_H
