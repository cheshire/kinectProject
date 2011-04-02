#ifndef KINECT_FACTORY_H
#define KINECT_FACTORY_H

#include <libfreenect.hpp>

using namespace freenect;

namespace camera {

class KinectFactory
{
  public:
    KinectFactory();
    ~KinectFactory();

    KinectDevice *create_kinect(int index = 0);
  protected:
    Freenect context;
  private:
};
}
#endif // KINECT_FACTORY_H
