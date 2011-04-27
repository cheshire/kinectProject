#ifndef KINECT_FACTORY_H
#define KINECT_FACTORY_H

#include <libfreenect.hpp>

#include "camera/file_source.h"
#include "camera/kinect_source.h"

namespace camera {

class KinectFactory
{
  public:
    KinectSource *create_kinect(int index = 0);
    FileSource  *create_kinect(const string &directory);
  protected:
    Freenect::Freenect context;
  private:
};
}
#endif // KINECT_FACTORY_H
