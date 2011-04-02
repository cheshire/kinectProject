#include "KinectFactory.h"

namespace camera {

KinectFactory::KinectFactory() {}

KinectFactory::~KinectFactory() {}

KinectFactory::create_kinect(int index) {
  return new KinectDevice(context, index);
}

}
