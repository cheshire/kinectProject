#include "kinect_factory.h"

namespace camera {

KinectFactory::KinectFactory() {}

KinectFactory::~KinectFactory() {}

KinectDevice *KinectFactory::create_kinect(int index) {
  return &context.createDevice<KinectDevice>(index);
}

}
