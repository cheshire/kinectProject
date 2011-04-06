#include "kinect_factory.h"

namespace camera {

KinectDevice *KinectFactory::create_kinect(int index) {
  return &context.createDevice<KinectDevice>(index);
}

FakeKinect *KinectFactory::create_kinect(const string &directory) {
  return new FakeKinect(directory);
}

}
