#include "kinect_factory.h"

namespace camera {

KinectFactory::KinectFactory() {}

KinectFactory::~KinectFactory() {}

KinectDevice *KinectFactory::create_kinect(int index) {
  return &context.createDevice<KinectDevice>(index);
}

FakeKinect *KinectFactory::create_kinect(const string &rgb_filename,
    const string &depth_filename) {
  return new FakeKinect(rgb_filename, depth_filename);
}

}
