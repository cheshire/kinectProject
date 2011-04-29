#include "kinect_factory.h"

namespace camera {

KinectSource *KinectFactory::create_kinect(int index) {
  return &context.createDevice<KinectSource>(index);
}

FileSource *KinectFactory::create_kinect(const string &directory) {
  return new FileSource(directory);
}

OpenNIKinectSource *KinectFactory::create_openni_kinect() {
  return new OpenNIKinectSource();
}

}
