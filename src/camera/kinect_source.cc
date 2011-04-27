#include "kinect_source.h"

#include <gflags/gflags.h>

DEFINE_string(distortion_data, "distortion.xml", "The path of the distortion correction data");

namespace camera {

namespace {

float raw_depth_to_meters_alt(int raw_depth)
{
  if (raw_depth < 2047)
  {
   return 1.0 / (raw_depth * -0.0030711016 + 3.3309495161);
  }
  return 0;
}

inline float raw_depth_to_meters(int raw_depth)
{
  double depth_baseline = 7.5e-02;
  double depth_offset = 1090;
  double focal_ir = 580;

  float depth = 0;
  if (raw_depth < 2047)
  {
    depth = focal_ir * 8.0 * depth_baseline / (depth_offset - raw_depth);
  } else {
    depth = 0;
  }
  if (depth < 0)
    depth = 0;
  else if (depth > 10)
    depth = 10;
  return depth;
}

void convert_depth_matrix_to_meters(cv::Mat& mat) {
  cv::MatIterator_<float> it = mat.begin<float>(),
          it_end = mat.end<float>();

  for (; it != it_end; ++it) {
    *it = raw_depth_to_meters(*it);
  }
}
}

KinectSource::KinectSource(freenect_context *ctx, int index)
      : Freenect::FreenectDevice(ctx, index),
      depth_mat(cv::Size(640, 480), CV_16UC1, cv::Scalar(0)),
      rgb_mat(cv::Size(640, 480), CV_8UC3, cv::Scalar(0)),
      new_depth_frame(false), new_rgb_frame(false),
      buffer_depth(FREENECT_DEPTH_11BIT_SIZE),
      buffer_rgb(FREENECT_VIDEO_RGB_SIZE),
      gamma(2048) {
    for (unsigned int i = 0; i < 2048; i++) {
      float v = i / 2048.0;
      v = std::pow(v, 3) * 6;
      gamma[i] = v * 6 * 256;
    }

    // Load calibration data.
    cv::FileStorage fs(FLAGS_distortion_data, cv::FileStorage::READ);

    cv::Mat rgb_intrinsics, rgb_distortion, depth_intrinsics, depth_distortion;
    fs["rgb_intrinsics"] >> rgb_intrinsics;
    fs["rgb_distortion"] >> rgb_distortion;
    fs["depth_intrinsics"] >> depth_intrinsics;
    fs["depth_distortion"] >> depth_distortion;

    rgb_perspective = new CameraPerspective(rgb_intrinsics, rgb_distortion);
    depth_perspective = new CameraPerspective(depth_intrinsics, depth_distortion);

    this->startVideo();
    this->startDepth();
}

KinectSource::~KinectSource() {
  this->stopVideo();
  this->stopDepth();
  delete rgb_perspective;
  delete depth_perspective;
}

CameraResponse KinectSource::get_image(Image *frame) {
  bool success = false;

  depth_mutex.lock();
  rgb_mutex.lock();

  if (new_rgb_frame && new_depth_frame) {
    cv::cvtColor(rgb_mat, frame->rgb, CV_RGB2BGR);
    depth_mat.convertTo(frame->depth, CV_32FC1);
    depth_mat.copyTo(frame->raw_depth);
    convert_depth_matrix_to_meters(frame->depth);
    frame->timestamp = time(NULL);
    new_depth_frame = new_rgb_frame = false;
    frame->rgb_perspective = rgb_perspective;
    frame->depth_perspective = depth_perspective;
    success = true;
  }

  rgb_mutex.unlock();
  depth_mutex.unlock();

  if (success){
    return OK;
  } else {
    return WAIT;
  }
}

void KinectSource::VideoCallback(void *_rgb, uint32_t timestamp) {
  rgb_mutex.lock();
  uint8_t *rgb = static_cast<uint8_t*>(_rgb);
  rgb_mat.data = rgb;
  new_rgb_frame = true;
  rgb_mutex.unlock();
}

void KinectSource::DepthCallback(void *_depth, uint32_t timestamp) {
  depth_mutex.lock();
  uint16_t *depth = static_cast<uint16_t*>(_depth);
  depth_mat = cv::Mat(cv::Size(640, 480), CV_16UC1, depth);
  new_depth_frame = true;
  depth_mutex.unlock();
}

} // NAMESPACE camera
