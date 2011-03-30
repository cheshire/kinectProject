#include "KinectAdapter.hpp"

KinectAdapter::KinectAdapter(freenect_context *ctx, int index)
    : Freenect::FreenectDevice(ctx, index), buffer_depth(FREENECT_VIDEO_RGB_SIZE),
    buffer_rgb(FREENECT_DEPTH_11BIT_SIZE), gamma(2048), new_rgb_frame(false),
    new_depth_frame(false), depth_mat(Size(640, 480), CV_8UC3, Scalar(0)),
    rgb_mat(Size(640, 480), CV_8UC3, Scalar(0)),
    own_mat(Size(640, 480), CV_8UC3, Scalar(0)) {
  for (unsigned int i = 0; i < 2048; i++) {
    float v = i / 2048.0;
    v = std::pow(v, 3) * 6;
    gamma[i] = v * 6 * 256;
  }
}

void KinectAdapter::videoCallback(void *_rgb, uint32_t timestamp) {
  rgb_mutex.lock();
  uint8_t *rgb = static_cast<uint8_t*>(_rgb);
  rgb_mat.data = rgb;
  new_rgb_frame = true;
  rgb_mutex.unlock();
}

void KinectAdapter::depthCallback(void *_depth, uint32_t timestamp) {
  depth_mutex.lock();
  uint16_t *depth = static_cast<uint16_t*>(_depth);
  depth_mat.data = (uchar*) depth;
  new_depth_frame = true;
  depth_mutex.unlock();
}

bool KinectAdapter::getRGBDepthFrame(RGBDepthFrame &frame) {
  bool success = false;

  depth_mutex.lock();
  rgb_mutex.lock();

  if (new_rgb_frame && new_depth_frame) {
    cv::cvtColor(rgb_mat, &(frame.rgbImage), CV_RGB2BGR);
    depth_mat.copyTo(&(frame.depthImage));
    new_depth_frame = new_rgb_frame = false;
    success = true;
  }

  rgb_mutex.unlock();
  depth_mutex.unlock();

  return success;
}
