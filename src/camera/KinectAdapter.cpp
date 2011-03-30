#include "KinectAdapter.hpp"

namespace camera {

namespace {

class KinectDevice : public Freenect::FreenectDevice {
public:
  KinectDevice(freenect_context *ctx, int index)
      : Freenect::FreenectDevice(ctx, index),
      depth_mat(Size(640, 480), CV_16UC1, Scalar(0)),
      rgb_mat(Size(640, 480), CV_8UC3, Scalar(0)),
      new_depth_frame(false), new_rgb_frame(false),
      buffer_depth(FREENECT_DEPTH_11BIT_SIZE),
      buffer_rgb(FREENECT_VIDEO_RGB_SIZE),
      gamma(2048) {
    for (unsigned int i = 0; i < 2048; i++) {
      float v = i / 2048.0;
      v = std::pow(v, 3) * 6;
      gamma[i] = v * 6 * 256;
    }
  }

  void VideoCallback(void *_rgb, uint32_t timestamp) {
    rgb_mutex.lock();
    uint8_t *rgb = static_cast<uint8_t*>(_rgb);
    rgb_mat.data = rgb;
    new_rgb_frame = true;
    rgb_mutex.unlock();
  }

  void DepthCallback(void *_depth, uint32_t timestamp) {
    depth_mutex.lock();
    uint16_t *depth = static_cast<uint16_t*>(_depth);
    depth_mat.data = (uchar*) depth;
    new_depth_frame = true;
    depth_mutex.unlock();
  }

  // From nicolas.burrus.name/index.php/Research/KinectCalibration
  void convert_depth_matrix_to_meters(Mat &mat) {
    Mat temp(Size(640, 480), CV_32FC1, Scalar(1));
    mat.convertTo(temp, CV_32FC1, -0.0030711016, 3.3309495161);
    cv::pow(temp, -1.0, mat);
  }

  bool getRGBDepthFrame(RGBDepthFrame &frame) {
    bool success = false;

    depth_mutex.lock();
    rgb_mutex.lock();

    if (new_rgb_frame && new_depth_frame) {
      cv::cvtColor(rgb_mat, frame.rgbImage, CV_RGB2BGR);
      depth_mat.copyTo(frame.depthImage);
      convert_depth_matrix_to_meters(frame.depthImage);
      frame.timestamp = time(NULL);
      new_depth_frame = new_rgb_frame = false;
      success = true;
    }

    rgb_mutex.unlock();
    depth_mutex.unlock();

    return success;
  }

private:
  void videoCallback(void *rgb, uint32_t timestamp);
  void depthCallback(void *depth, uint32_t timestamp);

  Mat depth_mat;
  Mat rgb_mat;

  Mutex rgb_mutex;
  Mutex depth_mutex;

  bool new_depth_frame;
  bool new_rgb_frame;

  std::vector<uint8_t> buffer_depth;
  std::vector<uint8_t> buffer_rgb;
  std::vector<uint16_t> gamma;

};

} // NAMESPACE

KinectAdapter::KinectAdapter(int index) {
  device = &(context.createDevice<KinectDevice>(index));
  device->startVideo();
  device->startDepth();
}

KinectAdapter::~KinectAdapter() {
  device->stopVideo();
  device->stopDepth();
}

bool KinectAdapter::getRGBDepthFrame(RGBDepthFrame &frame) {
  return static_cast<KinectDevice*>(device)->getRGBDepthFrame(frame);
}

} // NAMESPACE Camera
