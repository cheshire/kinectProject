#ifndef KINECT_DEVICE_H_
#define KINECT_DEVICE_H_

#include <libfreenect.hpp>

#include "abstract_rgb_depth_camera.h"
#include "util/mutex.h"

namespace camera {

class KinectDevice : public Freenect::FreenectDevice,
    public AbstractRGBDepthCamera {
  public:
    KinectDevice(freenect_context *ctx, int index);
    ~KinectDevice();

    // Gets the latest rgb/depth frame from the kinect device, if one
    // exists.
    //
    // Copies the frame into &frame and returns true if a new frame is
    // available; otherwise returns false.
    bool get_rgb_depth_frame(RgbDepthFrame *frame);

  protected:
    void convert_depth_matrix_to_meters(Mat *mat);
    void VideoCallback(void *_rgb, uint32_t timestamp);
    void DepthCallback(void *_depth, uint32_t timestamp);

  private:
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

} // NAMESPACE camera

#endif // _KINECT_DEVICE_H_
