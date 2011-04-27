#ifndef KINECT_DEVICE_H_
#define KINECT_DEVICE_H_

#include <libfreenect.hpp>

#include "camera/image.h"
#include "camera/image_source.h"
#include "camera/camera_perspective.h"
#include "util/mutex.h"

namespace camera {

class KinectSource : public Freenect::FreenectDevice,
    public ImageSource {
  public:
    KinectSource(freenect_context *ctx, int index);
    ~KinectSource();

    /** Gets the latest rgb/depth frame from the kinect device, if one
     * exists.
     * 
     * Copies the frame into &frame and returns CameraResponse.OK
     * if one is available and CameraResponse.WAIT otherwise.
     */
    CameraResponse get_image(Image *frame);

  protected:
    void VideoCallback(void *_rgb, uint32_t timestamp);
    void DepthCallback(void *_depth, uint32_t timestamp);

  private:
    cv::Mat depth_mat;
    cv::Mat rgb_mat;

    CameraPerspective *rgb_perspective;
    CameraPerspective *depth_perspective;

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
