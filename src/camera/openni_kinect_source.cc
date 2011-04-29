/*
 * openni_kinect_source.cc
 *
 *  Created on: 28/04/2011
 *      Author: laurence
 */

#include "openni_kinect_source.h"

#include "image_bayer_grbg.h"

#include <XnOpenNI.h>
#include <XnCppWrapper.h>
#include <glog/logging.h>

namespace camera {

namespace {


void copy_and_convert_depth_matrix(cv::Mat& mat, const XnDepthPixel* pixel_depth) {
  cv::MatIterator_<float> it = mat.begin<float>(),
          it_end = mat.end<float>();

  for (int i = 0; it != it_end; ++it, ++i) {
    *it = pixel_depth[i]/1000.0f;
  }
}

void check_error(const XnStatus& status, const char* what) {
  if (status != XN_STATUS_OK) {
    LOG(ERROR) << what;
  }
}

void ensure(bool assert, const char* what) {
  if (!assert) {
    LOG(ERROR) << what;
  }
}

}

OpenNIKinectSource::OpenNIKinectSource()
    : high_resolution_mode_(false) {
  // TODO(laurencer): make command line flag.
  const char* config_file = "config/NestkConfig.xml";

  xn::EnumerationErrors errors;

  // TODO(laurencer): add status code checking.
  XnStatus status = context.InitFromXmlFile(config_file, &errors);
  check_error(status, xnGetStatusString(status));
  status = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth_generator);
  check_error(status, "Find depth generator");
  status = context.FindExistingNode(XN_NODE_TYPE_IMAGE, rgb_generator);
  check_error(status, "Find image generator");


  if (high_resolution_mode_)
  {
    XnMapOutputMode rgb_mode;
    rgb_mode.nFPS = 15;
    rgb_mode.nXRes = 1280;
    rgb_mode.nYRes = 1024;
    rgb_generator.SetMapOutputMode(rgb_mode);
  }

  // We perform custom bayer decoding.
  status = rgb_generator.SetIntProperty ("InputFormat", 6);
  check_error(status, "Change input format");
  status = rgb_generator.SetPixelFormat(XN_PIXEL_FORMAT_GRAYSCALE_8_BIT);
  check_error(status, "Change pixel format");

   // TODO(laurencer): change to assert.
  ensure(depth_generator.IsCapabilitySupported(XN_CAPABILITY_ALTERNATIVE_VIEW_POINT), "Cannot register images.");
  depth_generator.GetAlternativeViewPointCap().SetViewPoint(rgb_generator);

  status = context.StartGeneratingAll();
  check_error(status, "StartGenerating");

  context.WaitAndUpdateAll();
  estimate_calibration();
}

// From void NiteRGBDGrabber :: estimateCalibration()
void OpenNIKinectSource::estimate_calibration()
{
  XnPoint3D p;
  p.X = 0; p.Y = 0; p.Z = -1;
  depth_generator.ConvertProjectiveToRealWorld(1, &p, &p);

  p.X = 0; p.Y = 0; p.Z = -1;
  depth_generator.ConvertRealWorldToProjective(1, &p, &p);
  double cx = p.X;
  double cy = p.Y;

  p.X = 1; p.Y = 1; p.Z = -1;
  depth_generator.ConvertRealWorldToProjective(1, &p, &p);

  double fx = -(p.X-cx);
  double fy = p.Y-cy;

  const double f_correction_factor = 528.0/575.8;
  fx *= f_correction_factor;
  fy *= f_correction_factor;

  const double cy_correction_factor = 267.0/240.0;
  cy *= cy_correction_factor;

  xn::DepthMetaData depth_md;
  depth_generator.GetMetaData(depth_md);

  xn::ImageMetaData rgb_md;
  rgb_generator.GetMetaData(rgb_md);

  int depth_width = depth_md.XRes();
  int depth_height = depth_md.YRes();

  int rgb_width = rgb_md.XRes();
  int rgb_height = rgb_md.YRes();

  float width_ratio = float(rgb_width)/depth_width;
  float height_ratio = float(rgb_height)/depth_height;

  float rgb_fx = fx * width_ratio;
  // Pixels are square on a Kinect.
  // Image height gets cropped when going from 1280x1024 in 640x480.
  // The ratio remains 2.
  float rgb_fy = rgb_fx;
  float rgb_cx = cx * width_ratio;
  float rgb_cy = cy * width_ratio;


  // Setup the intrinsics for both cameras.
  cv::Mat1d intrinsics(3,3);
  cv::setIdentity(intrinsics);
  intrinsics(0,0) = rgb_fx;
  intrinsics(1,1) = rgb_fy;
  intrinsics(0,2) = rgb_cx;
  intrinsics(1,2) = rgb_cy;

  cv::Mat1d distortion(1,5);
  distortion = 0.;


  cv::Mat1d rotation_matrix(3,3);
  setIdentity(rotation_matrix);

  cv::Mat1d translation_matrix(3,1);
  translation_matrix = 0.;

  rgb_perspective = new CameraPerspective(intrinsics, distortion);
  depth_perspective = new CameraPerspective(intrinsics, distortion);

  // TODO(laurencer): add optimiziation - we don't need undistort or align
  // when using openni.
}

CameraResponse OpenNIKinectSource::get_image(Image *image) {
  ImageBayerGRBG bayer_decoder(ImageBayerGRBG::EdgeAware);
  xn::DepthMetaData depth_md;
  xn::ImageMetaData rgb_md;

  // This captures the latest frames synchronously from the kinect.
  context.WaitAndUpdateAll();

  depth_generator.GetMetaData(depth_md);
  rgb_generator.GetMetaData(rgb_md);

  image->depth = cv::Mat(cv::Size(depth_md.XRes(), depth_md.YRes()), CV_32FC1);
  const XnDepthPixel* pixel_depth = depth_md.Data();
  copy_and_convert_depth_matrix(image->depth, pixel_depth);

  cv::Mat temp_rgb_image(cv::Size(rgb_md.XRes(), rgb_md.YRes()), CV_8UC3);

  uchar* raw_rgb_ptr = temp_rgb_image.ptr<uchar>();
  bayer_decoder.fillRGB(rgb_md, temp_rgb_image.cols,temp_rgb_image.rows,
                        raw_rgb_ptr);
  cvtColor(temp_rgb_image, temp_rgb_image, CV_RGB2BGR);
  temp_rgb_image.copyTo(image->rgb);

  image->mapped_depth = image->depth.clone();
  image->mapped_rgb = image->rgb.clone();

  image->aligned = true;
  image->undistorted = true;

  return OK;
}


OpenNIKinectSource::~OpenNIKinectSource() {
  delete rgb_perspective;
  delete depth_perspective;
}
}
