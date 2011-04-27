/*
 * camera_perspective.cpp
 *
 *  Created on: 20/04/2011
 *      Author: laurence
 */

#include "camera_perspective.h"

namespace camera {

namespace {
// from eigen_utils.h - helper conversion function.
template <typename CvScalarType, typename EScalarType, int H, int W>
inline void to_eigen(const cv::Mat_<CvScalarType>& mat, Eigen::Matrix<EScalarType,H,W>& ep)
{
  for (int r = 0; r < H; ++r)
    for (int c = 0; c < W; ++c)
      ep(r,c) = mat(r,c);
}

template <typename CvScalarType, typename EScalarType>
inline void to_eigen(const cv::Vec<CvScalarType,3>& p, Eigen::Matrix<EScalarType,3,1>& ep)
{
  ep(0) = p[0];
  ep(1) = p[1];
  ep(2) = p[2];
}

template <typename CvScalarType, typename EScalarType>
inline void to_eigen(const cv::Point3_<CvScalarType>& p, Eigen::Matrix<EScalarType,4,1>& ep)
{
  ep(0) = p.x;
  ep(1) = p.y;
  ep(2) = p.z;
  ep(3) = 1;
}

inline Eigen::Vector3d to_eigen_vector3d(const cv::Vec3f& v)
{ Eigen::Vector3d r; to_eigen(v, r); return r; }

template <typename EScalarType>
inline cv::Vec3f to_vec3f(const Eigen::Matrix<EScalarType,4,1>& v)
{
  // FIXME: divide by v(3) ?
  return cv::Vec3f(v(0),v(1),v(2));
}

}

CameraPerspective::CameraPerspective(cv::Mat intrinsics, cv::Mat distortion,
    cv::Mat r, cv::Mat t)
    : intrinsics(intrinsics), distortion(distortion),
      rotation_matrix(r), translation_matrix(t), requires_rotation_and_translation(true) {
  initialize();
}

CameraPerspective::CameraPerspective(cv::Mat intrinsics, cv::Mat distortion)
    : intrinsics(intrinsics), distortion(distortion),
      requires_rotation_and_translation(false) {
  initialize();
}

void CameraPerspective::initialize() {
  // Generate the undistortion maps.
  cv::initUndistortRectifyMap(
        intrinsics,
        distortion,
        cv::Mat(),
        intrinsics,
        cv::Size(640, 480),
        CV_16SC2,
        undistort_map_1,
        undistort_map_2
        );

  // Set the projective/camera transform by default to the identity matrix.
  projective_transform.setIdentity();
  inverse_projective_transform = projective_transform.inverse();

  camera_transform.setIdentity();
  inverse_camera_transform = camera_transform.inverse();

  // Set the camera intrinsics (RGBDCalibration::loadFromFile - depth)
  focal_x = intrinsics.at<double>(0,0);
  focal_y = intrinsics.at<double>(1,1);
  image_center_x = intrinsics.at<double>(0,2);
  image_center_y = intrinsics.at<double>(1,2);

  // Normalise the camera perspective to a predetermined datum if required.
  if (requires_rotation_and_translation) {
    rotate_and_translate();
  }

  // Compute the transforms.
  compute_projective_transform();
}

// From Pose3d::toRightCamera
void CameraPerspective::rotate_and_translate() {
  // TODO(laurencer): add assert to check that rotate_and_translate is required.
  cv::Mat1d to_gl_base(3,3);
  setIdentity(to_gl_base);
  to_gl_base(1,1) = -1;
  to_gl_base(2,2) = -1;
  cv::Mat1d rotation_matrix = to_gl_base.inv() * rotation_matrix.inv() * to_gl_base;
  cv::Mat1d temp_translation_matrix = to_gl_base * (-translation_matrix);

  cv::Vec3d translation_vector = cv::Vec3d(temp_translation_matrix(0,0),
     temp_translation_matrix(1,0), temp_translation_matrix(2,0));

  // convert OpenCV matricies to Eigen matricies and perform translation/rotation
  Eigen::Matrix3d emat;
  to_eigen(rotation_matrix, emat);
  camera_transform.prerotate(emat);
  camera_transform.pretranslate(to_eigen_vector3d(translation_vector));
}

// From Pose3d::intrinsicsTransform
/** Creates an OpenCV Intrinsics matrix in an Eigen 3d transform.
 */
Eigen::Isometry3d CameraPerspective::intrinsics_transform() const
{
  // TODO(laurencer): memoize result.
  Eigen::Isometry3d transform;
  transform.setIdentity();
  transform(0,0) = focal_x;
  transform(1,1) = focal_y;
  transform(0,2) = image_center_x;
  transform(1,2) = image_center_y;
  return transform;
}

// From Pose3d::computeProjectiveTransfrom
void CameraPerspective::compute_projective_transform() {
  Eigen::Isometry3d to_opencv = Eigen::Isometry3d::Identity();
  to_opencv(1,1) = to_opencv(2,2) = -1;

  Eigen::Projective3d projection = Eigen::Projective3d::Identity();

  inverse_camera_transform = camera_transform.inverse();

  projective_transform = intrinsics_transform() * projection * to_opencv * camera_transform;
  inverse_projective_transform = projective_transform.inverse();
}

CameraPerspective::~CameraPerspective() {
	// TODO Auto-generated destructor stub
}

/* From https://github.com/nburrus/nestk/blob/master/ntk/geometry/pose_3d.cpp */
cv::Vec3f CameraPerspective::unproject_from_image(int x, int y, float depth) {
  Eigen::Vector4d img(x * depth, y * depth, depth, 1);
  img = inverse_projective_transform * img;
  return cv::Vec3f(img(0), img(1), img(2));
}

cv::Point3f CameraPerspective::project_to_image(const cv::Point3f &p) {
  Eigen::Vector4d ep;
  to_eigen(p, ep);

  Eigen::Vector4d r = projective_transform * ep;
  r(0) /= r(2);
  r(1) /= r(2);

  return to_vec3f(r);
}



}
