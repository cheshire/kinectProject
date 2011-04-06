/*
 * image_corrector.cc
 *
 * Calibration algorithm from
 * https://github.com/nburrus/nestk/blob/master/ntk/camera/calibration.cpp
 * and https://github.com/nburrus/nestk/blob/master/ntk/camera/rgbd_processor.cpp
 * was used.
 *
 *  Created on: 06/04/2011
 *      Author: Laurence Rouesnel
 */

#include "image_corector.h"

#include <gflags/gflags.h>
#include <cmath.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

DEFINE_string(distortion_data, "distortion.xml", "The path of the distortion correction data");

namespace camera {

namespace {

// From https://github.com/nburrus/nestk/blob/master/ntk/geometry/pose_3d.cpp
Eigen::Isometry3d intrinsics_transform(CameraCalibrationData& data) const {
	Eigen::Isometry3d m;
	m.setIdentity();
	m(0,0) = data.focal_x;
	m(0,2) = data.image_center_x;
	m(1,1) = data.focal_y;
	m(1,2) = data.image_center_y;
	return m;
}

void convert_depth_matrix_to_meters(Mat *mat) {
  Mat temp(Size(640, 480), CV_32FC1, Scalar(1));
  mat->convertTo(temp, CV_32FC1, -0.0030711016, 3.3309495161);
  cv::pow(temp, -1.0, *mat);
}

void alt_convert_depth_matrix_to_meters(Mat *mat) {
	Mat temp(Size(640, 480), CV_32FC1, Scalar(7.5e-02 * 8.0 * 580));
	Mat offset(Size(640, 480), CV_32FC1, Scalar(1090));
	offset -= *mat;
	*mat = temp / offset;
}

inline bool is_yx_in_range(const cv::Mat& image, int y, int x)
{ return (x >= 0) && (y >= 0) && (x < image.cols) && (y < image.rows); }

}

ImageCorrector::ImageCorrector() {
	load_calibration_data();
}

// https://github.com/nburrus/nestk/blob/master/ntk/camera/calibration.cpp
ImageCorrector::load_calibration_data() {
	FileStorage fs(FLAGS_distortion_data, FileStorage::READ);

	// Load the matricies from the calibration file.
	fs["rgb_intrinsics"] >> calibration_data.rgb_intrinsics;
	fs["rgb_distortion"] >> calibration_data.rgb_distortion;
	fs["depth_intrinsics"] >> calibration_data.depth_intrinsics;
	fs["depth_distortion"] >> calibration_data.depth_distortion;

	// Create the undistortion map.
	cv::initUndistortRectifyMap(
			calibration_data.rgb_intrinsics,
			calibration_data.rgb_distortion,
			cv::Mat(),
			calibration_data.rgb_intrinsics,
			cv::Size(640, 480),
			cv::CV_16SC2,
			rgb_undistort_map_1,
			rgb_undistort_map_2
			);

	cv::initUndistortRectifyMap(
			calibration_data.depth_intrinsics,
			calibration_data.depth_distortion,
			cv::Mat(),
			calibration_data.depth_intrinsics,
			cv::Size(640, 480),
			cv::CV_16SC2,
			depth_undistort_map_1,
			depth_undistort_map_2
			);

	// Load the transformations for aligning the images.
	// See https://github.com/nburrus/nestk/blob/master/ntk/geometry/pose_3d.cpp
	cv::Mat r,t;
	fs["R"] >> r;
	fs["T"] >> t;

	cv::Mat to_gl_base(3,3);
	cv::setIdentity(to_gl_base);

	to_gl_base(1, 1) = -1;
	to_gl_base(2, 2) = -1;

	rgb_camera.r = to_gl_base.inv() * r.inv() * to_gl_base;
	rgb_camera.t = to_gl_base & (-t);

	Eigen::Isometry3d camera_transform;
	camera_transform.setIdentity();

	// Convert rotation_matrix to an Eigen::Matrix
	Eigen::Matrix3d rotation_matrix;
	for (int r = 0; r < 3; ++r) {
		for (int c = 0; c < 3; ++c) {
			rotation_matrix(r, c) = rgb_camera.r(r, c);
		}
	}

	// Apply the translation and rotation matricies to the
	// camera transform matrix.
	camera_transform.prerotate(rotation_matrix);
	camera_transform.pretranslate(Eigen::Vector3d(
			rgb_camera.t[0],
			rgb_camera.t[1],
			rgb_camera.t[2]));


	// Compute the 'projective transform'
	Eigen::Isometry3d to_opencv = Eigen::Isometry3d::Identity();
	to_opencv(1,1) = to_opencv(2,2) = -1;
	Eigen::Projective3d projection = Eigen::Projective3d::Identity();
	Eigen::Isometry3d intrinsics = intrinsics_transform(rgb_camera);

	rgb_camera.projective_transform
		= intrinsics * projection * to_opencv * camera_transform;
}

/* Aligns the RGB image with the depth image in place. */
void ImageCorrector::align(RgbDepthFrame *frame) {
	cv::Mat mapped_depth(frame->size(), frame->depth_image.type()),
			mapped_colour(frame->size(), frame->rgb_image.type());

	const cv::Mat& depth_image = frame->depth_image;
	const cv::Mat& rgb_image = frame->rgb_image;

	for (int i = 0, rows = depth_image.rows; i < rows; i++) {
		for (int j = 0, cols = depth_image.cols; j < cols; j++) {

			if (!is_yx_in_range(depth_image, i, j)) {
				continue;
			}

			double depth = depth_image.at<double>(i,j);
			cv::Point3f p = unproject_from_image(i, j, depth);
			cv::Point3f prgb = project_to_image(p, rgb_camera.projective_transform);

			int x = floor(prgb.x + 0.5);
			int y = floor(prgb.y + 0.5);
			if (is_yx_in_range(rgb_image, x, y)) {
				cv::Vec bgr = rgb_image(y, x);
				mapped_colour(i, j) = bgr;
				mapped_depth(y, x) = prgb.z;
			}

		}
	}

	frame->rgb_image = mapped_colour;
	frame->depth_image = mapped_depth;

}

/* From https://github.com/nburrus/nestk/blob/master/ntk/geometry/pose_3d.cpp */
cv::Vec3f unproject_from_image(int x, int y, double depth) {
	Eigen::Vector4d ep (x, y, depth, 1);
	Eigen::Vector4d r (ep(0) * ep(2), ep(1) * ep(2), ep(2), 1);
	return cv::Vec3f(r(0), r(1), r(2));
}

/* From https://github.com/nburrus/nestk/blob/master/ntk/geometry/pose_3d.cpp */
cv::Point3f project_to_image(const cv::Point3f& p, Eigen::Projective3d projective_transform) {
	// Translate the cvPoint to an EigenVector.
	Eigen::Vector4d ep(p.x, p.y, p.z, 1);

	// Project transform will be set for the rgb camera.
	p = projective_transform * p;

	p(0) /= p(2);
	p(1) /= p(2);

	return cv::Point3f(p(0), p(1), p(2));
}

/* Undistorts the image in place (this affects the field of view).
 *
 * Note: this does not align the RGB and Depth Images.
 */
void ImageCorrector::undistort(RgbDepthFrame *frame) {
	cv::Mat depth_m, rgb;

	// 1. Convert depth matrix to meters.
	// This does an in-place conversion.
	convert_depth_matrix_to_meters(&(frame.depth_image));

	// 2. Undistort the images using the calibration data.
	cv::remap(frame.rgb_image,
			rgb,
			rgb_undistort_map_1,
			rgb_undistort_map_2,
			CV_INTER_LINEAR);

	cv::remap(frame.depth_image,
				depth,
				depth_undistort_map_1,
				depth_undistort_map_2,
				CV_INTER_LINEAR);

	// 3. Copy the undistorted images back to the frame.
	depth_m.copyTo(frame->depth_image);
	rgb.copyTo(frame->rgb_image);
}

}