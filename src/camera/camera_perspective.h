/*
 * camera_perspective.h
 *
 *  Created on: 20/04/2011
 *      Author: laurence
 */

#ifndef CAMERA_PERSPECTIVE_H_
#define CAMERA_PERSPECTIVE_H_

#include <cv.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace camera {

class CameraPerspective {
	double focal_x;
	double focal_y;
	double image_center_x;
	double image_center_y;

	cv::Mat intrinsics;
	cv::Mat distortion;

	cv::Mat rotation_matrix;
	cv::Mat translation_matrix;

	bool requires_rotation_and_translation;

  Eigen::Isometry3d camera_transform;
  Eigen::Isometry3d inverse_camera_transform;
	Eigen::Projective3d projective_transform;
	Eigen::Projective3d inverse_projective_transform;

	void initialize();
	void rotate_and_translate();
	Eigen::Isometry3d intrinsics_transform() const;
	void compute_projective_transform();

public:
	cv::Mat undistort_map_1;
	cv::Mat undistort_map_2;

	// r and t will only be set for the rgb camera.
	CameraPerspective(cv::Mat intrinsics, cv::Mat distortion, cv::Mat r, cv::Mat t);
	CameraPerspective(cv::Mat intrinsics, cv::Mat distortion);
	virtual ~CameraPerspective();

	/**
	 * @param x
	 * @param y
	 * @param depth
	 *
	 * @return 3-vector representing the point coordinates out of x-, y- pixel coordinates
	 * and depth.
	 */
	cv::Vec3f unproject_from_image(int x, int y, float depth);
	void unproject_from_image(const cv::Mat1f& pixels, const cv::Mat1b& mask, cv::Mat3f voxels) const;
	cv::Point3f project_to_image(const cv::Point3f &p);
	void project_to_image(const cv::Mat3f& voxels, const cv::Mat1b& mask, cv::Mat3f& pixels) const;
};

}

#endif /* CAMERA_PERSPECTIVE_H_ */
