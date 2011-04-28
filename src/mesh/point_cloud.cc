/*
 * point_cloud.cpp
 *
 *  Created on: 27/04/2011
 *      Author: laurence
 */

#include "point_cloud.h"

#include "camera/image.h"

using namespace camera;

namespace mesh {

namespace {

inline bool is_yx_in_range(const cv::Mat& image, int y, int x){
  return (x >= 0) && (y >= 0) && (x < image.cols) && (y < image.rows);
}

} // End namespace

PointCloud::PointCloud() {
  // TODO Auto-generated constructor stub

}

void PointCloud::add_image(const Image& image, cv::Mat1b mask){
  const cv::Mat &depth = image.depth;
  const cv::Mat &rgb = image.rgb;

  cv::Mat3f voxels (depth.size());
  cv::Mat3f rgb_points (depth.size());

  image.depth_perspective->unproject_from_image(depth, mask, voxels);
  image.rgb_perspective->project_to_image(voxels, mask, rgb_points);

  for (int r = 0; r < voxels.rows; ++r)
  {
    Vec3f* voxels_data = voxels.ptr<Vec3f>(r);
    const uchar* mask_data = mask.ptr<uchar>(r);
    for (int c = 0; c < voxels.cols; ++c)
    {
      if (!mask_data[c])
        continue;

      Vec3b colour (0,0,0);

      Point3f prgb = rgb_points.at<Point3f>(r,c);
      int y = floor(prgb.y + 0.5);
      int x = floor(prgb.x + 0.5);
      if (is_yx_in_range(rgb, y, x))
      {
        Vec3b bgr = rgb.at<Vec3b>(y, x);
        colour = Vec3b(bgr[2], bgr[1], bgr[0]);
      }
      vertices.push_back(voxels_data[c]);
      colours.push_back(colour);
    }
  }
}

PointCloud::~PointCloud() {
  // TODO Auto-generated destructor stub
}

}
