/*
 * point_cloud.h
 *
 *  Created on: 27/04/2011
 *      Author: laurence
 */

#ifndef POINT_CLOUD_H_
#define POINT_CLOUD_H_

#include <cv.h>
#include <vector>

#include "camera/image.h"

using namespace camera;
using namespace cv;

namespace mesh {

class PointCloud {


public:
  PointCloud();
  virtual ~PointCloud();

  void add_image(const Image& image, cv::Mat1b mask);
  std::vector<cv::Point3f> vertices;
  std::vector<cv::Vec3b> colours;
};

}

#endif /* POINT_CLOUD_H_ */
