/*
 * mesh_viewer.h
 *
 *  Created on: 27/04/2011
 *      Author: laurence
 */

#ifndef MESH_VIEWER_H_
#define MESH_VIEWER_H_

#include <cv.hpp>
#include <cxcore.h>
#include <highgui.h>

#include "mesh/point_cloud.h"

using namespace cv;

namespace mesh {

class MeshViewer {
  const PointCloud &point_cloud;
public:
  MeshViewer(const PointCloud &point_cloud);
  virtual ~MeshViewer();
};

}

#endif /* MESH_VIEWER_H_ */
