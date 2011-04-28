/*
 * mesh_viewer.cc
 *
 *  Created on: 27/04/2011
 *      Author: laurence
 */

#include "mesh_viewer.h"

#include <cv.hpp>
#include <cxcore.h>
#include <highgui.h>
#include <GL/gl.h>
#include <glog/logging.h>

namespace mesh {

namespace {

struct viewport {
  int display_center_x, display_center_y, display_center_z, prev_x, x_slider;
  const PointCloud *pc;
};

void on_opengl(void *param) {
  viewport *view = static_cast<viewport*>(param);
  const PointCloud *point_cloud = view->pc;
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glBegin(GL_POINTS);



    for (int i = 0; i < point_cloud->vertices.size(); ++i)
    {
      const cv::Point3f& v = point_cloud->vertices[i];
      glColor3f(point_cloud->colours[i][0]/255.0, point_cloud->colours[i][1]/255.0, point_cloud->colours[i][2]/255.0);
      glVertex3f(v.x, v.y, v.z);
    }
  glEnd();

  GLdouble m[16];
   glMatrixMode(GL_MODELVIEW);
   glGetDoublev(GL_MODELVIEW_MATRIX, m);
   glLoadIdentity();
   glTranslatef(view->display_center_x,view->display_center_y,view->display_center_z);
   glRotatef(view->prev_x*0.02, 0,1,0);
   glRotatef(0*0.2, 1,0,0);
   glTranslatef(-view->display_center_x,-view->display_center_y,-view->display_center_z);
   glMultMatrixd(m);
  glFlush();
}


void on_view_change(int x, void *param) {
  viewport *view = static_cast<viewport*>(param);
  view->prev_x = x;
}

}


MeshViewer::MeshViewer(const PointCloud &point_cloud)
: point_cloud(point_cloud) {
  viewport *view = new viewport();
  view->pc = &point_cloud;

  namedWindow("scene", CV_WINDOW_AUTOSIZE);
  createOpenGLCallback("scene", &on_opengl, (void*) (view));
  createTrackbar("view", "scene", &view->x_slider, 100, on_view_change, (void*) view);
}

MeshViewer::~MeshViewer() {
  cvDestroyWindow("scene");
}

}
