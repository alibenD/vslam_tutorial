/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: example_pangolin.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-04-03 20:35:49
  * @last_modified_date: 2019-04-04 22:42:32
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <pangolin/pangolin.h>

//CODE

void DrawCamera(pangolin::OpenGlMatrix& Twc)
{
  const float &w = 0.15;
  const float h = w*0.75;
  const float z = w*0.6;
  glPushMatrix();
#ifdef HAVE_GLES
  glMultMatrixf(Twc.m);
#else
  glMultMatrixd(Twc.m);
#endif
  glLineWidth(2);
  glColor3f(0.0f, 1.0f, 0.0f);

  glBegin(GL_LINES);
  glVertex3f(0,0,0);
  glVertex3f(w,h,z);
  glVertex3f(0,0,0);
  glVertex3f(w,-h,z);
  glVertex3f(0,0,0);
  glVertex3f(-w,-h,z);
  glVertex3f(0,0,0);
  glVertex3f(-w,h,z);

  glVertex3f(w,h,z);
  glVertex3f(w,-h,z);

  glVertex3f(-w,h,z);
  glVertex3f(-w,-h,z);

  glVertex3f(-w,h,z);
  glVertex3f(w,h,z);

  glVertex3f(-w,-h,z);
  glVertex3f(w,-h,z);
  glEnd();
  glPopMatrix();
}

int main(int argc, char** argv)
{
  pangolin::CreateWindowAndBind("Example for Pangolin", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::CreatePanel("menu").SetBounds(0.,1.,0.,pangolin::Attach::Pix(175));
  pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
  pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
  pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
  pangolin::Var<bool> menuGraph("menu.Show Graph", true, true);
  pangolin::Var<bool> menuReset("menu.Reset", false, false);

  float viewpoint_x = 0;
  float viewpoint_y = -10;
  float viewpoint_z = -0.1;
  float viewpoint_focal = 2000;
  // Camera Rendoer Obj
  pangolin::OpenGlRenderState state_cam(
        pangolin::ProjectionMatrix(1024, 768, viewpoint_x, viewpoint_y, 512, 386, 0.1, 1000),
        pangolin::ModelViewLookAt(viewpoint_x, viewpoint_y, viewpoint_z,0, 0, 0, 0.0, -1., 0.));
  pangolin::View& display_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f).SetHandler(new pangolin::Handler3D(state_cam));

  pangolin::OpenGlMatrix Twc;
  Twc.SetIdentity();
  while(true)
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    state_cam.Follow(Twc);


    display_cam.Activate(state_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    DrawCamera(Twc);
    pangolin::FinishFrame();
  }
  return 0;
}
