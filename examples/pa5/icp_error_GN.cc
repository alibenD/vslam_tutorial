/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: icp_error.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-12-07 13:18:24
  * @last_modified_date: 2018-12-13 15:58:13
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <sophus/se3.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <unistd.h>
#include <thread>
#include <limits>

#include <pangolin/pangolin.h>

using PoseV = std::vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
void drawTraj(const PoseV& poses1, const PoseV& poses2);
//CODE
int main(int argc, char** argv)
{
  std::string traj_path = "../data/compare.txt";
  std::ifstream traj_file(traj_path);
  double timestampe, tx_e, ty_e, tz_e, qx_e, qy_e, qz_e, qw_e;
  double timestampg, tx_g, ty_g, tz_g, qx_g, qy_g, qz_g, qw_g;
  std::string line;
  PoseV poses_gt;
  PoseV poses_es;
  std::getline(traj_file, line);
  std::stringstream sin_t(line);
  sin_t >> timestampe >> tx_e >> ty_e >> tz_e
      >> qx_e >> qy_e >> qz_e >> qw_e
      >> timestampg >> tx_g >> ty_g >> tz_g
      >> qx_g >> qy_g >> qz_g >> qw_g;
  Sophus::SE3 se_g(Eigen::Quaterniond(qw_g, qx_g, qy_g, qz_g),
                   Eigen::Vector3d(tx_g, ty_g, tz_g));
  Sophus::SE3 se_e(Eigen::Quaterniond(qw_e, qx_e, qy_e, qz_e),
                   Eigen::Vector3d(tx_e, ty_e, tz_e));
  Sophus::SE3 se_diff_ge = se_g * se_e.inverse();
  poses_gt.push_back(se_g);
  poses_es.push_back(se_e);
  std::cout << "diff:\n" << se_diff_ge << std::endl;
  while(!traj_file.eof())
  {
    std::getline(traj_file, line);
    std::stringstream sin(line);
    sin >> timestampe >> tx_e >> ty_e >> tz_e
        >> qx_e >> qy_e >> qz_e >> qw_e
        >> timestampg >> tx_g >> ty_g >> tz_g
        >> qx_g >> qy_g >> qz_g >> qw_g;
    Eigen::Quaterniond qg(qw_g, qx_g, qy_g, qz_g);
    Eigen::Vector3d tg(tx_g, ty_g, tz_g);
    poses_gt.push_back(Sophus::SE3(qg, tg));
    Eigen::Quaterniond qe(qw_e, qx_e, qy_e, qz_e);
    Eigen::Vector3d te(tx_e, ty_e, tz_e);
    //**** The first method to align two curve
    //te = se_diff_ge * te;
    poses_es.push_back(Sophus::SE3(qe, te));
  }

  int iteration = 100;
  auto num_poses = poses_es.size();
  double cost = 0;
  double last_cost = 0.;
  for(auto iter = 0; iter < iteration; ++iter)
  {
    // Compute Hessian Matrix
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Vector3d point(1,1,1);
    Eigen::Vector4d point_homo(1,1,1,1);
    Vector6d b = Vector6d::Zero();
    cost = 0;
    for(size_t i = 0; i < num_poses; ++i)
    {
      // Compute error(size: 6x1)
      Eigen::Matrix<double, 3, 6> J;
      Sophus::SE3 g_se = poses_gt[i];
      Sophus::SE3 e_se = poses_es[i];
      Sophus::SE3 ge_se = se_diff_ge;

      Eigen::Matrix3d Re = e_se.rotation_matrix();
      Eigen::Vector3d te = e_se.translation();
      Eigen::Matrix3d Rg = g_se.rotation_matrix();
      Eigen::Vector3d tg = g_se.translation();
      Eigen::Matrix3d Rge = ge_se.rotation_matrix();
      Eigen::Vector3d tge = ge_se.translation();

      //auto e_point4d = e_se.matrix() * g_se.matrix().inverse().eval() * ge_se.matrix() * point_homo;
      Eigen::Matrix4d residual_tf = e_se.matrix() * g_se.matrix().inverse().eval() * ge_se.matrix();
      Eigen::Vector4d e_point4d = residual_tf * point_homo;
      //std::cout << "Residual_TF:\n" << residual_tf << std::endl;
      Eigen::Vector3d e_point3d(e_point4d[0],
                                e_point4d[1],
                                e_point4d[2]);
      //std::cout << "Homo: " << point_homo.transpose() << std::endl;
      //std::cout << "TF_point4d: " << e_point4d << std::endl;
      //std::cout << "TF_point4d: " << e_point4d.transpose() << std::endl;
      //std::cout << "TF_point3d: " << e_point3d.transpose() << std::endl;
      auto error = point - e_point3d;
      //std::cout << "Error_point: \n" << error << std::endl;
      Eigen::Vector4d p_tf = ge_se.matrix() * point_homo;
      //std::cout << "P_tf_ge:\n" << p_tf.transpose() << std::endl;
      Eigen::Matrix3d p_hat = Eigen::Matrix3d::Zero();
      p_hat(0,1) = -p_tf[2];
      p_hat(0,2) = p_tf[1];
      p_hat(1,0) = p_tf[2];
      p_hat(1,2) = -p_tf[0];
      p_hat(2,0) = -p_tf[1];
      p_hat(2,1) = p_tf[0];
      //std::cout << "p_hat:\n" << p_hat << std::endl;
      Eigen::Matrix3d J11 = Re * Rg.inverse().eval();
      Eigen::Matrix3d J12 = -Re * Rg.inverse().eval() * p_hat;
      // Compute Jaccobian Matrix(size: 3x6)
      J(0,0) = J11(0,0);
      J(0,1) = J11(0,1);
      J(0,2) = J11(0,2);
      J(0,3) = J12(0,0);
      J(0,4) = J12(0,1);
      J(0,5) = J12(0,2);
      J(1,0) = J11(1,0);
      J(1,1) = J11(1,1);
      J(1,2) = J11(1,2);
      J(1,3) = J12(1,0);
      J(1,4) = J12(1,1);
      J(1,5) = J12(1,2);
      J(2,0) = J11(2,0);
      J(2,1) = J11(2,1);
      J(2,2) = J11(2,2);
      J(2,3) = J12(2,0);
      J(2,4) = J12(2,1);
      J(2,5) = J12(2,2);
      // Compute Hessian Matrix(6X6)( J.transpose() * J(3*6) )
      H += J.transpose() * J;
      // Compute error b(size: 6x1)
      b += -J.transpose() * error;
      cost += error.norm();
    }
    // Compute delta_se
    Vector6d dx;
    dx = H.ldlt().solve(b);
    if(isnan(dx[0]))
    {
      std::cout << "Result is nan!" << std::endl;
      break;
    }
    if(iter > 0 && cost >= last_cost)
    {
      std::cout << "cost: " << cost << ", last cost: " << last_cost << std::endl;
      break;
    }
    // Update e_diff
    se_diff_ge = Sophus::SE3::exp(dx) * se_diff_ge;
    //std::cout << "T:\n" << se_diff_ge.matrix() << std::endl;
    //std::cout << "diff:\n" << se_diff_ge << std::endl;
    last_cost = cost;
    std::cout << "Iteration: " << iter << " cost=" << std::cout.precision(12) << cost << std::endl;
  }
  //std::cout << "diff:\n" << se_diff_ge << std::endl;
  std::cout << "Rge:\n" << se_diff_ge.matrix() <<  std::endl;
  std::cout << "tge:\n" << se_diff_ge.translation().transpose() <<  std::endl;
  for(auto& pose_es:poses_es)
  {
    pose_es = se_diff_ge * pose_es;
  }
  drawTraj(poses_gt, poses_es);
  //std::thread draw(drawTraj, poses_gt, poses_es);
  //std::thread draw([](){std::cout << "Hello world!" << std::endl;});
  //draw.join();
  return 0;
}

void drawTraj(const PoseV& poses1, const PoseV& poses2)
{
  if(poses1.empty() || poses2.empty())
  {
    std::cerr << "Trajectory is empty!" << std::endl;
    return;
  }
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  auto proj_matrix = pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000);
  auto mvl = pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0., -1., 0.);
  pangolin::OpenGlRenderState s_cam(proj_matrix, mvl);;
  pangolin::View& d_cam = pangolin::CreateDisplay().SetBounds(0., 1., pangolin::Attach::Pix(175), 1., -1024.f/768.f).SetHandler(new pangolin::Handler3D(s_cam));
  while(pangolin::ShouldQuit() == false)
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.f, 1.f, 1.f, 1.f);
    glLineWidth(2);
    for(size_t i=0; i < poses1.size() - 1; i++)
    {
      {
        glColor3f(0., 0., 1.);
        glBegin(GL_LINES);
        auto p1 = poses1[i], p2 = poses1[i + 1];
        glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
        glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
        glEnd();
      }
      {
        glColor3f(1., 0.0f, 0.);
        glBegin(GL_LINES);
        auto p1 = poses2[i], p2 = poses2[i + 1];
        glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
        glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
        glEnd();
      }
    }
    pangolin::FinishFrame();
    usleep(5000);
  }
}
