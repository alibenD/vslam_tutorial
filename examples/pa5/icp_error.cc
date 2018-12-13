/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: icp_error.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-12-07 13:18:24
  * @last_modified_date: 2018-12-13 16:15:15
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
  Eigen::Vector3d pg_mess_centre(0, 0, 0);
  Eigen::Vector3d pe_mess_centre(0, 0, 0);
  std::vector<Eigen::Vector3d> gt_points;
  std::vector<Eigen::Vector3d> es_points;
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
    gt_points.push_back(tg);
    poses_gt.push_back(Sophus::SE3(qg, tg));
    Eigen::Quaterniond qe(qw_e, qx_e, qy_e, qz_e);
    Eigen::Vector3d te(tx_e, ty_e, tz_e);
    es_points.push_back(te);
    //**** The first method to align two curve
    //te = se_diff_ge * te;
    poses_es.push_back(Sophus::SE3(qe, te));
    pg_mess_centre += tg;
    pe_mess_centre += te;
  }
  pg_mess_centre /= poses_es.size();
  pe_mess_centre /= poses_es.size();
  std::cout << "Pg_mess: " << pg_mess_centre.transpose() << std::endl;
  std::cout << "Pe_mess: " << pe_mess_centre.transpose() << std::endl;
  for(size_t i=0; i< poses_es.size(); i++)
  {
    es_points[i] -= pe_mess_centre;
    gt_points[i] -= pg_mess_centre;
  }

  Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
  for(size_t i=0; i<poses_es.size(); i++)
  {
    W += Eigen::Vector3d(es_points[i][0], es_points[i][1], es_points[i][2])
        *Eigen::Vector3d(gt_points[i][0], gt_points[i][1], gt_points[i][2]).transpose();
  }
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU|Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  if(U.determinant() * V.determinant() < 0)
  {
    for(int x=0; x<3; ++x)
    {
      U(x, 2) *= -1;
    }
  }
  Eigen::Matrix3d R = U * (V.transpose());
  Eigen::Vector3d t = Eigen::Vector3d(pe_mess_centre[0], pe_mess_centre[1], pe_mess_centre[2]) - R * Eigen::Vector3d(pg_mess_centre[0], pg_mess_centre[1], pg_mess_centre[2]);
  std::cout << "R:\n " << R << std::endl
            << "t: " << t.transpose() << std::endl;
  Sophus::SE3 icp_eg(R, t);
  for(auto& pose_es:poses_es)
  {
    pose_es = icp_eg.inverse() * pose_es;
  }
  drawTraj(poses_gt, poses_es);
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
