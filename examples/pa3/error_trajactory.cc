/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: error_trajactory.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-12-07 11:11:10
  * @last_modified_date: 2018-12-07 13:15:12
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

using PoseV = std::vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>;
//CODE
int main(int argc, char** argv)
{
  std::string est_traj_file = "../data/estimated.txt";
  std::string gt_traj_file = "../data/groundtruth.txt";

  std::ifstream file_es(est_traj_file);
  std::ifstream file_gt(gt_traj_file);
  double timestamp, tx, ty, tz, qx, qy, qz, qw;
  std::string line_es, line_gt;

  PoseV poses_gt;
  PoseV poses_es;

  double sum_e = 0.;
  while(!file_es.eof() && !file_gt.eof())
  {
    std::getline(file_es, line_es);
    std::getline(file_gt, line_gt);
    std::stringstream sin_es(line_es);
    std::stringstream sin_gt(line_gt);
    sin_es >> timestamp
           >> tx >> ty >> tz
           >> qx >> qy >> qz >> qw;
    Eigen::Quaterniond q_es(qw, qx, qy, qz);
    Eigen::Vector3d t_es(tx, ty, tz);
    Sophus::SE3 se3_es(q_es, t_es);
    poses_es.push_back(se3_es);

    sin_gt >> timestamp
           >> tx >> ty >> tz
           >> qx >> qy >> qz >> qw;
    Eigen::Quaterniond q_gt(qw, qx, qy, qz);
    Eigen::Vector3d t_gt(tx, ty, tz);
    Sophus::SE3 se3_gt(q_gt, t_gt);
    poses_gt.push_back(se3_gt);
    
    //auto error_mat = se3_gt.matrix().inverse() * se3_es.matrix();
    Sophus::SE3 se3_error = se3_gt.inverse() * se3_es;
    double error = se3_error.log().norm();
    //std::cout << "se3_error:\n" << se3_error << std::endl;
    //std::cout << "error: " << std::endl;
    sum_e += error * error;
  }

  std::cout << "RMSE: " << sqrt(sum_e / poses_es.size()) << std::endl;
  return 0;
}
