/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: tf_point.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-11-28 18:55:03
  * @last_modified_date: 2018-11-29 13:50:57
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <iostream>
#include <cmath>
#include <Eigen/Geometry>

//CODE
int main(int argc, char** argv)
{
  Eigen::Vector3d point(2,0,0);
  Eigen::Vector3d translation(1,1,0);
  Eigen::AngleAxisd r_vec(M_PI/4, Eigen::Vector3d(0,0,1));
  Eigen::Quaterniond q(r_vec);

  // All is transfomation for point, but not for coordinates
  Eigen::Vector3d new_point = q.inverse() * (point - translation);
  Eigen::Vector3d rec_point = q * new_point + translation;
  std::cout << "Origin:\n"
            << point
            << std::endl
            << "After:\n"
            << new_point
            << std::endl
            << "Recovery:\n"
            << rec_point
            << std::endl;
  
  std::cout << "Quaternion: " << q.coeffs().transpose() << std::endl;
  return 0;
}
