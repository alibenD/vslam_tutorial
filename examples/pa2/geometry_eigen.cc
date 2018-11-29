/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: geometry_eigen.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-11-27 15:35:27
  * @last_modified_date: 2018-11-29 14:26:27
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <Eigen/Geometry>
#include <iostream>
#include <cmath>

//CODE
int main(int argc, char** argv)
{
  Eigen::Quaterniond q1(0.55, 0.3, 0.2, 0.2);
  Eigen::Quaterniond q2(-0.1, 0.3, -0.7, 0.2);

  Eigen::Vector3d t1(0.7, 1.1, 0.2);
  Eigen::Vector3d t2(-0.1, 0.4, 0.8);

  Eigen::Vector3d p1(0.5, -0.1, 0.2);
  Eigen::Vector3d p2;

  // q,t is Tcw, which is different from Twc.
  Eigen::Vector3d pw;
  pw = q1.normalized().inverse() * (p1 - t1);
  std::cout << "pw:\n"
            << pw
            << std::endl;

  p2 = q2.normalized() * pw + t2;
  std::cout << "p2:\n"
            << p2
            << std::endl;

  // Matrix method
  //Eigen::Isometry3d r1 = Eigen::Isometry3d::Identity();
  //Eigen::Isometry3d r2 = Eigen::Isometry3d::Identity();
  //r1.rotate(q1);
  //r1.pretranslate(t1);
  //r2.rotate(q2);
  //r2.pretranslate(t2);
  //std::cout << "Transform matrix1 = \n"
  //          << r1.matrix()
  //          << std::endl;
  //pw = r1.inverse() * p1;
  //p2 = r2 * pw;
  //std::cout << "p2_:\n"
  //          << p2
  //          << std::endl;
  return 0;
}
