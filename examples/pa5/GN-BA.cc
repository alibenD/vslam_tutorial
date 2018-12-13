//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector2d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "../data/p3d.txt";
string p2d_file = "../data/p2d.txt";

int main(int argc, char **argv)
{
    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    std::ifstream file_2d(p2d_file);
    std::ifstream file_3d(p3d_file);
    std::string line;
    while(!file_2d.eof())
    {
      std::getline(file_2d, line);
      std::stringstream sin(line);
      double x,y;
      sin >> x;
      sin >> y;
      p2d.push_back(Eigen::Vector2d(x,y));
    }
    while(!file_3d.eof())
    {
      std::getline(file_3d, line);
      std::stringstream sin(line);
      double x,y,z;
      sin >> x;
      sin >> y;
      sin >> z;
      p3d.push_back(Eigen::Vector3d(x,y,z));
    }
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());
    int iterations = 100;
    double cost = 0;
    double lastCost = 0; //std::numeric_limits<double>::max();
    //std::cout << "Max: " << lastCost << std::endl;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;
    Sophus::SE3 T_esti; // estimated pose
    for (int iter = 0; iter < iterations; iter++)
    {
      Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
      Vector6d b = Vector6d::Zero();
      cost = 0;
      // compute cost
      for (int i = 0; i < nPoints; i++)
      {
        // compute cost for p3d[I] and p2d[I]
        // START YOUR CODE HERE 
        Eigen::Vector3d point3d = p3d[i];
        Eigen::Vector2d point2d = p2d[i];
        Eigen::Vector4d point3d_homo = Eigen::Vector4d(point3d[0], point3d[1], point3d[2], 1);
        Eigen::Vector4d  point4d_tf = T_esti.matrix() * point3d_homo;
        Eigen::Vector3d point3d_tf = Eigen::Vector3d(point4d_tf[0],
                                          point4d_tf[1],
                                          point4d_tf[2]);
        point3d_tf = K * point3d_tf;
        Eigen::Vector2d reproj_point2d = Eigen::Vector2d(point3d_tf[0] / point3d_tf[2],
                                            point3d_tf[1] / point3d_tf[2]);
        Eigen::Vector2d error = -point2d + reproj_point2d;
        cost += error.norm();
        // END YOUR CODE HERE
        // compute jacobian
        Matrix<double, 2, 6> J;
        // START YOUR CODE HERE 
        auto fn = pow(point4d_tf[2], 2);
        J(0, 0) = fx / point4d_tf[2];
        J(0, 1) = 0;
        J(0, 2) = -fx * point4d_tf[0] / fn;
        J(0, 3) = -fx * point4d_tf[0] * point4d_tf[1] / fn;
        J(0, 4) = fx + fx * pow(point4d_tf[0], 2) / fn;
        J(0, 5) = -fx * point4d_tf[1] / point4d_tf[2];
        J(1, 0) = 0;
        J(1, 1) = fy / point4d_tf[2];
        J(1, 2) = -fy * point4d_tf[1] / fn;
        J(1, 3) = -fy - fy * pow(point4d_tf[1], 2) / fn;
        J(1, 4) = fy * point4d_tf[0] * point4d_tf[1] / fn;
        J(1, 5) = fy * point4d_tf[0] / point4d_tf[2];
        // END YOUR CODE HERE
        H += J.transpose() * J;
        b += -J.transpose() * error;
      }
      // solve dx 
      Vector6d dx;
      // START YOUR CODE HERE 
      //dx = H.ldlt().solve(b);
      dx = H.colPivHouseholderQr().solve(b);
      //std::cout << "H:\n" << H
      //          << "\nb:\n" << b
      //          << "\ndx:\n" << dx
      //          << std::endl;
      // END YOUR CODE HERE
      if (isnan(dx[0]))
      {
         cout << "result is nan!" << endl;
         break;
      }
      if (iter > 0 && cost >= lastCost)
      {
        // cost increase, update is not good
        cout << "cost: " << cost << ", last cost: " << lastCost << endl;
        break;
      }
      // update your estimation
      // START YOUR CODE HERE 
      T_esti = Sophus::SE3::exp(dx) * T_esti;
      std::cout << "T:\n" << T_esti.matrix().inverse() << std::endl;
      //std::cout << "T_inverse:\n" << T_esti.matrix().inverse() << std::endl;
      // END YOUR CODE HERE
      lastCost = cost;
      cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }
    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
