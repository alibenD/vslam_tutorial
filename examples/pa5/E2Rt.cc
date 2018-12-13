//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

using namespace Eigen;

#include <sophus/so3.h>

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    // START YOUR CODE HERE
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(E, Eigen::ComputeThinU | Eigen::ComputeThinV);
    auto V = svd.matrixV();
    auto U = svd.matrixU();
    auto S = U.inverse() * E * V.transpose().inverse();
    auto kexi = S(0,0);
    auto factor_res = 1.0 / kexi;
    std::cout << "U:\n" << U
              << "\nV:\n" << V
              << "\nS:\n" << S
              << "\nE:\n" << U * S * V.transpose()
              << std::endl;
    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    Matrix3d t_wedge1;
    Matrix3d t_wedge2;
    Eigen::Matrix3d R90 = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)).toRotationMatrix();
    std::cout << "R90:\n" << R90 << std::endl;
    t_wedge1 =  U * R90 * S * U.transpose() * factor_res;
    t_wedge2 =  U * -R90 * S * U.transpose() * factor_res;
    auto factor_rot = sqrt(t_wedge1(0,1)*t_wedge1(0,1) +
                           t_wedge1(0,2)*t_wedge1(0,2) +
                           t_wedge1(1,2)*t_wedge1(1,2));
    std::cout << "Rot_factor: " << factor_rot << std::endl;

    Matrix3d R1;
    Matrix3d R2;

    //R1 = U * R90.transpose() * V.transpose() * sqrt(2) * factor_rot;
    //R2 = U * -R90.transpose() * V.transpose() * sqrt(2) * factor_rot;
    R1 = t_wedge1.inverse() * E;
    R2 = t_wedge2.inverse() * E;
    // END YOUR CODE HERE

    cout << "R1 = " << R1 << endl;
    cout << "R2 = " << R2 << endl;
    cout << "t1 = " << Sophus::SO3::vee(t_wedge1) << endl;
    cout << "t2 = " << Sophus::SO3::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "t^R = " << tR << endl;

    return 0;
}
