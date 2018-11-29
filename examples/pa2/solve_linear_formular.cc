/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: solve_linear_formular.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-11-20 11:08:23
  * @last_modified_date: 2018-11-27 14:59:11
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Householder>
#include <Eigen/Dense>
#include <chrono>
using namespace std;
using namespace Eigen;

const int SIZE_MATRIX = 10;

//CODE
int main()
{
  using microseconds = std::chrono::microseconds;
  // 4*4 Matrix
  Eigen::Matrix3f simple_matrix;
  simple_matrix << 1,  2,  3,  4,
                   5,  6,  7,  8,
                   10;
  Eigen::Vector3f b1(3, 3, 4);
  std::cout << "A: \n" << simple_matrix
            << std::endl
            << "b: \n" << b1
            << std::endl;
  auto start = std::chrono::system_clock::now();
  Eigen::Vector3f solution1 = simple_matrix.colPivHouseholderQr().solve(b1);
  auto end= std::chrono::system_clock::now();
  auto time_cost = std::chrono::duration_cast<microseconds>(end - start);
  std::cout << "QR(s): " << double(time_cost.count())*
                            std::chrono::microseconds::period::num/
                            std::chrono::microseconds::period::den
            << "\t " << double(time_cost.count())
            << std::endl;
  std::cout << "Solution: \n"
            << solution1
            << std::endl;

  //============================================================
  // LDLT
  start = std::chrono::system_clock::now();
  auto solution2 = simple_matrix.householderQr().solve(b1);
  end= std::chrono::system_clock::now();
  time_cost = std::chrono::duration_cast<microseconds>(end - start);
  std::cout << "HouseQR:(s): " << double(time_cost.count())*
                            std::chrono::microseconds::period::num/
                            std::chrono::microseconds::period::den
            << "\t " << double(time_cost.count())
            << std::endl;
  std::cout << "Solution: \n"
            << solution2
            << std::endl;

  // 100*100 matrix
  //auto dynamic_matrix = Eigen::MatrixXd::Random(SIZE_MATRIX, SIZE_MATRIX);
  //std::cout << dynamic_matrix << std::endl;
  return 0;
}
