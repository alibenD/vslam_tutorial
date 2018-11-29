/**
  * @Copyright (C) 2018 All rights reserved.
  * @date: 2018
  * @file: solve_100_by_100_formular.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2018-11-27 15:14:35
  * @last_modified_date: 2018-11-27 15:33:27
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <iostream>
#include <chrono>

const int SIZE_MATRIX = 100;

//CODE
int main(int argc, char** argv)
{
  using microseconds = std::chrono::microseconds;
  auto dynamic_matrix = Eigen::MatrixXd::Random(SIZE_MATRIX, SIZE_MATRIX);
  auto dynamic_b = Eigen::VectorXd::Random(SIZE_MATRIX, 1);

  auto start = std::chrono::system_clock::now();
  Eigen::VectorXd solution = dynamic_matrix.colPivHouseholderQr().solve(dynamic_b);
  auto end = std::chrono::system_clock::now();
  auto time_cost_counter = std::chrono::duration_cast<microseconds>(end - start);
  std::cout << "Solve(s): " << double(time_cost_counter.count()) *
                               std::chrono::microseconds::period::num/
                               std::chrono::microseconds::period::den
            << "\t" << double(time_cost_counter.count())
            << std::endl;
  std::cout << "Solution: \n"
            << solution
            << std::endl;
  return 0;
}
