/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: test_frame.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-02-26 12:06:02
  * @last_modified_date: 2019-02-27 16:24:03
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <visual_slam/frame.hh>
#include <visual_slam/utils/io.hh>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <limits>

//CODE
namespace ak
{

class FrameTest : public testing::Test
{
  protected:
    virtual void SetUp() override
    {
      image_path_ = "/Users/aliben/project/data/slam_data/dataset/sequences/00/image_0/000000.png";
      test_image_ = cv::imread(image_path_, cv::IMREAD_GRAYSCALE);
      test_key_points_.push_back(cv::KeyPoint(842, 50, 31, 24.5380497, 146));
      test_key_points_.push_back(cv::KeyPoint(832, 54, 31, 36.1879578, 143));
      test_key_points_.push_back(cv::KeyPoint(68, 87, 31, 157.532959, 108));
      //
      grid_real_assign_.push_back(std::make_pair(43, 6));
      grid_real_assign_.push_back(std::make_pair(43, 7));
      grid_real_assign_.push_back(std::make_pair(4, 11));
      // 0
      test_ransec_keypoints_init_homo_.push_back(cv::KeyPoint(258, 83, 31));
      test_ransec_keypoints_cur_homo_.push_back(cv::KeyPoint(257, 80, 31));
      test_normalized_keypoints_init_homo_.push_back(cv::KeyPoint(-1.03102267, -1.11353064, 31));
      test_normalized_keypoints_cur_homo_.push_back(cv::KeyPoint(-1.0584867, -1.14469051, 31));
      // 1
      test_ransec_keypoints_init_homo_.push_back(cv::KeyPoint(989, 31, 31));
      test_ransec_keypoints_cur_homo_.push_back(cv::KeyPoint(1011, 22, 31));
      test_normalized_keypoints_init_homo_.push_back(cv::KeyPoint(1.89086425, -1.83825541, 31));
      test_normalized_keypoints_cur_homo_.push_back(cv::KeyPoint(2.09878039, -1.9474349, 31));
      // 2
      test_ransec_keypoints_init_homo_.push_back(cv::KeyPoint(331, 58, 31));
      test_ransec_keypoints_cur_homo_.push_back(cv::KeyPoint(323, 56, 31));
      test_normalized_keypoints_init_homo_.push_back(cv::KeyPoint(-0.739233732, -1.46195602, 31));
      test_normalized_keypoints_cur_homo_.push_back(cv::KeyPoint(-0.744412958, -1.47686064, 31));
      // 3
      test_ransec_keypoints_init_homo_.push_back(cv::KeyPoint(543, 203, 31));
      test_ransec_keypoints_cur_homo_.push_back(cv::KeyPoint(544, 204, 31));
      test_normalized_keypoints_init_homo_.push_back(cv::KeyPoint(0.108153477, 0.558911204, 31));
      test_normalized_keypoints_cur_homo_.push_back(cv::KeyPoint(0.168880209, 0.571521521, 31));
      // 4
      test_ransec_keypoints_init_homo_.push_back(cv::KeyPoint(421, 87, 31));
      test_ransec_keypoints_cur_homo_.push_back(cv::KeyPoint(419, 87, 31));
      test_normalized_keypoints_init_homo_.push_back(cv::KeyPoint(-0.379493862, -1.05778253, 31));
      test_normalized_keypoints_cur_homo_.push_back(cv::KeyPoint(-0.347688347, -1.04780757, 31));
      // 5
      test_ransec_keypoints_init_homo_.push_back(cv::KeyPoint(420, 163, 31));
      test_ransec_keypoints_cur_homo_.push_back(cv::KeyPoint(419, 163, 31));
      test_normalized_keypoints_init_homo_.push_back(cv::KeyPoint(-0.38349098, 0.00143057713, 31));
      test_normalized_keypoints_cur_homo_.push_back(cv::KeyPoint(-0.347688347, 0.00406431407, 31));
      // 5
      test_ransec_keypoints_init_homo_.push_back(cv::KeyPoint(955, 339, 31));
      test_ransec_keypoints_cur_homo_.push_back(cv::KeyPoint(998, 356, 31));
      test_normalized_keypoints_init_homo_.push_back(cv::KeyPoint(1.75496256, 2.45434523, 31));
      test_normalized_keypoints_cur_homo_.push_back(cv::KeyPoint(2.04505706, 2.67526531, 31));
      // 6
      test_ransec_keypoints_init_homo_.push_back(cv::KeyPoint(370, 70, 31));
      test_ransec_keypoints_cur_homo_.push_back(cv::KeyPoint(363, 68, 31));
      test_normalized_keypoints_init_homo_.push_back(cv::KeyPoint(-0.583346426, -1.29471183, 31));
      test_normalized_keypoints_cur_homo_.push_back(cv::KeyPoint(-0.57911104, -1.31077564, 31));
      // 7
      test_ransec_keypoints_init_homo_.push_back(cv::KeyPoint(370, 70, 31));
      test_ransec_keypoints_cur_homo_.push_back(cv::KeyPoint(363, 68, 31));
      test_normalized_keypoints_init_homo_.push_back(cv::KeyPoint(-0.583346426, -1.29471183, 31));
      test_normalized_keypoints_cur_homo_.push_back(cv::KeyPoint(-0.57911104, -1.31077564, 31));
      H_ = (cv::Mat_<float>(3,3) <<
            -0.59589821, -0.0070154676, -0.027351039,
            -0.0023939069, -0.57682085, -0.0021076852,
            0.010585656, 0.0027909847, -0.55790335);
    }

    ak::Frame::Ptr ptr_frame_;
    cv::Mat test_image_;
    std::vector<cv::KeyPoint> test_key_points_;
    std::vector<std::pair<size_t, size_t>> grid_real_assign_;
    std::vector<cv::KeyPoint> test_normalized_keypoints_init_homo_;
    std::vector<cv::KeyPoint> test_normalized_keypoints_cur_homo_;
    std::vector<cv::KeyPoint> test_ransec_keypoints_init_homo_;
    std::vector<cv::KeyPoint> test_ransec_keypoints_cur_homo_;
    cv::Mat H_;
    std::string image_path_;
};

TEST_F(FrameTest, getGridPosition)
{
  ptr_frame_ = ak::Frame::CreateFrame(test_image_);
  size_t x, y;
  for(size_t idx=0; idx < test_key_points_.size(); ++idx)
  {
    ptr_frame_->getGridPosition(test_key_points_[idx], x, y);
    EXPECT_EQ(x, grid_real_assign_[idx].first);
    EXPECT_EQ(y, grid_real_assign_[idx].second);
  }
}

TEST_F(FrameTest, NormalizeKeyPoints)
{
  std::vector<cv::KeyPoint> keypoints_init;
  std::vector<cv::KeyPoint> keypoints_cur;
  std::vector<cv::KeyPoint> keypoints_init_norm;
  std::vector<cv::KeyPoint> keypoints_init_norm_compute;
  std::vector<cv::KeyPoint> keypoints_cur_norm;
  std::vector<cv::KeyPoint> keypoints_cur_norm_compute;
  std::ifstream file_kps1("../data/keys1.txt");
  std::ifstream file_kps2("../data/keys2.txt");
  int kps1_count;
  int kps2_count;
  std::vector<float> pre_mat;
  cv::Mat T1, T2;
  file_kps1 >> kps1_count;
  file_kps2 >> kps2_count;
  float x, y, p_size, angle, response, x_norm, y_norm, elem_mat;
  for(int i=0; i<kps1_count; ++i)
  {
    ReadPlainData(x, file_kps1);
    ReadPlainData(y, file_kps1);
    ReadPlainData(p_size, file_kps1);
    ReadPlainData(angle, file_kps1);
    ReadPlainData(response, file_kps1);
    keypoints_init.push_back(cv::KeyPoint(x, y, p_size, angle, response));
    ReadPlainData(x_norm, file_kps1);
    ReadPlainData(y_norm, file_kps1);
    ReadPlainData(p_size, file_kps1);
    keypoints_init_norm.push_back(cv::KeyPoint(x_norm, y_norm, p_size));
  }
  EXPECT_EQ(keypoints_init.size(), kps1_count);
  for(int i=0; i<9; ++i)
  {
    ReadPlainData(elem_mat, file_kps1);
    pre_mat.push_back(elem_mat);
  }
  cv::Mat temp1 = cv::Mat(pre_mat);
  T1 = temp1.reshape(3,3).clone();

  pre_mat.clear();
  for(int i=0; i<kps2_count; ++i)
  {
    ReadPlainData(x, file_kps2);
    ReadPlainData(y, file_kps2);
    ReadPlainData(p_size, file_kps2);
    ReadPlainData(angle, file_kps2);
    ReadPlainData(response, file_kps2);
    keypoints_cur.push_back(cv::KeyPoint(x, y, p_size, angle, response));

    ReadPlainData(x_norm, file_kps2);
    ReadPlainData(y_norm, file_kps2);
    ReadPlainData(p_size, file_kps2);
    keypoints_cur_norm.push_back(cv::KeyPoint(x_norm, y_norm, p_size));
  }
  EXPECT_EQ(keypoints_cur.size(), kps2_count);
  for(int i=0; i<9; ++i)
  {
    ReadPlainData(elem_mat, file_kps2);
    pre_mat.push_back(elem_mat);
  }
  file_kps1.close();
  file_kps2.close();
  cv::Mat temp2 = cv::Mat(pre_mat);
  T2 = temp2.reshape(3,3).clone();
  cv::Mat T1_compute, T2_compute;
  Frame::NormalizeKeyPoints(keypoints_init,
                            keypoints_init_norm_compute,
                            T1_compute);
  Frame::NormalizeKeyPoints(keypoints_cur,
                            keypoints_cur_norm_compute,
                            T2_compute);
  cv::Mat T1_diff = cv::Mat::zeros(3,3,CV_32F);
  //cv::absdiff(T1, T1_compute, T1_diff);
  cv::Mat T2_diff = cv::Mat::zeros(3,3,CV_32F);
  //cv::absdiff(T2, T2_compute, T2_diff);
  float error_t1_all{0.}, error_t2_all{0.};
  for(int i=0; i<3; ++i)
  {
    for(int j=0; j<3; ++j)
    {
      auto error_t1 = T1.at<float>(i,j) - T1_compute.at<float>(i,j);
      auto error_t2 = T2.at<float>(i,j) - T2_compute.at<float>(i,j);
      T1_diff.at<float>(i,j) = error_t1;
      T2_diff.at<float>(i,j) = error_t2;
      error_t1_all += std::abs(error_t1);
      error_t2_all += std::abs(error_t2);
    }
  }
  EXPECT_LT(error_t1_all, 1e-05);
  EXPECT_LT(error_t2_all, 1e-05);
  //std::cout << "T1_diff:\n" << T1_diff << std::endl;
  //std::cout << "T2_diff:\n" << T1_diff << std::endl;
  //std::cout << "Min float: " << std::numeric_limits<float>::min() << std::endl;
  //std::cout << "Error1 all: " << error_t1_all << std::endl;
  //std::cout << "Error2 all: " << error_t2_all << std::endl;
  //std::cout << "T1:\n" << T1 << std::endl;
  //std::cout << "T1_compute:\n" << T1_compute << std::endl;
  //std::cout << "T2:\n" << T2 << std::endl;
  //std::cout << "T2_compute:\n" << T2_compute << std::endl;
}

TEST_F(FrameTest, ComputeH21)
{
  std::vector<cv::KeyPoint> keypoints_init_norm;
  std::vector<cv::KeyPoint> keypoints_cur_norm;
  std::ifstream file_kps1("../data/keys1.txt");
  std::ifstream file_kps2("../data/keys2.txt");
  int kps1_count;
  int kps2_count;
  file_kps1 >> kps1_count;
  file_kps2 >> kps2_count;
  float x, y, p_size, angle, response, x_norm, y_norm, elem_mat;
  for(int i=0; i<kps1_count; ++i)
  {
    ReadPlainData(x, file_kps1);
    ReadPlainData(y, file_kps1);
    ReadPlainData(p_size, file_kps1);
    ReadPlainData(angle, file_kps1);
    ReadPlainData(response, file_kps1);
    //keypoints_init.push_back(cv::KeyPoint(x, y, p_size, angle, response));
    ReadPlainData(x_norm, file_kps1);
    ReadPlainData(y_norm, file_kps1);
    ReadPlainData(p_size, file_kps1);
    keypoints_init_norm.push_back(cv::KeyPoint(x_norm, y_norm, p_size));
  }

  for(int i=0; i<9; ++i)
  {
    ReadPlainData(elem_mat, file_kps1);
  }
  for(int i=0; i<kps2_count; ++i)
  {
    ReadPlainData(x, file_kps2);
    ReadPlainData(y, file_kps2);
    ReadPlainData(p_size, file_kps2);
    ReadPlainData(angle, file_kps2);
    ReadPlainData(response, file_kps2);

    ReadPlainData(x_norm, file_kps2);
    ReadPlainData(y_norm, file_kps2);
    ReadPlainData(p_size, file_kps2);
    keypoints_cur_norm.push_back(cv::KeyPoint(x_norm, y_norm, p_size));
  }
  for(int i=0; i<9; ++i)
  {
    ReadPlainData(elem_mat, file_kps2);
  }
  file_kps1.close();
  file_kps2.close();
  //===========
  std::ifstream file_ransec;
  file_ransec.open("../data/ransec_set.txt");
  //std::vector<cv::KeyPoint> kps_init_normalized;
  //std::vector<cv::KeyPoint> kps_cur_normalized;
  int iteration, num_ransec, iteration_all;
  //file_ransec >> iteration_all;
  ReadPlainData(iteration_all, file_ransec);
  //std::cout << "All_iteration: " << iteration_all << std::endl;

  std::vector<float> pre_mat;
  int init_idx, cur_idx;
  std::vector<cv::KeyPoint> kps_norm_ransec_init(8);
  std::vector<cv::KeyPoint> kps_norm_ransec_cur(8);
  for(int iter=0; iter<iteration_all; ++iter)
  {
    for(int j=0; j<8; ++j)
    {
      ReadPlainData(iteration, file_ransec);
      ReadPlainData(num_ransec, file_ransec);
      EXPECT_EQ(iteration, iter);
      EXPECT_EQ(num_ransec, j);
      ReadPlainData(init_idx, file_ransec);
      ReadPlainData(cur_idx, file_ransec);
      kps_norm_ransec_init[j] = keypoints_init_norm[init_idx];
      kps_norm_ransec_cur[j] = keypoints_cur_norm[cur_idx];
      //std::cout << iteration << " " << num_ransec << " "
      //          << init_idx << " " << cur_idx << " "
      //          << std::endl;
    }
    pre_mat.clear();
    for(int i=0; i<9; ++i)
    {
      ReadPlainData(elem_mat, file_ransec);
      pre_mat.push_back(elem_mat);
    }
    cv::Mat H21;
    cv::Mat temp1 = cv::Mat(pre_mat);
    H21 = temp1.reshape(3,3).clone();
    //std::cout << "HERE" << std::endl;
    cv::Mat H21_compute = Frame::ComputeH21(kps_norm_ransec_init, kps_norm_ransec_cur);
    //std::cout << "H21: \n" << H21 << std::endl;
    //std::cout << "H21_compute: \n" << H21_compute << std::endl;
    cv::Mat H21_diff = cv::Mat::zeros(3, 3, CV_32F);
    float error_h21_all{0.};
    for(int i=0; i<3; ++i)
    {
      for(int j=0; j<3; ++j)
      {
        auto error_h21 = H21.at<float>(i,j) - H21_compute.at<float>(i,j);
        H21_diff.at<float>(i,j) = error_h21;
        error_h21_all += std::abs(error_h21);
      }
    }
    //std::cout << "H21_diff: \n" << H21_diff << std::endl;
    //std::cout << "H21_error_all: " << error_h21_all << std::endl;
    EXPECT_LT(error_h21_all, 1e-04);
  }
  //cv::Mat H21 = Frame::ComputeH21(kps_init_normalized, kps_cur_normalized);
  //std::cout << "ComputeH21:\n" << H21 << std::endl;
}

}
