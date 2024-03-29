#ifndef __OPTIMIZER_HH__
#define __OPTIMIZER_HH__
/**-----------------------------------------------
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: optimizer.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-03-31 18:50:05
  * @last_modified_date: 2019-04-12 08:50:30
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <visual_slam/logger_advanced.hh>
#include <visual_slam/landmark.hh>
#include <visual_slam/frame.hh>
#include <visual_slam/map.hh>
#include <g2o/types/sba/types_six_dof_expmap.h>

// Declaration
namespace ak
{
  class Optimizer;

  class Optimizer
  {
    public:
      using Ptr = std::shared_ptr<Optimizer>;
      Optimizer() = default;
      ~Optimizer() = default;

    public:
      void bundleAdjustment(const std::vector<Frame::Ptr>& keyframes,
                            const std::vector<Landmark::Ptr>& landmarks,
                            const cv::Mat& K,
                            float& sigma2,
                            int iteration = 5,
                            bool* ptr_stop_flag = nullptr,
                            const unsigned long num_loop_keyframe = 0,
                            const bool flag_robust = true
                            );
      void globalBundleAdjustment(const Map::Ptr& ptr_map,
                                  const cv::Mat& K,
                                  float& sigma2,
                                  int iteration = 5,
                                  bool* ptr_stop_flag = nullptr,
                                  const unsigned long num_loop_keyframe = 0,
                                  const bool flag_robust = true);

      int PoseOptimization(const Frame::Ptr& ptr_frame,
                           const cv::Mat& K,
                           float& sigma2
                           );

      static void localBundleAdjustment(const Frame::Ptr& ptr_local_frame,
                                        const Map::Ptr& map,
                                        float& sigma2,
                                        const cv::Mat& K,
                                        bool* ptr_stop_flag=nullptr,
                                        const bool flag_robust = true
                                          );
  };
}
#endif // __OPTIMIZER_HH__
