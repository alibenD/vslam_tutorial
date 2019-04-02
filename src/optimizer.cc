/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: optimizer.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-03-31 18:50:05
  * @last_modified_date: 2019-04-01 14:08:56
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <visual_slam/optimizer.hh>
#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <memory>

// For convert ID(unsigned long) to int.
// This feature with G2O is not good, ID should be unsigned
#include <climits>


//CODE
namespace ak
{
  void Optimizer::bundleAdjustment(const std::vector<Frame::Ptr>& keyframes,
                                   const std::vector<Landmark::Ptr>& landmarks,
                                   const cv::Mat& K,
                                   float& sigma2,
                                   int iteration,
                                   bool* ptr_stop_flag,
                                   const unsigned long num_loop_keyframe,
                                   const bool flag_robust
                                   )
  {
    // Upgrade g2o with smart point. With mac, the memory management is strict.
    // So if to use g2o without smart point, we have to release heap(object with new)
    // manually.
    AK_DLOG_WARNING << "Global optimizing.";
    g2o::SparseOptimizer graph;
    //g2o::BlockSolver_6_3::LinearSolverType* ptr_linear_solver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    //g2o::BlockSolver_6_3* ptr_block_solver = new g2o::BlockSolver_6_3(ptr_linear_solver);
    //g2o::OptimizationAlgorithmLevenberg* ptr_solver = new g2o::OptimizationAlgorithmLevenberg(ptr_block_solver);
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> ptr_linear_solver = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
    g2o::OptimizationAlgorithmLevenberg* ptr_solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(ptr_linear_solver)));

    Frame::ID_t max_id;

    graph.setAlgorithm(ptr_solver);
    if(ptr_stop_flag != nullptr)
    {
      graph.setForceStopFlag(ptr_stop_flag);
    }

    std::vector<g2o::VertexSE3Expmap*> release_frame_list;
    std::vector<g2o::VertexSBAPointXYZ*> release_landmark_list;
    for(const auto& ptr_keyframe:keyframes)
    {
      if(ptr_keyframe->isGood() == false || ptr_keyframe == nullptr)
      {
        AK_DLOG_WARNING << "A bad keyframe ID: " << ptr_keyframe->getID();
        continue;
      }
      auto vertex_frame = new g2o::VertexSE3Expmap();
      //release_frame_list.push_back(vertex_frame);

      // Tmp convert TF to SEQuat
      Eigen::Matrix<double, 3, 3> R;
      auto TF = ptr_keyframe->getPose();
      R << TF.at<float>(0,0) , TF.at<float>(0,1) , TF.at<float>(0,2),
           TF.at<float>(1,0) , TF.at<float>(1,1) , TF.at<float>(1,2),
           TF.at<float>(2,0) , TF.at<float>(2,1) , TF.at<float>(2,2);
      Eigen::Matrix<double, 3, 1> t(TF.at<float>(0,3), TF.at<float>(1,3), TF.at<float>(2,3));
      vertex_frame->setEstimate(g2o::SE3Quat(R, t));
      int id = ptr_keyframe->getID() & INT_MAX;
      vertex_frame->setId(id);
      // TODO: (aliben.develop@gmail.com)
      // Set the first keyframe as fixed but not ID is equal to 0.
      AK_DLOG_WARNING << "For now, set 0-th frame as fixed";
      vertex_frame->setFixed(ptr_keyframe->getID() == 0);
      graph.addVertex(vertex_frame);
      if(ptr_keyframe->getID() > max_id)
      {
        max_id = ptr_keyframe->getID();
      }
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    for(const auto& ptr_landmark:landmarks)
    {
      if(ptr_landmark->isGood() == false || ptr_landmark == nullptr)
      {
        continue;
      }
      auto vertex_landmark = new g2o::VertexSBAPointXYZ();
      //release_landmark_list.push_back(vertex_landmark);
      Eigen::Matrix<double, 3, 1> point_location;
      auto location_at_world = ptr_landmark->getPosition();
      point_location << location_at_world.x, location_at_world.y, location_at_world.z;
      vertex_landmark->setEstimate(point_location);
      const int id = (int)(max_id & INT_MAX) + (int)(ptr_landmark->getID() & INT_MAX) + 1;
      vertex_landmark->setId(id);
      vertex_landmark->setMarginalized(true);
      graph.addVertex(vertex_landmark);

      const auto observers = ptr_landmark->getObservers();

      unsigned long count_edge = 0;
      for(const auto& observer_pair:observers)
      {
        auto ptr_observer = observer_pair.first;
        const auto& keypoint = ptr_observer->getKeyPoints()[observer_pair.second];
        if(ptr_observer == nullptr || ptr_observer->isGood() == false)
        {
          continue;
        }
        ++count_edge;

        Eigen::Matrix<double, 2, 1> measurement;
        measurement << keypoint.pt.x, keypoint.pt.y;
        auto edge = new g2o::EdgeSE3ProjectXYZ();

        edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(graph.vertex(id)));
        edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(graph.vertex(ptr_observer->getID() & INT_MAX)));
        edge->setMeasurement(measurement);
        const float inv_sigma2 = sigma2==0. ? 0 : std::pow(1/sigma2, keypoint.octave);
        edge->setInformation(Eigen::Matrix2d::Identity() * inv_sigma2);
        if(flag_robust == true)
        {
          g2o::RobustKernelHuber* robust_kernel = new g2o::RobustKernelHuber();
          edge->setRobustKernel(robust_kernel);
          robust_kernel->setDelta(thHuber2D);
        }
        edge->fx = K.at<float>(0,0);
        edge->fy = K.at<float>(1,1);
        edge->cx = K.at<float>(0,2);
        edge->cy = K.at<float>(1,2);
        graph.addEdge(edge);
      }
      if(count_edge == 0)
      {
        graph.removeVertex(vertex_landmark);
        ptr_landmark->setInMap(false);
      }
      else
      {
        ptr_landmark->setInMap(true);
      }
    }
    //graph.setVerbose(true);
    graph.initializeOptimization();
    graph.optimize(iteration);

    // Update optimization result
    for(auto& ptr_keyframe:keyframes)
    {
      if(ptr_keyframe->isGood() == false || ptr_keyframe == nullptr)
      {
        continue;
      }
      g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(graph.vertex(ptr_keyframe->getID() & INT_MAX));
      g2o::SE3Quat SE3quat = vSE3->estimate();
      Eigen::Matrix<double,4,4> eigMat = SE3quat.to_homogeneous_matrix();
      cv::Mat cvMat(4,4,CV_32F);
      for(int i=0; i<4; ++i)
      {
        for(int j=0; j<4; ++j)
        {
          cvMat.at<float>(i,j)=eigMat(i,j);
        }
      }
      if(num_loop_keyframe == 0)
      {
        ptr_keyframe->setTF(cvMat);
      }
      else
      {
        // TODO: (aliben.develop@gmail.com)
        // Restore Global BA optimizated pose
      }
    }

    for(auto& ptr_landmark:landmarks)
    {
      if(ptr_landmark->isInMap() == false || ptr_landmark == nullptr)
      {
        continue;
      }
      const int id = (int)(max_id & INT_MAX) + (int)(ptr_landmark->getID() & INT_MAX) + 1;
      g2o::VertexSBAPointXYZ* vertex_landmark= static_cast<g2o::VertexSBAPointXYZ*>(graph.vertex(id));
      Eigen::Matrix<double, 3, 1> point = vertex_landmark->estimate();
      ptr_landmark->setPosition(cv::Point3f(point(0), point(1), point(2)));
      ptr_landmark->updateLandmark();
    }
  }
}
