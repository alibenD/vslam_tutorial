/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: optimizer.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-03-31 18:50:05
  * @last_modified_date: 2019-04-24 15:38:08
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

    ID_t max_id;

    graph.setAlgorithm(ptr_solver);
    if(ptr_stop_flag != nullptr)
    {
      graph.setForceStopFlag(ptr_stop_flag);
    }

    std::vector<g2o::VertexSE3Expmap*> release_frame_list;
    std::vector<g2o::VertexSBAPointXYZ*> release_landmark_list;
    int count_keyframe = 0;
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
      auto TF = ptr_keyframe->getTF();
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
      vertex_frame->setFixed(count_keyframe == 0);
      count_keyframe++;
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
      if(ptr_landmark->isAvailable() == false || ptr_landmark == nullptr)
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
        ptr_landmark->setUnavailable();
      }
      else
      {
        ptr_landmark->setAvailable();
      }
    }
    graph.setVerbose(false);
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
        // If no loop optimize before, set the global result as TruePose
        ptr_keyframe->setTF(cvMat);
      }
      else
      {
        // TODO: (aliben.develop@gmail.com)
        // Restore Global BA optimizated pose
        ptr_keyframe->setGBATF(cvMat);
        ptr_keyframe->setNumGBA(num_loop_keyframe);
      }
    }

    for(auto& ptr_landmark:landmarks)
    {
      if(ptr_landmark->isAvailable() == false || ptr_landmark == nullptr)
      {
        continue;
      }

      const int id = (int)(max_id & INT_MAX) + (int)(ptr_landmark->getID() & INT_MAX) + 1;
      g2o::VertexSBAPointXYZ* vertex_landmark= static_cast<g2o::VertexSBAPointXYZ*>(graph.vertex(id));
      Eigen::Matrix<double, 3, 1> point = vertex_landmark->estimate();

      // Check loop
      if(num_loop_keyframe == 0)
      {
        ptr_landmark->setPosition(cv::Point3f(point(0), point(1), point(2)));
        ptr_landmark->updateLandmark();
      }
      else
      {
        ptr_landmark->setGBAPose(cv::Point3f(point(0), point(1), point(2)));
        ptr_landmark->setNumGBA(num_loop_keyframe);
      }
    }
  }

  void Optimizer::globalBundleAdjustment(const ak::Map::Ptr& ptr_map,
                                         const cv::Mat& K,
                                         float& sigma2,
                                         int iteration,
                                         bool* ptr_stop_flag,
                                         const unsigned long num_loop_keyframe,
                                         const bool flag_robust)
  {
    auto keyframes = ptr_map->getKeyframes();
    auto landmarks = ptr_map->getLandmarks();
    bundleAdjustment(keyframes, landmarks, K, sigma2, iteration, ptr_stop_flag, num_loop_keyframe, flag_robust);
  }

  int Optimizer::PoseOptimization(const Frame::Ptr& ptr_frame,
                                  const cv::Mat& K,
                                  float& sigma2
                                  )
  {
    AK_DLOG_WARNING << "Pose Optimization";
    g2o::SparseOptimizer graph;
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> ptr_linear_solver = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
    g2o::OptimizationAlgorithmLevenberg* ptr_solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(ptr_linear_solver)));

    graph.setAlgorithm(ptr_solver);

    //int num_initial_correspondences = 0;

    auto vertex_frame = new g2o::VertexSE3Expmap();

    // TODO: (aliben.develop@gmail.com)
    // The convert will be packaged into an obj.
    Eigen::Matrix<double, 3, 3> R;
    auto TF = ptr_frame->getTF();
    R << TF.at<float>(0,0) , TF.at<float>(0,1) , TF.at<float>(0,2),
      TF.at<float>(1,0) , TF.at<float>(1,1) , TF.at<float>(1,2),
      TF.at<float>(2,0) , TF.at<float>(2,1) , TF.at<float>(2,2);
    Eigen::Matrix<double, 3, 1> t(TF.at<float>(0,3), TF.at<float>(1,3), TF.at<float>(2,3));
    vertex_frame->setEstimate(g2o::SE3Quat(R, t));
    vertex_frame->setId(0);
    vertex_frame->setFixed(false);
    graph.addVertex(vertex_frame);

    auto landmarks = ptr_frame->getLandmarks();
    auto size_landmark = landmarks.size();
    std::vector<g2o::EdgeSE3ProjectXYZOnlyPose*> edge_vector;
    std::vector<bool> outlier_vector;
    edge_vector.reserve(size_landmark);
    outlier_vector.reserve(size_landmark);

    const float deltaMono = sqrt(5.991);
//    const float thHuber3D = sqrt(7.815);
    int debug_index = 0;
    for(const auto& landmark_pair : landmarks)
    {
      //++num_initial_correspondences;
      const auto& keypoint = ptr_frame->getKeyPoints()[landmark_pair.first];
      auto ptr_landmark = landmark_pair.second;
      assert(ptr_landmark!=nullptr);
      Eigen::Matrix<double, 2, 1> measurement;
      measurement << keypoint.pt.x, keypoint.pt.y;

      g2o::EdgeSE3ProjectXYZOnlyPose* edge = new g2o::EdgeSE3ProjectXYZOnlyPose();
      edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(graph.vertex(0)));
      edge->setMeasurement(measurement);
      const float inv_sigma2 = sigma2==0. ? 0 : std::pow(1/sigma2, keypoint.octave);
      edge->setInformation(Eigen::Matrix2d::Identity() * inv_sigma2);

      g2o::RobustKernelHuber* robust_kernel = new g2o::RobustKernelHuber;
      edge->setRobustKernel(robust_kernel);
      robust_kernel->setDelta(deltaMono);

      edge->fx = K.at<float>(0,0);
      edge->fy = K.at<float>(1,1);
      edge->cx = K.at<float>(0,2);
      edge->cy = K.at<float>(1,2);
      assert(ptr_landmark!=nullptr);

      //      AK_DLOG_INFO << "Debug Index: " << debug_index;
//      if(ptr_landmark == nullptr)
//      {
//        AK_DLOG_FATAL << "ptr_landmark is nullptr";
//      }
      //AK_DLOG_INFO << "Landmark ID: " << ptr_landmark->getID();
      auto point_location = ptr_landmark->getPosition();
      edge->Xw[0] = point_location.x;
      edge->Xw[1] = point_location.y;
      edge->Xw[2] = point_location.z;

      graph.addEdge(edge);
      edge_vector.emplace_back(edge);
      outlier_vector.push_back(false);
      ++debug_index;
    }

    if(size_landmark<3)
    {
      return 0;
    }

    const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
    const int iteration[4] = {10, 10, 10, 10};
    int num_bad_edge = 0;
    
    for(int i=0; i<4; ++i)
    {
      Eigen::Matrix<double, 3, 3> R_iter;
      auto TF_iter = ptr_frame->getTF();
      R_iter << TF_iter.at<float>(0,0) , TF_iter.at<float>(0,1) , TF_iter.at<float>(0,2),
        TF_iter.at<float>(1,0) , TF_iter.at<float>(1,1) , TF_iter.at<float>(1,2),
        TF_iter.at<float>(2,0) , TF_iter.at<float>(2,1) , TF_iter.at<float>(2,2);
      Eigen::Matrix<double, 3, 1> t_iter(TF_iter.at<float>(0,3), TF_iter.at<float>(1,3), TF_iter.at<float>(2,3));
      vertex_frame->setEstimate(g2o::SE3Quat(R_iter, t_iter));

      graph.initializeOptimization(0);
      graph.optimize(iteration[i]);

      num_bad_edge = 0;

      size_t index = 0;
      for(auto& edge:edge_vector)
      {
        if(outlier_vector[index] == false)
        {
          edge->computeError();
        }
        const float chi2 = edge->chi2();
        if(chi2 > chi2Mono[i])
        {
          outlier_vector[index] = true;
          edge->setLevel(1);
          ++num_bad_edge;
        }
        else
        {
          outlier_vector[index] = false;
          edge->setLevel(0);
        }

        if(i == 2)
        {
          edge->setRobustKernel(0);
        }
        ++index;
      }

      if(graph.edges().size() < 10)
      {
        break;
      }

    }

    //g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    //g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    //cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    //pFrame->SetPose(pose);
    //return nInitialCorrespondences-nBad;

    auto vertex_recovery = static_cast<g2o::VertexSE3Expmap*>(graph.vertex(0));
    g2o::SE3Quat SE3quat_recovery = vertex_recovery->estimate();
    Eigen::Matrix<double,4,4> eigMat = SE3quat_recovery.to_homogeneous_matrix();
    cv::Mat cvMat(4,4,CV_32F);
    for(int i=0; i<4; ++i)
    {
      for(int j=0; j<4; ++j)
      {
        cvMat.at<float>(i,j)=eigMat(i,j);
      }
    }
    ptr_frame->setTF(cvMat);
    return size_landmark - num_bad_edge;
  }

  void Optimizer::localBundleAdjustment(const Frame::Ptr& ptr_local_frame,
                                        const Map::Ptr& map,
                                        float& sigma2,
                                        const cv::Mat& K,
                                        bool* ptr_stop_flag,
                                        const bool flag_robust)
  {
    std::list<Frame::Ptr> local_keyframes;

    local_keyframes.push_back(ptr_local_frame);
    ptr_local_frame->localBA_for_id_ = ptr_local_frame->id_;

    const auto neighbor_frames = ptr_local_frame->getAllBestCovisibleFrame();
    for(auto ptr_neigh_frame:neighbor_frames)
    {
      ptr_neigh_frame->localBA_for_id_ = ptr_local_frame->id_;
      if(ptr_neigh_frame->isGood())
      {
        local_keyframes.push_back(ptr_neigh_frame);
      }
    }

    std::list<Landmark::Ptr> local_landmarks;
    for(auto ptr_localmap_frame:local_keyframes)
    {
      auto landmarks_vec = ptr_localmap_frame->getLandmarksVector();
      for(auto ptr_landmark:landmarks_vec)
      {
        if(ptr_landmark==nullptr||ptr_landmark->isAvailable()==false)
        {
          continue;
        }
        local_landmarks.push_back(ptr_landmark);
        ptr_landmark->localBA_for_id_ = ptr_local_frame->id_;
      }
    }

    // Fixed KeyFrames.
    std::list<Frame::Ptr> fixed_frames;
    for(auto ptr_landmark:local_landmarks)
    {
      auto observers = ptr_landmark->getObservers();
      for(auto obs_pair:observers)
      {
        auto ptr_obs_frame = obs_pair.first;
        if(ptr_obs_frame->localBA_for_id_!=ptr_local_frame->id_ && ptr_obs_frame->fixedBA_for_id_!=ptr_local_frame->id_)
        {
          ptr_obs_frame->fixedBA_for_id_ = ptr_local_frame->id_;
          if(ptr_obs_frame->isGood()==true)
          {
            fixed_frames.push_front(ptr_obs_frame);
          }
        }
      }
    }

    g2o::SparseOptimizer graph;
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> ptr_linear_solver = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
    g2o::OptimizationAlgorithmLevenberg* ptr_solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolver_6_3>(std::move(ptr_linear_solver)));

    ID_t max_id=0;
    graph.setAlgorithm(ptr_solver);

    if(ptr_stop_flag != nullptr)
    {
      graph.setForceStopFlag(ptr_stop_flag);
    }

    int count_keyframe = 0;
    // Set vertex for local frame
    for(auto ptr_keyframe:local_keyframes)
    {
      auto vertex_frame = new g2o::VertexSE3Expmap();

      Eigen::Matrix<double, 3, 3> R;
      auto TF = ptr_keyframe->getTF();
      R << TF.at<float>(0,0) , TF.at<float>(0,1) , TF.at<float>(0,2),
        TF.at<float>(1,0) , TF.at<float>(1,1) , TF.at<float>(1,2),
        TF.at<float>(2,0) , TF.at<float>(2,1) , TF.at<float>(2,2);
      Eigen::Matrix<double, 3, 1> t(TF.at<float>(0,3), TF.at<float>(1,3), TF.at<float>(2,3));
      vertex_frame->setEstimate(g2o::SE3Quat(R, t));
      int id = ptr_keyframe->getID() & INT_MAX;
      vertex_frame->setId(id);

      vertex_frame->setFixed(count_keyframe == 0);
      count_keyframe++;
      graph.addVertex(vertex_frame);
      if(ptr_keyframe->getID() > max_id)
      {
        max_id = ptr_keyframe->getID();
      }
    }

    // Set vertex for fixed frame
    for(auto ptr_fixed_keyframe:fixed_frames)
    {
      auto vertex_frame = new g2o::VertexSE3Expmap();

      Eigen::Matrix<double, 3, 3> R;
      auto TF = ptr_fixed_keyframe->getTF();
      R << TF.at<float>(0,0) , TF.at<float>(0,1) , TF.at<float>(0,2),
        TF.at<float>(1,0) , TF.at<float>(1,1) , TF.at<float>(1,2),
        TF.at<float>(2,0) , TF.at<float>(2,1) , TF.at<float>(2,2);
      Eigen::Matrix<double, 3, 1> t(TF.at<float>(0,3), TF.at<float>(1,3), TF.at<float>(2,3));
      vertex_frame->setEstimate(g2o::SE3Quat(R, t));
      int id = ptr_fixed_keyframe->getID() & INT_MAX;
      vertex_frame->setId(id);

      vertex_frame->setFixed(true);
      count_keyframe++;
      graph.addVertex(vertex_frame);
      if(ptr_fixed_keyframe->getID() > max_id)
      {
        max_id = ptr_fixed_keyframe->getID();
      }
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    const int nExpectedSize = (local_keyframes.size()+fixed_frames.size())*local_landmarks.size();
    std::vector<Frame::Ptr> vertex_frame_vec;
    std::vector<Landmark::Ptr> vertex_landmark_vec;
    std::vector<g2o::EdgeSE3ProjectXYZ*> edges_vec;

    vertex_frame_vec.reserve(nExpectedSize);
    vertex_landmark_vec.reserve(nExpectedSize);
    edges_vec.reserve(nExpectedSize);

    for(auto ptr_landmark:local_landmarks)
    {
      if(ptr_landmark->isAvailable()==false || ptr_landmark == nullptr)
      {
        continue;
      }
      auto vertex_landmark = new g2o::VertexSBAPointXYZ();
      Eigen::Matrix<double, 3, 1> point_location;
      auto location_at_world = ptr_landmark->getPosition();
      point_location << location_at_world.x, location_at_world.y, location_at_world.z;
      vertex_landmark->setEstimate(point_location);
      const int id = (int)(max_id & INT_MAX) + (int)(ptr_landmark->getID() & INT_MAX) + 1;
      vertex_landmark->setId(id);
      vertex_landmark->setMarginalized(true);
      graph.addVertex(vertex_landmark);

      auto observers = ptr_landmark->getObservers();
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

        edges_vec.push_back(edge);
        vertex_frame_vec.push_back(ptr_observer);
        vertex_landmark_vec.push_back(ptr_landmark);
      }
    }

    graph.setVerbose(false);
    graph.initializeOptimization();
    graph.optimize(5);

    bool flag_do_more = true;
    if(ptr_stop_flag != nullptr)
    {
      if(*ptr_stop_flag == false)
      {
        flag_do_more = false;
      }
    }

    if(flag_do_more == true)
    {
      // check inliers
      auto size_edges = edges_vec.size();
      for(size_t idx=0; idx<size_edges; ++idx)
      {
        auto edge = edges_vec[idx];
        auto ptr_landmark = vertex_landmark_vec[idx];
        if(ptr_landmark->isAvailable()==false)
        {
          continue;
        }
        if(edge->chi2()>5.991||edge->isDepthPositive())
        {
          edge->setLevel(1);
        }
        edge->setRobustKernel(0);
      }
      graph.initializeOptimization();
      graph.optimize(10);
    }

    std::vector<std::pair<Frame::Ptr,Landmark::Ptr>> drop_frame_landmark_pair;
    drop_frame_landmark_pair.reserve(edges_vec.size());

    // Check inlier observations
    for(size_t i=0, iend=edges_vec.size(); i<iend;i++)
    {
      g2o::EdgeSE3ProjectXYZ* edge = edges_vec[i];
      auto pMP = vertex_landmark_vec[i];

      if(pMP->isAvailable() == false)
      {
        continue;
      }

      if(edge->chi2()>5.991 || !edge->isDepthPositive())
      {
        Frame::Ptr pKFi = vertex_frame_vec[i];
        drop_frame_landmark_pair.push_back(std::make_pair(pKFi,pMP));
      }
    }

    if(!drop_frame_landmark_pair.empty())
    {
      for(size_t i=0;i<drop_frame_landmark_pair.size();i++)
      {
        auto pKFi = drop_frame_landmark_pair[i].first;
        auto pMPi = drop_frame_landmark_pair[i].second;
        pKFi->dropLandmark(pMPi);
        pMPi->dropObserver(pKFi);
      }
    }

    // Recover optimized data

    //Keyframes
    for(auto ptr_keyframe:vertex_frame_vec)
    {
      g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(graph.vertex(ptr_keyframe->id_& INT_MAX));
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
      ptr_keyframe->setTF(cvMat);
    }

    //Points
    for(auto ptr_landmark:vertex_landmark_vec)
    {
      const int id = (int)(max_id & INT_MAX) + (int)(ptr_landmark->getID() & INT_MAX) + 1;
      g2o::VertexSBAPointXYZ* vertex_landmark= static_cast<g2o::VertexSBAPointXYZ*>(graph.vertex(id));
      Eigen::Matrix<double, 3, 1> point = vertex_landmark->estimate();

      ptr_landmark->setPosition(cv::Point3f(point(0),point(1),point(2)));
      ptr_landmark->updateNormalRepresentation();
    }
  }
}
