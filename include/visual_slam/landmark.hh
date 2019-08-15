#ifndef __LANDMARK_HH__
#define __LANDMARK_HH__
/**-----------------------------------------------
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: landmark.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-03-02 23:57:11
  * @last_modified_date: 2019-04-25 12:28:00
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <memory>
#include <opencv2/opencv.hpp>
#include <visual_slam/utils/type.hh>
//#include <visual_slam/ORBmatcher.hh>

// Declaration
namespace ak
{
  class Frame;
  class VisualOdometry;
  class Landmark : public std::enable_shared_from_this<Landmark>
  {
    public:
      //using Count_t = unsigned int;
      using Count_t = size_t;
      using Ptr = std::shared_ptr<Landmark>;
      friend class VisualOdometry;
      friend class Map;
      friend class ORBmatcher;
      friend class Optimizer;
      struct Property{
        float distance_;
        float level_factor_;
        int level_at_;
        int num_level_;
      };

    public:
      Landmark() = delete;
      Landmark(ID_t id, float x, float y, float z);
      Landmark(ID_t id, const cv::Point3f& position);
      Landmark(ID_t id,
               std::shared_ptr<Frame>& ptr_last_frame,
               std::shared_ptr<Frame>& ptr_current_frame,
               const cv::Point3f& position);
      ~Landmark() = default;

    public:
      static Landmark::Ptr CreateLandmark(std::shared_ptr<Frame>& ptr_last_frame,
                                          std::shared_ptr<Frame>& ptr_current_frame,
                                          const cv::Point3f& position);
      inline std::pair<std::shared_ptr<Frame>, std::shared_ptr<Frame>>& getFirstFramePair()
      {
        return first_frame_pair_;
      }
      inline Count_t getObservationTimes()
      {
        return num_observation_times_;
      }
      inline const std::unordered_map<std::shared_ptr<Frame>, size_t>& getObservers()
      {
        return observers_;
      }
      inline ID_t getID()
      {
        return id_;
      }
      inline const cv::Point3f& getPosition()
      {
        return position_at_world_;
      }
      inline const cv::Mat getDescriptor()
      {
        return mean_descriptor_.clone();
      }
      inline void setPosition(const cv::Point3f& position)
      {
        position_at_world_ = position;
      }
      inline void setGBAPose(const cv::Point3f& position)
      {
        position_gba_ = position;
      }
      inline void setNumGBA(unsigned int num_ba_global)
      {
        num_ba_global_ = num_ba_global;
      }
      inline bool isAvailable()
      {
        return is_available_;
      }
      inline void setUnavailable(bool avaliable=false)
      {
        is_available_ = avaliable;
      }
      inline void setAvailable(bool avaliable=true)
      {
        is_available_ = avaliable;
      }
      //inline void setInMap(bool flag)
      //{
      //  flag_include_in_map_ = flag;
      //}
      //inline bool isInMap()
      //{
      //  return flag_include_in_map_;
      //}
      //inline bool isDrop()
      //{
      //  return is_drop_;
      //}
      void increaseFound(Count_t num_found);
      void increaseVisible(Count_t num_visible);
      float calFoundRatio()
      {
        assert(num_visible_!=0);
        return static_cast<float>(num_found_)/num_visible_;
      }

      float getMaxDistanceInvariance()
      { return max_distance_; }
      float getMinDistanceInvariance()
      { return min_distance_; }
      cv::Mat getNormPos()
      { return cv::Mat(position_normal_at_camera_);}

      int predictScale(const float& current_dist);
      void replace(const Landmark::Ptr& ptr_new_landmark);
      void dropObserver(const std::shared_ptr<Frame>& ptr_frame)
      {
        bool bBad=false;
        {
//          unique_lock<mutex> lock(mMutexFeatures);
          if(observers_.count(ptr_frame))
          {
            int idx = observers_[ptr_frame];
            num_observation_times_--;
            observers_.erase(ptr_frame);

            if(first_frame_pair_.first==ptr_frame)
              first_frame_pair_.first=observers_.begin()->first;

            // If only 2 observations or less, discard point
            if(num_observation_times_<=2)
              bBad=true;
          }
        }

        if(bBad)
          setUnavailable();
      }

      void updateLandmark();

      static ID_t factory_id;


      // Check function
      bool isInFrame(const std::shared_ptr<Frame>& ptr_frame);

    protected:
      void addObserver(const std::shared_ptr<Frame>& ptr_observer_frame,
                       size_t kp_idx);
      void updateMeanDescriptor();
      void updateNormalRepresentation();


    private:
      ID_t id_;
      ID_t fuse_for_id_;
      ID_t localBA_for_id_;
      cv::Point3f position_at_world_;
      cv::Point3f position_gba_;
      cv::Point3f position_normal_at_camera_;
      Property property_;
      Count_t num_observation_times_;
      Count_t num_visible_;
      Count_t num_found_;
      //bool is_good_{true};
      //bool flag_include_in_map_{true};
      unsigned int num_ba_global_{0};
      bool is_available_ = true;
      bool is_fused_ = false;
      //bool is_drop_{false};

      std::pair<std::shared_ptr<Frame>, std::shared_ptr<Frame>> first_frame_pair_;
      std::unordered_map<std::shared_ptr<Frame>, size_t> observers_;    /*! Observation FrameID and correspond keypointID*/
      std::vector<cv::Mat> descriptors_all_observed_; /*! Description for all observations */
      cv::Mat mean_descriptor_;
      cv::Mat mean_orientation_;
      cv::Mat obs_normal_;
      float max_distance_;
      float min_distance_;

      Landmark::Ptr ptr_replace_landmark_;
  };
}
#endif // __LANDMARK_HH__
