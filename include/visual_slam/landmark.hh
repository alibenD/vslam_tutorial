#ifndef __LANDMARK_HH__
#define __LANDMARK_HH__
/**-----------------------------------------------
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: landmark.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-03-02 23:57:11
  * @last_modified_date: 2019-03-12 14:59:44
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <memory>
#include <opencv2/opencv.hpp>
//#include <visual_slam/ORBmatcher.hh>

// Declaration
namespace ak
{
  class Frame;
  class VisualOdometry;
  class Landmark : public std::enable_shared_from_this<Landmark>
  {
    public:
      using ID_t = unsigned long;
      using Count_t = unsigned int;
      using Ptr = std::shared_ptr<Landmark>;
      friend class VisualOdometry;
      friend class Map;
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
      inline void setPosition(const cv::Point3f& position)
      {
        position_at_world_ = position;
      }
      inline bool isGood()
      {
        return is_good_;
      }
      inline void setInMap(bool flag)
      {
        flag_include_in_map_ = flag;
      }
      inline bool isInMap()
      {
        return flag_include_in_map_;
      }
//      inline bool isDrop()
//      {
//        return is_drop_;
//      }
      void updateLandmark();

      static ID_t factory_id;

    protected:
      void addObserver(const std::shared_ptr<Frame>& ptr_observer_frame,
                       size_t kp_idx);
      void updateMeanDescriptor();
      void updateNormalRepresentation();


    private:
      ID_t id_;
      cv::Point3f position_at_world_;
      cv::Point3f position_normal_at_camera_;
      Property property_;
      Count_t num_observation_times_;
      bool is_good_{true};
      bool flag_include_in_map_{true};
//      bool is_drop_{false};

      std::pair<std::shared_ptr<Frame>, std::shared_ptr<Frame>> first_frame_pair_;
      std::unordered_map<std::shared_ptr<Frame>, size_t> observers_;    /*! Observation FrameID and correspond keypointID*/
      std::vector<cv::Mat> descriptors_all_observed_; /*! Description for all observations */
      cv::Mat mean_descriptor_;
      cv::Mat mean_orientation_;
      cv::Mat obs_normal_;
      float max_distance_;
      float min_distance_;
  };
}
#endif // __LANDMARK_HH__
