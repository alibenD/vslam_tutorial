#ifndef __MAP_HH__
#define __MAP_HH__
/**-----------------------------------------------
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: map.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-03-03 11:06:28
  * @last_modified_date: 2019-04-25 12:17:23
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <visual_slam/frame.hh>
#include <visual_slam/landmark.hh>
#include <visual_slam/utils/type.hh>
#include <visual_slam/ORBmatcher.hh>

// Declaration
namespace ak
{
  class Map;
  class Map : public std::enable_shared_from_this<Map>
  {
    public:
      friend class VisualOdometry;
      using Ptr = std::shared_ptr<Map>;
      Map() = default;
      ~Map() = default;

    public:
      inline int addFrame(const Frame::Ptr& new_frame)
      {
        if(hash_frames_.count(new_frame->getID()) == 0)
        {
          hash_frames_.insert(std::make_pair(new_frame->id_, new_frame));
          frames_vector_.emplace_back(new_frame);
          return 0;
        }
        AK_DLOG_WARNING << "Existed Frame added again.";
        return -1;
      };

      inline int addKeyFrame(const Frame::Ptr& new_keyframe)
      {
        addFrame(new_keyframe);
        if(hash_keyframes_.count(new_keyframe->getID()) == 0)
        {
          hash_keyframes_.insert(std::make_pair(new_keyframe->getID(), new_keyframe));
          keyframes_vector_.emplace_back(new_keyframe);
          return 0;
        }
        AK_DLOG_WARNING << "Existed KeyFrame added again.";
        return -1;
      };

      inline int addLandmark(const Landmark::Ptr& new_landmark)
      {
        if(hash_landmarks_.count(new_landmark->getID()) == 0)
        {
          landmarks_vector_.push_back(new_landmark);
          hash_landmarks_.insert(std::make_pair(new_landmark->getID(), new_landmark));
          return 0;
        }
        return -1;
      }

      inline size_t getSize()
      { return frames_vector_.size();}

      inline const std::vector<Frame::Ptr>& getKeyframes()
      { return keyframes_vector_;}

      inline const std::vector<Landmark::Ptr>& getLandmarks()
      { return landmarks_vector_;}

      int initializeMap();
      void resetMap();

      // For local map
      inline void addLocalQueue(Frame::Ptr& ptr_local_frame)
      {
        keyframes_wait_to_local_optimized_.push_back(ptr_local_frame);
      }
      void optimizeLocalMap();
      void initLocalMap(const Frame::Ptr& ptr_local_current_frame);
      void addLocalQueue(const Frame::Ptr& ptr_local_current_frame);

    protected:
      void resetLocalMap();
      void resetGlobalMap();
      void newTriangLandmark(Frame::Ptr& ptr_local_current_frame);
      void screenLocalLandmark(std::list<Landmark::Ptr>& local_landmarks);
      void screenKeyFrame(Frame::Ptr& ptr_local_current_frame);
      void searchNeighborKeyframes();


    private:
    /*static*/ std::vector<Frame::Ptr> frames_vector_;
    /*static*/ std::vector<Frame::Ptr> keyframes_vector_;
    /*static*/ std::unordered_map<ID_t, Frame::Ptr> hash_frames_;
    /*static*/ std::unordered_map<ID_t, Frame::Ptr> hash_keyframes_;
    std::vector<Landmark::Ptr> landmarks_vector_;
    std::vector<Landmark::Ptr> landmarks_vector_valid_;
    std::unordered_map<ID_t, Landmark::Ptr> hash_landmarks_;
    std::unordered_map<ID_t, Landmark::Ptr> hash_landmarks_valid_;

    //Local Map
    Frame::Ptr ptr_local_current_frame_;
    std::list<Frame::Ptr> keyframes_wait_to_local_optimized_;
    std::list<Frame::Ptr> local_keyframes_;
    std::list<Landmark::Ptr> local_landmarks_;

    bool is_abort_BA{false};
  };
}
#endif // __MAP_HH__
