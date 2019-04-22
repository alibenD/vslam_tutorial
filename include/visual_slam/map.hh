#ifndef __MAP_HH__
#define __MAP_HH__
/**-----------------------------------------------
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: map.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-03-03 11:06:28
  * @last_modified_date: 2019-04-22 16:59:56
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <visual_slam/frame.hh>
#include <visual_slam/landmark.hh>
#include <visual_slam/utils/type.hh>

// Declaration
namespace ak
{
  class Map;
  class Map
  {
    public:
      friend class VisualOdometry;
      using Ptr = std::shared_ptr<Map>;
      Map() = default;
      ~Map() = default;

    public:
      inline int addFrame(const Frame::Ptr& new_frame)
      {
        hash_frames_.insert(std::make_pair(new_frame->id_, new_frame));
        frames_vector_.emplace_back(new_frame);
        return 0;
      };

      inline int addKeyFrame(const Frame::Ptr& new_keyframe)
      {
        addFrame(new_keyframe);
        hash_keyframes_.insert(std::make_pair(new_keyframe->getID(), new_keyframe));
        keyframes_vector_.emplace_back(new_keyframe);
        return 0;
      };

      inline int addLandmark(const Landmark::Ptr& new_landmark)
      {
        landmarks_vector_.push_back(new_landmark);
        hash_landmarks_.insert(std::make_pair(new_landmark->getID(), new_landmark));
        return 0;
      }

      inline size_t getSize()
      { return frames_vector_.size();}

      inline const std::vector<Frame::Ptr>& getKeyframes()
      { return keyframes_vector_;}

      inline const std::vector<Landmark::Ptr>& getLandmarks()
      { return landmarks_vector_;}

      int initializeMap();

    protected:
      void resetLocalMap();
      void updateLocalMap();


    private:
    /*static*/ std::vector<Frame::Ptr> frames_vector_;
    /*static*/ std::vector<Frame::Ptr> keyframes_vector_;
    std::vector<Landmark::Ptr> landmarks_vector_;
    /*static*/ std::unordered_map<ID_t, Frame::Ptr> hash_frames_;
    /*static*/ std::unordered_map<ID_t, Frame::Ptr> hash_keyframes_;
    std::unordered_map<Landmark::ID_t, Landmark::Ptr> hash_landmarks_;

    //Local Map
    std::vector<Frame::Ptr> local_keyframes_;
    std::vector<Landmark::Ptr> local_landmarks_;
  };
}
#endif // __MAP_HH__
