#ifndef __TIME_HH__
#define __TIME_HH__
/**-----------------------------------------------
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: time.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-02-27 18:50:29
  * @last_modified_date: 2019-02-28 09:25:22
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <chrono>
#include <visual_slam/logger_advanced.hh>
#include <iostream>

// Declaration
#define TIMER_START(str) ak::utils::Timer::start(str);
#define TIMER_END(str) ak::utils::Timer::end();
namespace ak
{
  namespace utils
  {
    class Timer;

    class Timer
    {
      public:
        Timer() = default;
        ~Timer() = default;

        static void start(const std::string& title)
        {
          Timer::title_ = title;
          Timer::time_start_ = std::chrono::system_clock::now();
        };
        static void end()
        {
          Timer::time_end_ = std::chrono::system_clock::now();
          Timer::time_duration_ = time_end_ - time_start_;
          AK_LOG_INFO << Timer::title_ << "-Time cost: " << Timer::time_duration_.count();
        };
        
        static std::chrono::time_point<std::chrono::system_clock> time_start_;
        static std::chrono::time_point<std::chrono::system_clock> time_end_;
        static std::chrono::duration<double> time_duration_;
        static std::string title_;
    };
  }
}
#endif // __TIME_HH__
