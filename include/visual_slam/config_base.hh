#ifndef __CONFIG_BASE_HH__
#define __CONFIG_BASE_HH__
/**-----------------------------------------------
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: config_base.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-14 09:31:38
  * @last_modified_date: 2019-01-14 11:20:45
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <string>

// Declaration
namespace ak
{
  class ConfigBase
  {
    public:
      ConfigBase() = default;
      virtual ~ConfigBase() = default;

    public:
      virtual int open(const std::string& path) = 0;
      virtual int release() = 0;
      
      //virtual int read(const std::string& item_name) = 0;
      //virtual int write(const std::string& path) = 0;
  };
}
#endif // __CONFIG_BASE_HH__
