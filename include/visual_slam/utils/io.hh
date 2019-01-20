#ifndef __IO_HH__
#define __IO_HH__
/**-----------------------------------------------
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: io.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-18 14:32:36
  * @last_modified_date: 2019-01-20 21:34:45
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <visual_slam.hh>
#include <vector>
#include <string>

// Declaration
int ReadFileList(const std::string& path, std::vector<std::string>& item_list);
#endif // __IO_HH__
