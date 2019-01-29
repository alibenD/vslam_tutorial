/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: io.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-18 14:53:56
  * @last_modified_date: 2019-01-29 14:00:20
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <visual_slam/utils/io.hh>

//CODE
int ReadFileList(const std::string& path, std::vector<std::string>& item_list)
{
  std::ifstream file_in(path);
  if(!file_in)
  {
    AK_LOG(FATAL) << "No such file calling " << path;
    return -1;
  }
  while(!file_in.eof())
  {
    std::string item;
    file_in >> item;
    if(file_in.good() == false)
    {
      break;
    }
    item_list.push_back(item);
  }
  return item_list.size();
}
