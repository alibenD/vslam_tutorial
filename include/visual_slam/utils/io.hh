#ifndef __IO_HH__
#define __IO_HH__
/**-----------------------------------------------
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: io.hh
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-01-18 14:32:36
  * @last_modified_date: 2019-02-27 10:03:02
  * @brief: TODO
  * @details: TODO
  *-----------------------------------------------*/

// Header include
#include <visual_slam.hh>
#include <vector>
#include <string>
#include <fstream>
//#include <ofstream>

namespace ak{

// Declaration
int ReadFileList(const std::string& path, std::vector<std::string>& item_list);

// Save binary file, use:
// // std::ofstream file;
// file.open(path, std::ios_base::out | std::ios_base::binary);
// file.read(path, std::ios_base::binary);

template <typename T>
int WriteBinaryData(T& data, std::ofstream& file)
{
  auto cast_data_pointer = reinterpret_cast<char*>(&data);
  file.write(cast_data_pointer, sizeof(data));
  return 0;
};

template <typename T>
int ReadBinaryData(T& data, std::ifstream& file)
{
  char* cast_data_pointer = reinterpret_cast<char*>(&data);
  file.read(cast_data_pointer, sizeof(data));
  return 0;
};

template <typename T>
int ReadPlainData(T& data, std::ifstream& file)
{
  file >> data;
  return 0;
}

} // namespace ak
#endif // __IO_HH__
