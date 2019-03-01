/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: read_computeH_data.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-02-26 18:13:57
  * @last_modified_date: 2019-02-27 10:04:56
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <visual_slam/utils/io.hh>
#include <iostream>

//CODE
using namespace ak;
int main()
{
  std::ifstream file;
  file.open("../data/date_computeH21_origin.bin");
  float x1, y1, x2, y2;
  int count = 0;
  while(!file.eof())
  {
    ReadBinaryData(x1, file);
    ReadBinaryData(y1, file);
    ReadBinaryData(x2, file);
    ReadBinaryData(y2, file);
    std::cout << "(" << x1 << ", " << y1 << ")\t->"
              << "\t(" << x2 << ", " << y2 << ")"
              << std::endl;
    //std::cout << "HEHE" << std::endl;
    if(file.peek() == EOF)
    {
      break;
    }
    //std::cout << "count: " << count << std::endl;
    ++count;
  }
  file.close();
  return 0;
}
