/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: read_txt.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-02-27 09:29:23
  * @last_modified_date: 2019-02-27 10:02:06
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
  std::ifstream file_keys1("../data/keys1.txt");
  int kps_size;
  file_keys1 >> kps_size;
  std::cout << "Kps_size: " << kps_size << std::endl;
  float x, y, angle, p_size, response;
  for(int i=0; i<kps_size; ++i)
  {
    //file_keys1 >> x;
    //file_keys1 >> y;
    //file_keys1 >> p_size;
    //file_keys1 >> angle;
    //file_keys1 >> response;
    ReadPlainData(x, file_keys1);
    ReadPlainData(y, file_keys1);
    ReadPlainData(p_size, file_keys1);
    ReadPlainData(angle, file_keys1);
    ReadPlainData(response, file_keys1);
    std::cout << "(" << x << " ," << y << ")\t" << angle << "\t"
              << p_size << "\t" << response << std::endl;
  }
  for(int i=0; i<3; ++i)
  {
    float element;
    for(int j=0; j<3; ++j)
    {
      //file_keys1 >> element;
      ReadPlainData(element, file_keys1);
      std::cout << element << " ";
    }
    std::cout << std::endl;
  }
  file_keys1.close();
}
