/**
  * @Copyright (C) 2019 All rights reserved.
  * @date: 2019
  * @file: convert_voc_to_bin.cc
  * @version: v0.0.1
  * @author: aliben.develop@gmail.com
  * @create_date: 2019-05-15 16:39:52
  * @last_modified_date: 2019-05-15 18:46:37
  * @brief: TODO
  * @details: TODO
  */

//INCLUDE
#include <DBoW3/DBoW3.h>
#include <visual_slam/logger_advanced.hh>

//CODE
int main(int argc, char** argv)
{
  if(argc != 2)
  {
    AK_LOG_FATAL << "Usage: " << argv[0] << " voc_yaml";
    exit(-1);
  }
  std::string voc_file(argv[1]);
  std::string voc_binfile = voc_file.substr(0, voc_file.rfind("."))+".bin";
  AK_LOG_INFO << "Origin: " << voc_file;
  AK_LOG_INFO << "Bin: " << voc_binfile;
  DBoW3::Vocabulary voc;
  voc.load(voc_file);
  AK_LOG_INFO << "Converting binary";
  voc.save(voc_binfile, true);
  AK_LOG_INFO << "Convert Done";
  return 0;
}
