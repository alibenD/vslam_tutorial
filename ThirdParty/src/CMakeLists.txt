#---Automatically Generated from template 'CMakeList.txt' wrote by @aliben---
#
# @Copyright (C) 2018 All rights reserved.
# @filename: visual_slam_tutorial
# @author: aliben.develop@gmail.com
# @create_date: 2018-11-20 10:31:08
# @last_modified_date: 2019-02-17 17:31:46
# @description: TODO
#---****************************************************************---

FILE(GLOB_RECURSE TYPES_HEADERS_THIRD
  ../../include/visual_slam_tutorial/*/*.h*
  ../../include/visual_slam_tutorial/*.h*
  ../../include/*.h*
  ../include/*.h*
  )
FILE(GLOB_RECURSE SOURCES_THIRD *.cc *.cpp *.c *.cxx)

ADD_LIBRARY(orb_lib SHARED ${TYPES_HEADERS_THIRD} ${SOURCES_THIRD})
TARGET_LINK_LIBRARIES(orb_lib ${OpenCV_LIBS})
