#---Automatically Generated from template 'CMakeList.txt' wrote by @aliben---
#
# @Copyright (C) 2018 All rights reserved.
# @filename: visual_slam_tutorial
# @author: aliben.develop@gmail.com
# @create_date: 2018-11-20 10:31:08
# @last_modified_date: 2019-02-17 18:31:55
# @description: TODO
#---****************************************************************---

FILE(GLOB_RECURSE TYPES_HEADERS
  ../include/visual_slam_tutorial/*/*.h*
  ../include/visual_slam_tutorial/*.h*
  ../include/*.h
  )
FILE(GLOB_RECURSE SOURCES *.cc *.cpp *.c *.cxx)

ADD_LIBRARY(visual_slam_tutorial SHARED ${TYPES_HEADERS} ${SOURCES})
TARGET_LINK_LIBRARIES(visual_slam_tutorial
  orb_lib
  ${THIRD_PARTY_LIBS}
  )
