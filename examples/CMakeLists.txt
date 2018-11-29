#---Automatically Generated from template 'CMakeList.txt' wrote by @aliben---
#
# @Copyright (C) 2018 All rights reserved.
# @filename: visual_slam_tutorial
# @author: aliben.develop@gmail.com
# @create_date: 2018-11-20 10:31:08
# @last_modified_date: 2018-11-28 10:14:21
# @description: TODO
#---****************************************************************---


FILE(GLOB_RECURSE EXAMPLES *.cc)
FOREACH(EXAMPLE ${EXAMPLES})
  GET_FILENAME_COMPONENT(MY_TARGET "${EXAMPLE}" NAME_WE)
  ADD_EXECUTABLE(${MY_TARGET} ${EXAMPLE})
  TARGET_LINK_LIBRARIES(${MY_TARGET} visual_slam_tutorial)
ENDFOREACH()
