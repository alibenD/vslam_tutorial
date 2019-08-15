#---Automatically Generated from template 'CMakeList.txt' wrote by @aliben---
#
# @Copyright (C) 2018 All rights reserved.
# @filename: visual_slam_tutorial
# @author: aliben.develop@gmail.com
# @create_date: 2018-11-20 10:31:08
# @last_modified_date: 2019-03-21 16:22:56
# @description: TODO
#---****************************************************************---


FILE(GLOB TEST_CASES *.cc)
FOREACH(TEST_CASE ${TEST_CASES})
  GET_FILENAME_COMPONENT(MY_TARGET "${TEST_CASE}" NAME_WE)
  ADD_EXECUTABLE(${MY_TARGET} ${TEST_CASE})
  TARGET_LINK_LIBRARIES(${MY_TARGET} 
    ${THIRD_PARTY_LIBS}
    ${TEST_LIBS}
    visual_slam_tutorial)
  ADD_TEST(${MY_TARGET} ${EXECUTABLE_OUTPUT_PATH}/${MY_TARGET})
ENDFOREACH()
