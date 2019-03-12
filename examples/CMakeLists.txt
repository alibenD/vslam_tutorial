#---Automatically Generated from template 'CMakeList.txt' wrote by @aliben---
#
# @Copyright (C) 2018 All rights reserved.
# @filename: visual_slam_tutorial
# @author: aliben.develop@gmail.com
# @create_date: 2018-11-20 10:31:08
# @last_modified_date: 2019-03-12 10:00:30
# @description: TODO
#---****************************************************************---


FILE(GLOB_RECURSE EXAMPLES *.cc)
FOREACH(EXAMPLE ${EXAMPLES})
  GET_FILENAME_COMPONENT(MY_TARGET "${EXAMPLE}" NAME_WE)
  IF(${CMAKE_BUILD_TYPE} STREQUAL "RELEASE")
    ADD_EXECUTABLE(${MY_TARGET} ${EXAMPLE})
    TARGET_LINK_LIBRARIES(${MY_TARGET}
      ${THIRD_PARTY_LIBS}
      visual_slam_tutorial)
  ELSEIF(${CMAKE_BUILD_TYPE} STREQUAL "DEBUG")
    ADD_EXECUTABLE(${MY_TARGET}${CMAKE_DEBUG_POSTFIX} ${EXAMPLE})
    TARGET_LINK_LIBRARIES(${MY_TARGET}${CMAKE_DEBUG_POSTFIX}
      ${THIRD_PARTY_LIBS}
      visual_slam_tutorial${CMAKE_DEBUG_POSTFIX})
  ENDIF()
ENDFOREACH()
