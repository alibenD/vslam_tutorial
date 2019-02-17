#---Automatically Generated from template 'CMakeList.txt' wrote by @aliben---
#
# @Copyright (C) 2018 All rights reserved.
# @filename: visual_slam_tutorial
# @author: aliben.develop@gmail.com
# @create_date: 2018-11-20 10:31:08
# @last_modified_date: 2019-02-17 18:30:35
# @description: TODO
#---****************************************************************---

# Cmake version required
cmake_minimum_required(VERSION 3.10)

#project name
project(visual_slam)

# CMake Build System Default Setup
  MESSAGE( STATUS "Project:  visual_slam_tutorial" )
  # Build type default
  SET(CMAKE_BUILD_TYPE DEBUG)
  IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE RELEASE)
  ENDIF()
  IF(CMAKE_BUILD_TYPE STREQUAL "RELEASE")
    ADD_DEFINITIONS(-DNDEBUG)
  ENDIF()
  MESSAGE( STATUS "Build Type: " ${CMAKE_BUILD_TYPE})
  OPTION(BUILD_GTESTS "Build test example cases with google test" ON)
  OPTION(WITH_GLOG "Enable google log in this case" ON)

  SET(CMAKE_VERBOSE_MAKEFILE OFF)
  SET(CMAKE_CXX_STANDARD 11)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -std=c++14 -Wextra -Wno-deprecated-register -Wno-deprecated-declarations -Wno-unused-private-field -Wno-unused-parameter -Wno-unused-variable -Qunused-arguments -fcolor-diagnostics -ftemplate-backtrace-limit=0")
  #SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -std=c++14 -Wextra -Wno-deprecated-register -Wno-deprecated-declarations -Qunused-arguments -fcolor-diagnostics -ftemplate-backtrace-limit=0")
  # Cross-platform check
  IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3")
  ELSEIF(CMAKE_HOST_APPLE)
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3")
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -O0 -g")
  ELSEIF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    SET(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
    SET(CMAKE_CXX_FLAGS_RELEASE "-O3")
    SET(CMAKE_EXE_LINKER_FLAGS_COVERAGE "${CMAKE_EXE_LINKER_FLAGS_DEBUG}" --coverage)
    SET(CMAKE_SHARED_LINKER_FLAGS_COVERAGE "${CMAKE_SHARED_LINKER_FLAGS_DEBUG}" --coverage)
  ELSEIF(CMAKE_CXX_COMPILER_ID MATCHES "^MSVC$")
    ADD_DEFINITION("-D _USE_MATH_DEFINES /bigobj /wd4305 /wd4244 /MP")
    ADD_DEFINITION("-D PROJECT_CORE_EXPORTS")
  ENDIF()

  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake_modules/")
  SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
  OPTION(BUILD_SHARED_LIBS "Build shared libraries" ON)
  SET(${PROJECT_NAME}_LIB_TYPE STATIC)
  IF(BUILD_SHARED_LIBS)
    SET(BUILD_SHARED_LIBS SHARED)
  ENDIF()
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add extra Find module
  LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
# Find *Config.cmake
  # Example: find_package(OpenCV REQUIRED)
  IF(BUILD_GTESTS)
    ENABLE_TESTING()
    find_package(GTEST REQUIRED)
    find_package(Threads REQUIRED)
    INCLUDE_DIRECTORIES(${GTEST_INCLUDE_DIRS})
    SET( TEST_LIBS
    ${GTEST_BOTH_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT})
  ENDIF()
  IF(WITH_GLOG)
    ADD_DEFINITIONS(-DENABLE_GLOG)
  ENDIF()
  find_package(Eigen3 REQUIRED)
  find_package(Pangolin REQUIRED)
  find_package(Sophus REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(GLM REQUIRED)
  find_package(GLFW3 REQUIRED)
  find_package(ASSIMP REQUIRED)
  #find_package(QT5 REQUIRED)
  find_package(G2O REQUIRED)
  find_package(Glog REQUIRED)
  message(STATUS "GLM included at ${GLM_INCLUDE_DIR}")
  message(STATUS "Found GLFW3 in ${GLFW3_INCLUDE_DIR}")
  message(STATUS "Found ASSIMP in ${ASSIMP_INCLUDE_DIR}")

# Set&Add INCLUDE PATH
  # Example: SET ( VPATH_NAME PATH )
  #          INCLUDE_DIRECTORIES(${VPATH_NAME})
  INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)
  INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/ThirdParty/include)
  INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/build/include)
  INCLUDE_DIRECTORIES(${CMAKE_INSTALL_PREFIX}/include)
  INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
  INCLUDE_DIRECTORIES(${Sophus_INCLUDE_DIRS})
  INCLUDE_DIRECTORIES(${Pangolin_INCLUDE_DIRS})
  INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
  #INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR})
  INCLUDE_DIRECTORIES(${GLOG_INCLUDE_DIRS})
  INCLUDE_DIRECTORIES(/System/Library/Frameworks)

# Set&Add LIB PATH/LINKING_DIRECTORIES
  # Example: LINK_DIRECTORIES(${CMAKE_INSTALL_PREFIX}/lib)
  LINK_DIRECTORIES(${CMAKE_INSTALL_PREFIX}/lib)
  LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib)
  LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/lib)


  SET(APPLE_LIBS ${COCOA_LIBRARY} ${IOKit_LIBRARY} ${OpenGL_LIBRARY} ${CoreVideo_LIBRARY})
  SET(APPLE_LIBS ${APPLE_LIBS} ${GLFW3_LIBRARY} ${ASSIMP_LIBRARY})
  SET(LIBS ${LIBS} ${APPLE_LIBS})
  SET( THIRD_PARTY_LIBS
    ${THIRD_PARTY_LIBS}
    ${Eigen3_LIBS}
    ${Pangolin_LIBRARIES}
    ${Sophus_LIBRARIES}
    ${OpenCV_LIBS}
    ${LIBS}
    ${G2O_STUFF_LIBRARY}
    ${G2O_CORE_LIBRARY}
    ${G2O_CLI_LIBRARY}
    ${GLOG_LIBRARIES}
  )

#MESSAGE("*****" ${Pangolin_LIBS})
#MESSAGE("#####" ${Pangolin_LIBRARIES})

# Set Subdir(src)
  ADD_SUBDIRECTORY(ThirdParty/src)
  ADD_SUBDIRECTORY(src)
  ADD_SUBDIRECTORY(examples)
  IF(BUILD_GTESTS)
    ADD_SUBDIRECTORY(test)
  ENDIF()

# EXECUTABLE
  # Example: ADD_EXECUTABLE( EXEC_NAME SRC_FILE_NAME_LIST )

# TARGET LINK
  # Example: TARGET_LINK_LIBRARIES( EXEC_NAME LIBPATH ) ...LIB_PATH e.g. ${OPENCV_LIBS}
