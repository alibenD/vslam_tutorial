#!/bin/bash

#---Automatically Generated from template 'bash' wrote by @aliben---
# @Copyright (C) 2018 All rights reserved.
# @file: build.sh
# @author: aliben.develop@gmail.com
# @created_date: 2018-11-20 10:31:08
# @last_modified_date: 2019-02-24 18:01:11
# @brief: TODO
# @details: TODO
#---***********************************************---


#---Variables
CREATED_TIME=`date '+%Y-%m-%d %H:%M:%S'`
CREATED_YEAR=`date '+%Y'`
CLEAN=$1
#---Shell Command
set -x
if [ ! -d "build" ]; then
  mkdir -p build
fi
if [ "${CLEAN}" == "clean" ]; then
  rm -rf bin/*
  rm -rf build/*
  rm -rf lib/*
fi

if [ "${CLEAN}" == "debug" ]; then
  BUILD_TYPE=DEBUG
else
  BUILD_TYPE=RELEASE
fi
ln -s `pwd`/log ./bin/log
cd build
cmake .. -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
make -j7
