#!/bin/bash

#---Automatically Generated from template 'bash' wrote by @aliben---
# @Copyright (C) 2018 All rights reserved.
# @file: 
# @author: aliben.develop@gmail.com
# @created_date: 2018-11-20 10:31:08
# @last_modified_date: 2019-01-11 15:50:57
# @brief: TODO
# @details: TODO
#---***********************************************---


#---Variables
CREATED_TIME=`date '+%Y-%m-%d %H:%M:%S'`
CREATED_YEAR=`date '+%Y'`

#---Shell Command
set -x
mkdir -p build
cd build
rm -rf build/*
cmake ..
make -j7
