#!/bin/bash

#---Automatically Generated from template 'bash' wrote by @aliben---
# @Copyright (C) 2019 All rights reserved.
# @file: 
# @author: aliben.develop@gmail.com
# @created_date: 2019-01-11 16:35:17
# @last_modified_date: NO_LAST_MODIFIED_DATE
# @brief: TODO
# @details: TODO
#---***********************************************---


#---Variables
CREATED_TIME=`date '+%Y-%m-%d %H:%M:%S'`
CREATED_YEAR=`date '+%Y'`

#---Shell Command
set -x
cd build
make test
