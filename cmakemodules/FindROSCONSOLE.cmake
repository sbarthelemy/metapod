## Copyright (c) 2012, 2013 Aldebaran Robotics. All rights reserved
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING.bsd file
get_filename_component(_ROOT_DIR ${CMAKE_CURRENT_LIST_FILE} PATH)
include("${_ROOT_DIR}/rosutils.cmake")
clean(ROSCONSOLE)
fpath(ROSCONSOLE "ros/console.h")
flib(ROSCONSOLE "rosconsole")
export_lib(ROSCONSOLE)
