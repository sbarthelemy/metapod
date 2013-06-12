## Copyright (c) 2012, 2013 Aldebaran Robotics. All rights reserved
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING.bsd file

# find urdf as provided with ROS fuerte

get_filename_component(_ROOT_DIR ${CMAKE_CURRENT_LIST_FILE} PATH)
include("${_ROOT_DIR}/rosutils.cmake")
# add the robot_model stack to the list of prefix cmake will consider.
foreach(root ${ROS_ROOTS})
  list(APPEND CMAKE_FIND_ROOT_PATH ${root}/stacks/robot_model)
endforeach()

clean(URDF)
fpath(URDF "urdf_interface/link.h" PATHS /urdf_interface/include)

fpath(URDF "urdf_parser/urdf_parser.h"  PATHS /urdf_parser/include)

fpath(URDF "collada_parser/collada_parser.h" PATHS /collada_parser/include)
flib(URDF "minizip"  PATHS /colladadom/lib)

fpath(URDF "tinyxml.h")
flib(URDF "tinyxml")

flib(URDF "roscpp")
flib(URDF "rosconsole")

fpath(URDF "urdf/model.h" PATHS /urdf/include)
flib(URDF "urdf"  PATHS /urdf/lib)
export_lib(URDF)
