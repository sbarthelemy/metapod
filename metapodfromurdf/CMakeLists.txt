## Copyright (c) 2012, 2013 Aldebaran Robotics. All rights reserved
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING.bsd file

QI_CREATE_BIN(metapodfromurdf
  src/metapodfromurdf.cpp)
QI_USE_LIB(metapodfromurdf
  metapod_robotbuilder
  boost boost_program_options metapod_robotbuilder eigen3)
QI_USE_LIB(metapodfromurdf urdfdom)
# without the following line, it won't compile
target_link_libraries(metapodfromurdf ${urdfdom_LIBRARIES})
QI_STAGE_BIN(metapodfromurdf)
QI_INSTALL_TARGET(metapodfromurdf)
SET(METAPODFROMURDF_EXECUTABLE metapodfromurdf CACHE
  STRING "Set by cmake if metapodfromurdf is built, by the user otherwise")
