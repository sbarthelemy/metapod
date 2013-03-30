## Copyright (c) 2012, 2013 Aldebaran Robotics. All rights reserved
## Use of this source code is governed by a BSD-style license that can be
## found in the COPYING.bsd file

QI_CREATE_BIN(metapodfromurdf
  src/metapodfromurdf.cpp)
QI_USE_LIB(metapodfromurdf
  metapod_robotbuilder
  boost_program_options urdf metapod_robotbuilder eigen3)
# rosconsole emits lots "variadic macro warnings". Let ignore them.
QI_USE_LIB(metapodfromurdf
   rosconsole
   ASSUME_SYSTEM_INCLUDE)
QI_STAGE_BIN(metapodfromurdf)
QI_INSTALL_TARGET(metapodfromurdf)
SET(METAPODFROMURDF_EXECUTABLE metapodfromurdf CACHE
  STRING "Set by cmake if metapodfromurdf is built, by the user otherwise")
