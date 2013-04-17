# Copyright 2011, 2012, 2013
#
# Maxime Reis (JRL/LAAS, CNRS/AIST)
# Antonio El Khoury (JRL/LAAS, CNRS/AIST)
# Sébastien Barthélémy (Aldebaran Robotics)
#
# This file is part of metapod.
# metapod is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# metapod is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with metapod.  If not, see <http://www.gnu.org/licenses/>.
#
# Creation: 14/06/2011
cmake_minimum_required(VERSION 2.8)
project(metapod)
find_package(qibuild)
include(CMakeDependentOption)
include(cmakeutils.cmake)
enable_testing()
qi_sanitize_compile_flags(HIDDEN_SYMBOLS)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmakemodules")


# generate config.hh using the same template as JRL-cmake
generate_config_header(
  ${CMAKE_CURRENT_BINARY_DIR}/include/metapod/config.hh
  metapod)
# Set header files.
SET(${PROJECT_NAME}_ROOT_HEADERS
  ${CMAKE_CURRENT_BINARY_DIR}/include/metapod/config.hh
  include/${PROJECT_NAME}/macro.hh
  )
SET(${PROJECT_NAME}_ALGOS_HEADERS
  include/${PROJECT_NAME}/algos/rnea.hh
  include/${PROJECT_NAME}/algos/crba.hh
  include/${PROJECT_NAME}/algos/jac.hh
  include/${PROJECT_NAME}/algos/jac_point_relative.hh
  include/${PROJECT_NAME}/algos/jac_point_chain.hh
  )
SET(${PROJECT_NAME}_TOOLS_HEADERS
  include/${PROJECT_NAME}/tools/backward.hh
  include/${PROJECT_NAME}/tools/backward_traversal.hh
  include/${PROJECT_NAME}/tools/backward_traversal_prev.hh
  include/${PROJECT_NAME}/tools/bcalc.hh
  include/${PROJECT_NAME}/tools/common.hh
  include/${PROJECT_NAME}/tools/constants.hh
  include/${PROJECT_NAME}/tools/deepest_common_ancestor.hh
  include/${PROJECT_NAME}/tools/depth_first_traversal.hh
  include/${PROJECT_NAME}/tools/fwd.hh
  include/${PROJECT_NAME}/tools/has_parent.hh
  include/${PROJECT_NAME}/tools/initconf.hh
  include/${PROJECT_NAME}/tools/is_ancestor.hh
  include/${PROJECT_NAME}/tools/jac_point_robot.hh
  include/${PROJECT_NAME}/tools/jcalc.hh
  include/${PROJECT_NAME}/tools/joint.hh
  include/${PROJECT_NAME}/tools/joint-freeflyer.hh
  include/${PROJECT_NAME}/tools/joint-freeflyerbody.hh
  include/${PROJECT_NAME}/tools/joint-aboutxaxis.hh
  include/${PROJECT_NAME}/tools/joint-aboutyaxis.hh
  include/${PROJECT_NAME}/tools/joint-aboutzaxis.hh
  include/${PROJECT_NAME}/tools/joint-anyaxis.hh
  include/${PROJECT_NAME}/tools/print.hh
  include/${PROJECT_NAME}/tools/spatial.hh
  include/${PROJECT_NAME}/tools/static_assert.hh
  )
SET(${PROJECT_NAME}_SPATIAL_HEADERS
  include/${PROJECT_NAME}/tools/spatial/cm-anyaxis.hh
  include/${PROJECT_NAME}/tools/spatial/cm-freeflyer.hh
  include/${PROJECT_NAME}/tools/spatial/cm-oneaxis.hh
  include/${PROJECT_NAME}/tools/spatial/constraintmotion.hh
  include/${PROJECT_NAME}/tools/spatial/force.hh
  include/${PROJECT_NAME}/tools/spatial/inertia.hh
  include/${PROJECT_NAME}/tools/spatial/lti.hh
  include/${PROJECT_NAME}/tools/spatial/motion.hh
  include/${PROJECT_NAME}/tools/spatial/rm-aboutxaxis.hh
  include/${PROJECT_NAME}/tools/spatial/rm-aboutyaxis.hh
  include/${PROJECT_NAME}/tools/spatial/rm-aboutzaxis.hh
  include/${PROJECT_NAME}/tools/spatial/rm-binaryop.hh
  include/${PROJECT_NAME}/tools/spatial/rm-chgaxis.hh
  include/${PROJECT_NAME}/tools/spatial/rm-general.hh
  include/${PROJECT_NAME}/tools/spatial/rm-identity.hh
  include/${PROJECT_NAME}/tools/spatial/rm-mulop.hh
  include/${PROJECT_NAME}/tools/spatial/rotation-matrix.hh
  include/${PROJECT_NAME}/tools/spatial/transform.hh
  )

FIND_PACKAGE(BOOST)
INCLUDE_DIRECTORIES(${BOOST_INCLUDE_DIRS})

# Eigen (at least version 3.0.5) makes gcc report conversion warnings
# when one assigns to a block. This floods the compilation results.
# So we include Eigen headers as if they were system headers: the compiler
# won't report warning coming from them.
FIND_PACKAGE(EIGEN3)
INCLUDE_DIRECTORIES(SYSTEM ${EIGEN3_INCLUDE_DIRS})

INCLUDE_DIRECTORIES("include" "${CMAKE_CURRENT_BINARY_DIR}/include")
QI_STAGE_HEADER_ONLY_LIB(metapod DEPENDS BOOST EIGEN3)
QI_INSTALL_HEADER(${${PROJECT_NAME}_ROOT_HEADERS}
  SUBFOLDER metapod)
QI_INSTALL_HEADER(${${PROJECT_NAME}_ALGOS_HEADERS}
  SUBFOLDER metapod/algos)
QI_INSTALL_HEADER(${${PROJECT_NAME}_TOOLS_HEADERS}
  SUBFOLDER metapod/tools)
QI_INSTALL_HEADER(${${PROJECT_NAME}_SPATIAL_HEADERS}
  SUBFOLDER metapod/tools/spatial)

# We build the generator tools only if we are not cross compiling.
# When cross compiling, the generators are searched for on the building host.
IF(NOT CMAKE_CROSSCOMPILING)
  ADD_SUBDIRECTORY(embedfile)
  ADD_SUBDIRECTORY(robotbuilder)
  ADD_SUBDIRECTORY(binarytreemodel)
  SET(WITH_METAPOD_BINARYTREEMODEL TRUE)
ELSE()
  FIND_GENERATOR(METAPOD_BINARYTREEMODEL)
ENDIF()

# Even when not cross compiling, we cannot always build metapodfromurdf,
# because it depends on ROS bits, which we do not control very well.
SET(BUILD_METAPODFROMURDF_DEFAULT FALSE)
IF(NOT CMAKE_CROSSCOMPILING)
  # by default, we build metapodfromurdf if we are not cros compiling and
  # we detect ros. Since the ros detection part is not fool proof.
  # we let the user override it.
  FIND_PROGRAM(ROSPACK_EXECUTABLE rospack)
  IF(ROSPACK_EXECUTABLE)
    SET(BUILD_METAPODFROMURDF_DEFAULT TRUE)
  ENDIF()
ENDIF()
CMAKE_DEPENDENT_OPTION(BUILD_METAPODFROMURDF
  "Build the urdf parser"
  ${BUILD_METAPODFROMURDF_DEFAULT}
  "NOT CMAKE_CROSSCOMPILING"
  OFF)
IF(BUILD_METAPODFROMURDF)
  ADD_SUBDIRECTORY(metapodfromurdf)
  SET(WITH_METAPODFROMURDF TRUE)
ELSE()
  FIND_GENERATOR(METAPODFROMURDF)
ENDIF()

OPTION(WITH_SIMPLE_HUMANOID
  "Build the simple humanoid example model and the related tests."
  ON)
OPTION(WITH_SIMPLE_ARM
  "Build the simple arm example model and the related tests."
  ON)

OPTION(BUILD_MULTI_MODEL_BENCHMARK
    "Generate and build the multi-model benchmark" TRUE)
OPTION(BUILD_SINGLE_MODEL_BENCHMARKS
    "Generate and build the single-model benchmarks" TRUE)


IF(WITH_SIMPLE_ARM)
  ADD_SAMPLEURDFMODEL("simple_arm")
ENDIF()
IF(WITH_SIMPLE_HUMANOID)
  ADD_SAMPLEURDFMODEL("simple_humanoid")
ENDIF()

IF(BUILD_MULTI_MODEL_BENCHMARK OR BUILD_SINGLE_MODEL_BENCHMARKS)
  ADD_SUBDIRECTORY(timer)
  ADD_SUBDIRECTORY(benchmark)
ENDIF()

ADD_SUBDIRECTORY(tests)
