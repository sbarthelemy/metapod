# Copyright 2011, 2012,
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
include(cmakeutils.cmake)
enable_testing()
qi_sanitize_compile_flags(HIDDEN_SYMBOLS)

# generate config.hh using the same template as JRL-cmake
generate_config_header(
  ${CMAKE_CURRENT_BINARY_DIR}/include/metapod/config.hh
  metapod)

OPTION(WITH_SIMPLE_HUMANOID "Build the simple humanoid example model" ON)
OPTION(WITH_SIMPLE_ARM "Build the simple arm example model" ON)
OPTION(BENCHMARK "Build the benchmarking tool" OFF)
IF (NOT CMAKE_CROSSCOMPILING)
  SET(BENCHMARK ON)
ENDIF()

# Set header files.
SET(${PROJECT_NAME}_ALGOS_HEADERS
  include/${PROJECT_NAME}/algos/rnea.hh
  include/${PROJECT_NAME}/algos/crba.hh
  include/${PROJECT_NAME}/algos/jac_point_relative.hh
  include/${PROJECT_NAME}/algos/jac_point_chain.hh
  )

SET(${PROJECT_NAME}_TOOLS_HEADERS
  include/${PROJECT_NAME}/tools/fwd.hh
  include/${PROJECT_NAME}/macro.hh
  include/${PROJECT_NAME}/tools/static_assert.hh
  include/${PROJECT_NAME}/tools/common.hh
  include/${PROJECT_NAME}/tools/spatial.hh
  include/${PROJECT_NAME}/tools/jointmacros.hh
  include/${PROJECT_NAME}/tools/bodymacros.hh
  include/${PROJECT_NAME}/tools/has_parent.hh
  include/${PROJECT_NAME}/tools/is_ancestor.hh
  include/${PROJECT_NAME}/tools/deepest_common_ancestor.hh
  include/${PROJECT_NAME}/tools/depth_first_traversal.hh
  include/${PROJECT_NAME}/tools/backward_traversal.hh
  include/${PROJECT_NAME}/tools/backward_traversal_prev.hh
  include/${PROJECT_NAME}/tools/initconf.hh
  include/${PROJECT_NAME}/tools/print.hh
  include/${PROJECT_NAME}/tools/jcalc.hh
  include/${PROJECT_NAME}/tools/bcalc.hh
  include/${PROJECT_NAME}/tools/jac_point_robot.hh
  ${CMAKE_CURRENT_BINARY_DIR}/include/metapod/config.hh
  )

INCLUDE_DIRECTORIES("include" "${CMAKE_CURRENT_BINARY_DIR}/include")

QI_CREATE_LIB(metapod
  ${${PROJECT_NAME}_ALGOS_HEADERS}
  ${${PROJECT_NAME}_TOOLS_HEADERS})
SET_TARGET_PROPERTIES(metapod PROPERTIES LINKER_LANGUAGE CXX)
QI_USE_LIB(metapod EIGEN3 BOOST)
QI_STAGE_HEADER_ONLY_LIB(metapod)


IF(${WITH_SIMPLE_HUMANOID})
  ADD_SUBDIRECTORY(simple_humanoid)
ENDIF()
IF(${WITH_SIMPLE_ARM})
  ADD_SUBDIRECTORY(simple_arm)
ENDIF()
ADD_SUBDIRECTORY(tests)

IF(${BENCHMARK})
  ADD_SUBDIRECTORY(timer)
  ADD_SUBDIRECTORY(benchmark)
ENDIF(${BENCHMARK})
