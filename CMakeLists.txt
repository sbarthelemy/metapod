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
enable_testing()
qi_sanitize_compile_flags(HIDDEN_SYMBOLS)

# generate config.hh using the same template as JRL-cmake
set(LIBRARY_NAME "METAPOD")
set(PROJECT_VERSION "999")
set(EXPORT_SYMBOL "metapod_EXPORTS")
configure_file(
  include/metapod/config.in.hh
  ${CMAKE_CURRENT_BINARY_DIR}/include/metapod/config.hh
  @ONLY)

# Set header files.
set(${PROJECT_NAME}_ALGOS_HEADERS
  include/${PROJECT_NAME}/algos/rnea.hh
  include/${PROJECT_NAME}/algos/crba.hh
  include/${PROJECT_NAME}/algos/jac_point_relative.hh
  include/${PROJECT_NAME}/algos/jac_point_chain.hh
  )

set(${PROJECT_NAME}_TOOLS_HEADERS
  include/${PROJECT_NAME}/tools/fwd.hh
  include/${PROJECT_NAME}/macro.hh
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

include_directories("include" "${CMAKE_CURRENT_BINARY_DIR}/include")

qi_create_lib(metapod
  ${${PROJECT_NAME}_ALGOS_HEADERS}
  ${${PROJECT_NAME}_TOOLS_HEADERS})
set_target_properties(metapod PROPERTIES LINKER_LANGUAGE CXX)
qi_stage_header_only_lib(metapod)

add_subdirectory(include/metapod/models/simple-humanoid)
add_subdirectory(include/metapod/models/simple-arm)
add_subdirectory(tests)

option(BENCHMARK "Build the benchmarking tool" OFF)
if (NOT CMAKE_CROSSCOMPILING)
  set(BENCHMARK ON)
endif()
if(${BENCHMARK})
  add_subdirectory(benchmark)
endif(${BENCHMARK})
