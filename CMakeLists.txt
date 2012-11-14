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

# generate config.hh using the same template as JRL-cmake
set(LIBRARY_NAME "METAPOD")
set(PROJECT_VERSION "999")
set(EXPORT_SYMBOL "metapod_EXPORTS")
configure_file(
  include/metapod/config.in.hh
  ${CMAKE_CURRENT_BINARY_DIR}/include/metapod/config.hh
  @ONLY)

# Set header files.
set(${PROJECT_NAME}_SIMPLE_HUMANOID_MODEL_HEADERS
  include/${PROJECT_NAME}/models/simple-humanoid/body.hh
  include/${PROJECT_NAME}/models/simple-humanoid/joint.hh
  include/${PROJECT_NAME}/models/simple-humanoid/robot.hh
  )

set(${PROJECT_NAME}_SIMPLE_ARM_MODEL_HEADERS
  include/${PROJECT_NAME}/models/simple-arm/body.hh
  include/${PROJECT_NAME}/models/simple-arm/joint.hh
  include/${PROJECT_NAME}/models/simple-arm/robot.hh
  )

set(${PROJECT_NAME}_ALGOS_HEADERS
  include/${PROJECT_NAME}/algos/rnea.hh
  include/${PROJECT_NAME}/algos/crba.hh
  include/${PROJECT_NAME}/algos/jac_point.hh
  include/${PROJECT_NAME}/algos/jac_point_chain.hh
  )

set(${PROJECT_NAME}_TOOLS_HEADERS
  include/${PROJECT_NAME}/tools/fwd.hh
  include/${PROJECT_NAME}/tools/common.hh
  include/${PROJECT_NAME}/tools/spatial.hh
  include/${PROJECT_NAME}/tools/jointmacros.hh
  include/${PROJECT_NAME}/tools/bodymacros.hh
  include/${PROJECT_NAME}/tools/initconf.hh
  include/${PROJECT_NAME}/tools/print.hh
  include/${PROJECT_NAME}/tools/jcalc.hh
  include/${PROJECT_NAME}/tools/bcalc.hh
  include/${PROJECT_NAME}/tools/deepest_common_body.hh
  )

# prefix all the elements of a list, in place.
function(prefix_list a_list_name a_prefix)
  foreach(element IN LISTS "${a_list_name}")
    list(APPEND output "${a_prefix}${element}")
  endforeach()
  set("${a_list_name}" "${output}" PARENT_SCOPE)
endfunction()

# prefix the sources. We could have written the prefixes directly, but it would
# have complicated merges with the upstream CMakeLists.txt
foreach(list_suffix
    SIMPLE_HUMANOID_MODEL_HEADERS
    SIMPLE_ARM_MODEL_HEADERS
    ALGOS_HEADERS
    TOOLS_HEADERS)
  prefix_list("${PROJECT_NAME}_${list_suffix}" "${PROJECT_SOURCE_DIR}/")
endforeach()
list(APPEND
  "${PROJECT_NAME}_TOOLS_HEADERS"
  "${CMAKE_CURRENT_BINARY_DIR}/include/metapod/config.hh")

include_directories("include" "${CMAKE_CURRENT_BINARY_DIR}/include")
qi_create_lib(metapod
  ${${PROJECT_NAME}_ALGOS_HEADERS}
  ${${PROJECT_NAME}_TOOLS_HEADERS})
set_target_properties(metapod PROPERTIES LINKER_LANGUAGE CXX)
qi_stage_header_only_lib(metapod)
add_subdirectory(tests)

option(BENCHMARK "Build the benchmarking tool" ON)
if(${BENCHMARK})
  add_subdirectory(benchmark)
endif(${BENCHMARK})

