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

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/benchmark)

# Returns the list of source files for sample_${dof}_dof model
FUNCTION(GET_SAMPLE_SOURCE_FILES output dof)
    SET(LIBRARY_NAME "SAMPLE_${dof}_DOF")
    SET(EXPORT_SYMBOL "sample_${dof}_dof")
    CONFIGURE_FILE(
        ${PROJECT_SOURCE_DIR}/cmake/config.hh.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/models/sample_${dof}_dof/config.hh
        @ONLY
        )
    SET("${output}"
        ${CMAKE_CURRENT_BINARY_DIR}/models/sample_${dof}_dof/sample_${dof}_dof.hh
        ${CMAKE_CURRENT_BINARY_DIR}/models/sample_${dof}_dof/sample_${dof}_dof.cc
        ${CMAKE_CURRENT_BINARY_DIR}/models/sample_${dof}_dof/body.hh
        ${CMAKE_CURRENT_BINARY_DIR}/models/sample_${dof}_dof/joint.hh
        ${CMAKE_CURRENT_BINARY_DIR}/models/sample_${dof}_dof/robot.hh
        ${CMAKE_CURRENT_BINARY_DIR}/models/sample_${dof}_dof/config.hh
        PARENT_SCOPE)
ENDFUNCTION()

# list of the sample models
SET(dof_list 3 7 15 31 63)

# put together a list of all the generated files
FOREACH(dof IN LISTS dof_list)
    GET_SAMPLE_SOURCE_FILES(_sources ${dof})
    LIST(APPEND _generated_files "${_sources}")
ENDFOREACH()

# build the generator
qi_create_bin(make_model make_model.cc make_model.hh)
qi_use_lib(make_model boost_filesystem)

# declare the generated files
ADD_CUSTOM_COMMAND(
  OUTPUT ${_generated_files}
  COMMAND make_model
  DEPENDS make_model
)

# build the sample models libraries
FOREACH(dof IN LISTS dof_list)
    GET_SAMPLE_SOURCE_FILES(_sources ${dof})
    SET(_target sample_${dof}_dof)
    qi_create_lib(${_target} SHARED ${_sources})
    qi_stage_lib(${_target})
    LIST(APPEND _generated_libs "${_target}")
ENDFOREACH()

# and finally, the benchmark
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
qi_create_bin(benchmark benchmark.cc)
qi_use_lib(benchmark metapod ${_generated_libs} simple_humanoid)
