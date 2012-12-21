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
FUNCTION(GET_SAMPLE_SOURCE_FILES output dof include_config_hh)
    # Generate config.hh using the same template as JRL-cmake
    SET(LIBRARY_NAME "SAMPLE_${dof}_DOF")
    SET(EXPORT_SYMBOL "sample_${dof}_dof_EXPORTS")
    CONFIGURE_FILE(
        ${PROJECT_SOURCE_DIR}/include/metapod/config.in.hh
        ${CMAKE_CURRENT_BINARY_DIR}/models/sample_${dof}_dof/config.hh
        @ONLY
        )
    SET(_output
        ${CMAKE_CURRENT_BINARY_DIR}/models/sample_${dof}_dof/sample_${dof}_dof.hh
        ${CMAKE_CURRENT_BINARY_DIR}/models/sample_${dof}_dof/sample_${dof}_dof.cc
        ${CMAKE_CURRENT_BINARY_DIR}/models/sample_${dof}_dof/body.hh
        ${CMAKE_CURRENT_BINARY_DIR}/models/sample_${dof}_dof/joint.hh
        ${CMAKE_CURRENT_BINARY_DIR}/models/sample_${dof}_dof/robot.hh)
    IF (include_config_hh)
        LIST(APPEND _output
            ${CMAKE_CURRENT_BINARY_DIR}/models/sample_${dof}_dof/config.hh)
    ENDIF()
    SET("${output}" ${_output} PARENT_SCOPE)
ENDFUNCTION()

# list of the sample models
SET(dof_list 3 7 15 31 63)

# put together a list of all the generated files
FOREACH(dof IN LISTS dof_list)
    GET_SAMPLE_SOURCE_FILES(_sources ${dof} False)
    LIST(APPEND _generated_files "${_sources}")
ENDFOREACH()

# build the generator
qi_create_bin(make_model make_model.cc make_model.hh)
qi_use_lib(make_model boost_filesystem)

# declare the generated files
IF(dof_list)
    ADD_CUSTOM_COMMAND(
      OUTPUT ${_generated_files}
      COMMAND make_model
    )
ENDIF()

# build the sample models libraries
FOREACH(dof IN LISTS dof_list)
    GET_SAMPLE_SOURCE_FILES(_sources ${dof} True)
    SET(_target sample_${dof}_dof)
    qi_create_lib(${_target} SHARED ${_sources})
    qi_stage_lib(${_target})
    LIST(APPEND _generated_libs "${_target}")
ENDFOREACH()

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

# build the timer library
# Generate config.hh using the same template as JRL-cmake
SET(LIBRARY_NAME "METAPODTIMER")
SET(EXPORT_SYMBOL "metapodtimer_EXPORTS")
CONFIGURE_FILE(
    ${PROJECT_SOURCE_DIR}/include/metapod/config.in.hh
    ${CMAKE_CURRENT_BINARY_DIR}/config.hh
    @ONLY
    )
QI_CREATE_LIB(metapodtimer
    timer.cc
    timer.hh
    ${CMAKE_CURRENT_BINARY_DIR}/config.hh)
# BOOST_TIMER was introduced in boost 1.48
QI_ADD_OPTIONAL_PACKAGE(BOOST_TIMER
    "Use the new boost timer library (requires boost >= 1.48)")
IF(WITH_BOOST_TIMER)
    QI_USE_LIB(metapodtimer BOOST_TIMER BOOST_CHRONO BOOST_SYSTEM)
ENDIF()
QI_STAGE_LIB(metapodtimer)

# and finally, the benchmark
qi_create_bin(benchmark benchmark.cc benchmark.hh timer.hh)
qi_use_lib(benchmark metapod metapodtimer ${_generated_libs} simple_humanoid)
