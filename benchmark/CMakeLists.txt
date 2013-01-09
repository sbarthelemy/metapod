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

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

# list of the sample models
SET(depth_list 1 2 3 4 5)

# build the generator
qi_create_bin(make_model make_model.cc)
qi_use_lib(make_model metapod_robotbuilder)

# build the sample models libraries
FOREACH(depth IN LISTS depth_list)
    SET(_libname "sample_${depth}")
    SET(_sources
        ${CMAKE_CURRENT_BINARY_DIR}/models/${_libname}/${_libname}.hh
        ${CMAKE_CURRENT_BINARY_DIR}/models/${_libname}/${_libname}.cc
        ${CMAKE_CURRENT_BINARY_DIR}/models/${_libname}/body.hh
        ${CMAKE_CURRENT_BINARY_DIR}/models/${_libname}/joint.hh
        ${CMAKE_CURRENT_BINARY_DIR}/models/${_libname}/robot.hh
        ${CMAKE_CURRENT_BINARY_DIR}/models/${_libname}/config.hh)
    ADD_CUSTOM_COMMAND(
      OUTPUT ${_sources}
      COMMAND make_model ${depth}
    )
    QI_CREATE_LIB(${_libname} SHARED ${_sources})
    QI_STAGE_LIB(${_libname})
    LIST(APPEND _generated_libs "${_libname}")
ENDFOREACH()

# and finally, the benchmark
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})
QI_CREATE_BIN(benchmark benchmark.cc benchmark.hh)
QI_USE_LIB(benchmark metapod metapod_timer ${_generated_libs} simple_humanoid)
