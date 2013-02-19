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
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

# list of the sample models
SET(depth_list 1 2 3 4 )
#SET(depth_list 1 )

# build the generator
QI_CREATE_BIN(make_model make_model.cc)
QI_USE_LIB(make_model metapod_robotbuilder)

# build the sample models libraries
FOREACH(depth IN LISTS depth_list)
    SET(_libname "sample_${depth}")
    SET(_sources
        ${CMAKE_CURRENT_BINARY_DIR}/models/${_libname}/${_libname}.hh
        ${CMAKE_CURRENT_BINARY_DIR}/models/${_libname}/${_libname}.cc
        ${CMAKE_CURRENT_BINARY_DIR}/models/${_libname}/config.hh)
    ADD_CUSTOM_COMMAND(
      OUTPUT ${_sources}
      COMMAND make_model ${depth}
      DEPENDS make_model
    )
    QI_CREATE_LIB(${_libname} SHARED ${_sources})
    QI_USE_LIB(${_libname} metapod)
    QI_STAGE_LIB(${_libname})
    LIST(APPEND _model_libs "${_libname}")
ENDFOREACH()

# add simple humanoid model
IF(WITH_SIMPLE_HUMANOID)
    SET_SOURCE_FILES_PROPERTIES("benchmark.cc" PROPERTIES
        COMPILE_FLAGS " -DWITH_SIMPLE_HUMANOID")
    LIST(APPEND _model_libs "metapod_simple_humanoid")
ENDIF()

# and finally, the benchmark
QI_CREATE_BIN(benchmark benchmark.cc benchmark.hh)
QI_USE_LIB(benchmark metapod_timer ${_model_libs})

QI_CREATE_BIN(benchmark_1 benchmark_1.cc benchmark.hh)
QI_USE_LIB(benchmark_1 metapod_timer ${_model_libs})

QI_CREATE_BIN(benchmark_2 benchmark_2.cc benchmark.hh)
QI_USE_LIB(benchmark_2 metapod_timer ${_model_libs})

QI_CREATE_BIN(benchmark_3 benchmark_3.cc benchmark.hh)
QI_USE_LIB(benchmark_3 metapod_timer ${_model_libs})

QI_CREATE_BIN(benchmark_4 benchmark_4.cc benchmark.hh)
QI_USE_LIB(benchmark_4 metapod_timer ${_model_libs})
