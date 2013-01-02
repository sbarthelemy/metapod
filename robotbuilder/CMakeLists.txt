# Copyright 2012, 2013
#
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

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

GENERATE_CONFIG_HEADER(
  ${CMAKE_CURRENT_BINARY_DIR}/metapod/robotbuilder/config.hh
  metapod_robotbuilder
  )
SET(METAPOD_ROBOTBUILDER_HEADERS
  ${CMAKE_CURRENT_BINARY_DIR}/metapod/robotbuilder/config.hh
  include/metapod/robotbuilder/robotbuilder.hh
  )
ADD_CUSTOM_COMMAND(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/config_hh.c
  COMMAND embedfile ${CMAKE_CURRENT_BINARY_DIR}/config_hh.c ${CMAKE_CURRENT_SOURCE_DIR}/tpl/config.in.hh config_hh
  )
INSTALL(FILES ${METAPOD_ROBOTBUILDER_MODEL_HEADERS}
  DESTINATION include/metapod/robotbuilder
  )
ADD_LIBRARY(metapod_robotbuilder SHARED
  src/robotbuilder.cc
  ${CMAKE_CURRENT_BINARY_DIR}/config_hh.c
  ${METAPOD_ROBOTBUILDER_HEADERS}
  )
TARGET_LINK_LIBRARIES(metapod_robotbuilder ${Boost_LIBRARIES})
