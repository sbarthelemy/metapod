# Copyright 2014
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
# GNU Lesser General Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with metapod.  If not, see <http://www.gnu.org/licenses/>.

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}/include)

GENERATE_CONFIG_HEADER(
  ${CMAKE_CURRENT_BINARY_DIR}/include/metapod/graph/config.hh
  metapod_graph
  )
SET(METAPOD_GRAPH_HEADERS
  ${CMAKE_CURRENT_BINARY_DIR}/include/metapod/robotbuilder/config.hh
  include/metapod/graph.hh
  )

QI_CREATE_LIB(metapod_graph SHARED
  src/graph.cc
  ${METAPOD_GRAPH_HEADERS}
  )
QI_USE_LIB(metapod_graph boost eigen3)
QI_STAGE_LIB(metapod_graph)
QI_INSTALL_HEADER(${METAPOD_GRAPH_HEADERS}
  SUBFOLDER metapod)

QI_CREATE_GTEST(test_graph
  SRC tests/test_graph.cpp
  DEPENDS metapod_graph)
