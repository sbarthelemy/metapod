# Copyright 2012, 2013,
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

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}/include)

GENERATE_CONFIG_HEADER(
  ${CMAKE_CURRENT_BINARY_DIR}/include/metapod/timer/config.hh
  metapod_timer
  )
SET(METAPOD_TIMER_HEADERS
  ${CMAKE_CURRENT_BINARY_DIR}/include/metapod/timer/config.hh
  include/metapod/timer/timer.hh
  )
QI_CREATE_LIB(metapod_timer SHARED
  src/timer.cc
  ${METAPOD_TIMER_HEADERS}
  )

QI_USE_LIB(metapod_timer BOOST)
# BOOST_TIMER was introduced in boost 1.48
QI_ADD_OPTIONAL_PACKAGE(BOOST_TIMER
  "Use the new boost timer library (requires boost >= 1.48)")
IF(WITH_BOOST_TIMER)
  SET_SOURCE_FILES_PROPERTIES("src/timer.cc" PROPERTIES
      COMPILE_FLAGS " -DWITH_BOOST_TIMER")
  QI_USE_LIB(metapod_timer BOOST_TIMER)
ENDIF()
QI_STAGE_LIB(metapod_timer)
