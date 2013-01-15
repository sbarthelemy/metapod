# Copyright 2011, 2012, 2013
#
# Maxime Reis (JRL/LAAS, CNRS/AIST)
# Sébastien Barthélémy (Aldebaran Robotics)
# Olivier Stasse (JRL/LAAS, CNRS/AIST)
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


# Add Boost path to include directories.
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_MAIN)
# note: the following line is needed if we link with the dynamic version
#       of libboost_test_exec_monitor
#ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK)

# ADD_TESTCASE(NAME)
# ------------------------
#
# simple wrapper to make merging with upstream CMakeLists easier
FUNCTION(ADD_TESTCASE test_name source_file)
  QI_CREATE_TEST(${test_name} ${source_file}
      DEPENDS BOOST BOOST_TEST_EXEC_MONITOR)
ENDFUNCTION()

# the "use_model" subfolder is included once per tested model. The tests
# defined within it should use the "CURRENT_MODEL" macro to use the
# corresponding solver.
# For each inclusion of the folder, a different binary folder is used. It acts
# as a namespace for CMake. This makes it possible to use different values
# for "CURRENT_MODEL" when calling add_definitions(), without conflicts.
IF(${WITH_SIMPLE_HUMANOID})
  SET(CURRENT_MODEL "simple_humanoid")
  ADD_SUBDIRECTORY(use_model use_simple_humanoid)
ENDIF()
IF(${WITH_SIMPLE_ARM})
  SET(CURRENT_MODEL "simple_arm")
  ADD_SUBDIRECTORY(use_model use_simple_arm)
ENDIF()
ADD_SUBDIRECTORY(spatial)
