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

# Make Boost.Test generate the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_MAIN -DBOOST_TEST_DYN_LINK)

# ADD_TESTCASE(NAME SOURCE_FILE1 [SOURCE_FILE2])
# ------------------------
#
# Define a test named `NAME'.
#
# This macro will
#
# * create a binary from user-provided source files and common.hh
# * link it against Boost
# * add it to the test suite.
#
# The test should #include the common.hh header
#
FUNCTION(ADD_TESTCASE NAME)
  QI_CREATE_TEST(${NAME}
    SRC ${ARGN} ${PROJECT_SOURCE_DIR}/tests/common.hh
    DEPENDS BOOST BOOST_UNIT_TEST_FRAMEWORK)
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
