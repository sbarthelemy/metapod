# Copyright 2011, 2012,
#
# Maxime Reis (JRL/LAAS, CNRS/AIST)
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

qi_create_lib(simple_humanoid SHARED
    ${PROJECT_SOURCE_DIR}/include/metapod/models/simple-humanoid/simple_humanoid.cc
    ${${PROJECT_NAME}_SIMPLE_HUMANOID_MODEL_HEADERS})
qi_use_lib(simple_humanoid metapod EIGEN3)
qi_stage_lib(simple_humanoid)

qi_create_lib(simple_arm SHARED
    ${PROJECT_SOURCE_DIR}/include/metapod/models/simple-arm/simple_arm.cc
    ${${PROJECT_NAME}_SIMPLE_ARM_MODEL_HEADERS})
qi_use_lib(simple_arm metapod EIGEN3)
qi_stage_lib(simple_arm)

# simple wrapper to make merging with upstream CMakeLists easier
FUNCTION(ADD_TESTCASE test_name source_file)
  QI_CREATE_TEST(${test_name} ${source_file}
      DEPENDS simple_arm simple_humanoid BOOST BOOST_TEST_EXEC_MONITOR)
ENDFUNCTION()

# Add Boost path to include directories.
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# Make Boost.Test generates the main function in test cases.
add_definitions(-DBOOST_TEST_MAIN)
# note: the following line is needed if we link with the dynamic version
#       of libboost_test_exec_monitor
#add_definitions(-DBOOST_TEST_DYN_LINK)

# the "use_model" subfolder is included once per tested model. The tests
# defined within it should use the "CURRENT_MODEL" macro to use the
# corresponding solver.
# For each inclusion of the folder, a different binary folder is used. It acts
# as a namespace for CMake. This makes it possible to use different values
# for "CURRENT_MODEL" when calling add_definitions(), without conflicts.
set(CURRENT_MODEL "simple_humanoid")
add_subdirectory(use_model use_simple_humanoid)
set(CURRENT_MODEL "simple_arm")
add_subdirectory(use_model use_simple_arm)
add_subdirectory(spatial)
