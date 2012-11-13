# Copyright 2012,
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
add_definitions(" -DCURRENT_MODEL_NAMESPACE=metapod::${CURRENT_MODEL}")
add_definitions(
   -DTEST_DIRECTORY="${CMAKE_SOURCE_DIR}/tests/data/${CURRENT_MODEL}")

# simple wrapper to make merging with upstream CMakeLists easier
function(ADD_TESTCASE test_name source_file)
  qi_create_test(${test_name} ${source_file}
      DEPENDS ${CURRENT_MODEL} BOOST BOOST_TEST_EXEC_MONITOR)
endfunction()

ADD_TESTCASE(test_rnea_${CURRENT_MODEL} test_rnea.cc)
ADD_TESTCASE(test_crba_${CURRENT_MODEL} test_crba.cc)
ADD_TESTCASE(test_bcalc_${CURRENT_MODEL} test_bcalc.cc)
ADD_TESTCASE(test_jac_point_${CURRENT_MODEL} test_jac_point.cc)
ADD_TESTCASE(test_jac_point_chain_${CURRENT_MODEL} test_jac_point_chain.cc)
