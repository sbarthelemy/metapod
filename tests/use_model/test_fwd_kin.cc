// Copyright 2011, 2012, 2013
//
// Maxime Reis (JRL/LAAS, CNRS/AIST)
// Sébastien Barthélémy (Aldebaran Robotics)
//
// This file is part of metapod.
// metapod is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// metapod is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with metapod.  If not, see <http://www.gnu.org/licenses/>.

/*
 * This test solves the forward kinematics problem on a test model with a
 * reference configuration, then compares the computed transforms with the
 * reference ones
 */

// Common test tools
#include "common.hh"
#include <metapod/tools/jcalc.hh>
#include <metapod/algos/fwd_kin.hh>

using namespace metapod;

BOOST_AUTO_TEST_CASE (test_fwd_kin)
{
  // Set configuration vectors (q) to reference values.
  CURRENT_MODEL_ROBOT::confVector q;
  std::ifstream qconf(TEST_DIRECTORY "/q.conf");
  initConf<CURRENT_MODEL_ROBOT>::run(qconf, q);
  qconf.close();

  // Apply the body calculations to the metapod multibody and print
  // the result in a log file.
  CURRENT_MODEL_ROBOT robot;
  metapod::Spatial::Transform iX1[CURRENT_MODEL_ROBOT::NBBODIES];
  // The first element is normaly never used (read nor written). Let init it
  // though.
  iX1[0] = metapod::Spatial::Transform::Identity();
  typedef Eigen::Matrix<metapod::FloatType, 6, CURRENT_MODEL_ROBOT::NBDOF> Jacobian;
  Jacobian J = Jacobian::Zero();
  Eigen::Matrix<metapod::FloatType, 3, 1> mass_com;
  metapod::jcalc_pos<CURRENT_MODEL_ROBOT>::run(robot, q);
  metapod::fwd_kin_iX1<CURRENT_MODEL_ROBOT>::run(robot, iX1, J, mass_com);
  metapod::fwd_kin_iX0_from_iX1<CURRENT_MODEL_ROBOT>::run(robot, iX1);
  const char result_file[] = "bcalc.log";
  std::ofstream log(result_file, std::ofstream::out);
  printTransforms<CURRENT_MODEL_ROBOT>(robot, log);
  log.close();

  // Compare results with reference file
  compareLogs(result_file, TEST_DIRECTORY "/bcalc.ref", 1e-3);
}