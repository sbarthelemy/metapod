// Copyright 2011, 2012,
//
// Maxime Reis
//
// JRL/LAAS, CNRS/AIST
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
 * This test applies the crba on a test model with a reference configuration,
 * then compares the computed inertia matrix with the reference inertia matrix.
 */

#ifndef METAPOD_TEST_CRBA_HH
# define METAPOD_TEST_CRBA_HH

// Common test tools
# include "common.hh"
# include "metapod/algos/crba.hh"
# include "metapod/algos/rnea.hh"

using namespace simplehumanoid;
typedef Eigen::Matrix< FloatType, Robot::nbDof, 1 > confVector;

BOOST_AUTO_TEST_CASE (test_crba)
{
  // set configuration vector
  vectorN q(Robot::nbDof);
  q = vectorN::Random(Robot::nbDof);

//  model.display(S_);
  std::ofstream log("H.log", std::ofstream::out);
  rnea< Robot::Tree, confVector, false >::run(q, q, q);
  Robot::H = matrixN::Zero(Robot::NBDOF, Robot::NBDOF);
  crba<Robot>(q);
  log << Robot::H << std::endl;

  log.close();

  // Perf test
# ifdef METAPOD_PERF_TEST
  long TICKS_PER_SECOND = 1e6;
  struct timeval tv_start, tv_stop;
  int N1 = 1000;
  int N2 = 100;

  std::ofstream perf_log("crba_perf.log", std::ofstream::out);

  long time_usec = 0;
  long inner_loop_time;
  // Outer loop : generate random configuration
  for(int i=0; i<N1; i++)
  {
    q = vectorN::Random(Robot::nbDof);
    ::gettimeofday(&tv_start, NULL);
    // Inner loop : The timer precision is 1µs, which is not high enough to
    // give proper result on a single iteration 
    for(int k=0; k<N2; k++)
    {
/*
      model.updateKinematics(q);
      CRBA(&model, q);
// */
      crba<Robot>(q);
    }
    ::gettimeofday(&tv_stop, NULL);
    
    inner_loop_time = ( tv_stop.tv_sec - tv_start.tv_sec ) * TICKS_PER_SECOND
               + ( tv_stop.tv_usec - tv_start.tv_usec );
    time_usec += inner_loop_time;
    // Log inner_loop_time to allow for statistical computations 
    perf_log << (double)inner_loop_time/(double)N2 << std::endl;
  }
  // Output global average execution time
  std::cout
    << "CRBA execution time = " << (double)time_usec/(double)(N1*N2) << "µs"
    << std::endl;
  perf_log.close();
# endif
}

#endif
