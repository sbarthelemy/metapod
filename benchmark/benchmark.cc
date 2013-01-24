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
 * This file run performance tests on metapods algorithms, on several sample
 * models.
 */

#define FUSION_MAX_VECTOR_SIZE 50
//#ifdef WITH_SIMPLE_HUMANOID
//# include "metapod/models/simple_humanoid/simple_humanoid.hh"
//#endif
#define S1_STATIC
#define S2_STATIC
#define S3_STATIC
#define S4_STATIC
#include "models/s1/s1.hh"
#include "models/s2/s2.hh"
#include "models/s3/s3.hh"
#include "models/s4/s4.hh"
//#include "models/sample_5/sample_5.hh"

#include "benchmark.hh"
using namespace metapod::benchmark;

int main()
{
#ifdef WITH_SIMPLE_HUMANOID
  //BENCHMARK(metapod::simple_humanoid);
#endif
  BENCHMARK(metapod::s1);
  BENCHMARK(metapod::s2);
  BENCHMARK(metapod::s3);
  BENCHMARK(metapod::s4);
  //BENCHMARK(metapod::sample_5);
}
