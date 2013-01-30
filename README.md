metapod
========

This software provides robot dynamics algorithms.
It uses a combination of a specific representation of robot models and C++
templates, such that each algorithm remains model-independant, yet is optimized
for a particular robot at compile-time.
It makes use of R. Featherstone's Spatial Algebra to describe forces, motions
and inertias (cf. Rigid Body Dynamics Algorithms, Roy Featherstone).

The metapod project was initiated at JRL/LAAS, CNRS/AIST. This version
is a fork, used at Aldebaran Robotics.

While the upstream metapod uses jrl-cmake as a build system, the Aldebaran
Robotics fork uses
[qibuild](www.aldebaran-robotics.com/documentation/qibuild/index.html).

Dependencies
------------

The package depends on several packages which have to be available on
your machine.

 - System tools:
   - CMake (>=2.8)
   - usual compilation tools (GCC/G++, make, etc.)
   - qibuild
 - Libraries:
   - Eigen (>=3.0.0)
   - Boost (>=1.40.0)
     Boost Test is used in the test suite
   - optionally, liburdf, as provided by ROS

Setup
-----

Install qibuild then

    qc --release
    qm --release
