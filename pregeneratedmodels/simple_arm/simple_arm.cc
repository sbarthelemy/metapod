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


// This file has been generated by the metapod robotbuilder library.

#ifdef _MSC_VER
# pragma warning( push )
// The following warnings are only needed if the FloatType is float, because
// the code generator uses double anyway.
// disable warning C4305 truncation from 'double' to 'float'
// disable warning C4244 conversion from 'double' to 'float', possible loss of data
# pragma warning( disable: 4305 4244)
#endif

#include "simple_arm.hh"

namespace metapod {

const std::string simple_arm::Node0::joint_name = std::string("SHOULDER");
const std::string simple_arm::Node0::body_name = std::string("ARM");
const Spatial::Transform simple_arm::Node0::Xt = Spatial::Transform(
    matrix3dMaker(2.22045e-16, 0, 1, 0, 1, 0, -1, 0, 2.22045e-16),
    Vector3d(0, 0, 1));
simple_arm::Node0::Node0():
  joint(1, 0, 0) {}

const std::string simple_arm::Node1::joint_name = std::string("ELBOW");
const std::string simple_arm::Node1::body_name = std::string("FOREARM");
const Spatial::TransformT<Spatial::RotationMatrixIdentity> simple_arm::Node1::Xt = Spatial::TransformT<Spatial::RotationMatrixIdentity>(
    Spatial::RotationMatrixIdentity(),
    Vector3d(0, 0, -0.45));
simple_arm::Node1::Node1():
  joint(1, 0, 0) {}

const std::string simple_arm::Node2::joint_name = std::string("WRIST");
const std::string simple_arm::Node2::body_name = std::string("HAND");
const Spatial::TransformT<Spatial::RotationMatrixIdentity> simple_arm::Node2::Xt = Spatial::TransformT<Spatial::RotationMatrixIdentity>(
    Spatial::RotationMatrixIdentity(),
    Vector3d(0, 0, -0.4));
simple_arm::Node2::Node2():
  joint(1, 0, 0) {}



Spatial::Inertia simple_arm::inertias[] = {
    spatialInertiaMaker(
        2.75,
        Vector3d(0, 0, -0.225),
        matrix3dMaker(0.0468703, 0, 0, 0, 0.0468703, 0, 0, 0, 0.000928125)),
    spatialInertiaMaker(
        1.75,
        Vector3d(0, 0, -0.2),
        matrix3dMaker(0.0235667, 0, 0, 0, 0.0235667, 0, 0, 0, 0.000466667)),
    spatialInertiaMaker(
        0.5,
        Vector3d(0, 0, -0.075),
        matrix3dMaker(0.000946875, 0, 0, 0, 0.000946875, 0, 0, 0, 1.875e-05)),
};
}

#ifdef _MSC_VER
# pragma warning( pop )
#endif

