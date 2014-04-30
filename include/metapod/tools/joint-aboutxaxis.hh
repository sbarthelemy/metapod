// Copyright 2013
//
// Maxime Reis (JRL/CNRS/AIST, LAAS/CNRS)
// Sébastien Barthélémy (Aldebaran Robotics)
// Olivier Stasse (LAAS/CNRS)
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

#ifndef METAPOD_JOINT_ABOUT_X_AXIS_HH
# define METAPOD_JOINT_ABOUT_X_AXIS_HH
#include <metapod/tools/common.hh>

namespace metapod
{
  class RevoluteAxisXJoint
  {
  public:
    RevoluteAxisXJoint();
    static const int NBDOF = 1;
    Spatial::TransformX Xj;
    Spatial::Motion cj; // used in rnea
    Spatial::Motion vj; // used in rnea
    Spatial::ConstraintMotionOneAxis<Spatial::AxisX> S;
    Spatial::Force f; // used by rnea

    void bcalc(const Vector1d& qi);
    void jcalc(const Vector1d& qi, const Vector1d& dqi);
  };

  inline RevoluteAxisXJoint::RevoluteAxisXJoint():
    cj(Spatial::Motion::Zero())
  {
    vj.v(Vector3d(0.0,0.0,0.0));
  }

  inline void RevoluteAxisXJoint::bcalc(const Vector1d & qi)
  {
    const FloatType angle = qi[0];
    FloatType c = cos(angle), s = sin(angle);
    Xj = Spatial::TransformX(Spatial::RotationMatrixAboutX(c,s),
				 Vector3d::Zero());
  }

  inline void RevoluteAxisXJoint::jcalc(const Vector1d & qi,
                                   const Vector1d & dqi)
  {
    bcalc(qi);
    vj.w(Vector3d(dqi[0], 0, 0));
  }
}
#endif

