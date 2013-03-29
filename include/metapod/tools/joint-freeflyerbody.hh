// Copyright 2013
//
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

#ifndef METAPOD_JOINT_FREE_FLYER_BODY_HH
# define METAPOD_JOINT_FREE_FLYER_BODY_HH
#include <metapod/tools/common.hh>

namespace metapod
{
namespace Spatial
{
// Class of motion constraint with a free flyer.
class ConstraintMotionFreeFlyerBody
{
public:
  ConstraintMotionFreeFlyerBody(): m_S()
  {
    m_S << Matrix3d::Zero(), Matrix3d::Identity(),
           Matrix3d::Identity(), Matrix3d::Zero();
  }

  Matrix6d operator*(FloatType x) const
  {
    return x*m_S;
  }

  Vector6d operator*(const Eigen::Matrix< FloatType, 6, 1 > &ddqi) const
  {
    Vector6d r = static_cast<Vector6d>(m_S *ddqi);
    return r;
  }

  const Matrix6d & S() const { return m_S; }
  Matrix6d transpose() const { return m_S.transpose(); }

private:
  Matrix6d m_S;
};

inline Matrix6d operator*(const Inertia &m,
                          const ConstraintMotionFreeFlyerBody &a) {
  Matrix6d r;
  r.block<3,3>(0,0) = skew(m.h())*a.S().block<3,3>(3,0);
  r.block<3,3>(0,3) = m.I()*static_cast<Matrix3d>(a.S().block<3,3>(0,3));
  r.block<3,3>(3,0) = m.m()*a.S().block<3,3>(3,0);
  r.block<3,3>(3,3) = -skew(m.h())*a.S().block<3,3>(0,3);
  return r;
}
} // End of spatial namespace

/// a free flyer joint, whose generalized position parameterization
/// is the same as FreeFlyerJoint one (3 translations and 3 Euler angles)
/// but whose generalized velocity parameterization is the body twist
/// expressed in the moving frame (aka the body frame). It follows
/// ALMath convention and not metapod::Spatial convention for the ordering of
/// the components: translational velocities appear before rotational
/// velocities.
///
///   qi: tx, ty, tz, rx, ry, rz <-- euler angles
///   dqi: vx, vy, vz, wx, wy, wz  <-- body twist
///
/// Note that dqi is not qi time derivative!
///
/// Thanks to that velocity parametrization, the S matrix is constant.
class FreeFlyerBodyJoint
{
public:
  FreeFlyerBodyJoint();
  static const int NBDOF = 6;
  Spatial::Transform Xj;
  Spatial::Motion cj; // used in rnea
  Spatial::Motion vj; // used in rnea
  Spatial::ConstraintMotionFreeFlyerBody S;
  Spatial::Force f; // used by rnea
  Vector6d torque; // used by rnea

  void bcalc(const Vector6d& qi);
  void jcalc(const Vector6d& qi, const Vector6d& dqi);
};

inline FreeFlyerBodyJoint::FreeFlyerBodyJoint() : cj(Spatial::Motion::Zero())
{
}

inline void FreeFlyerBodyJoint::bcalc(const Vector6d& qi)
{
  FloatType cPsi   = cos(qi(3)), sPsi   = sin(qi(3)),
            cTheta = cos(qi(4)), sTheta = sin(qi(4)),
            cPhi   = cos(qi(5)), sPhi   = sin(qi(5));
  // localR = rx(Psi) * ry(Theta) * rz(Phi)
  Matrix3d localR;
  localR(0,0) = cTheta * cPhi;
  localR(0,1) = cTheta * sPhi;
  localR(0,2) = -sTheta;
  localR(1,0) = -cPsi * sPhi + cPhi * sPsi * sTheta;
  localR(1,1) = cPsi * cPhi + sPsi * sTheta * sPhi;
  localR(1,2) = cTheta * sPsi;
  localR(2,0) = cPsi * cPhi * sTheta + sPhi * sPsi;
  localR(2,1) = -cPhi * sPsi + cPsi * sTheta * sPhi;
  localR(2,2) = cPsi * cTheta;
  Xj = Spatial::Transform(localR, qi.segment<3>(0));
}

inline void FreeFlyerBodyJoint::jcalc(const Vector6d& qi,
                                      const Vector6d& dqi)
{
  bcalc(qi);
  vj = Spatial::Motion(S.S()*dqi);
}
}
#endif /* METAPOD_JOINT_FREE_FLYER_BODY_HH */
