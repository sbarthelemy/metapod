// Copyright 2011, 2012,
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
 * This file defines the macros that create the joint classes and their
 * corresponding jcalc routine.
 */

#ifndef METAPOD_JOINT_MACROS_HH
# define METAPOD_JOINT_MACROS_HH

namespace metapod
{
  // Create a FIXED joint class
  # define JOINT_FIXED(classname)                                   \
    class classname                                                 \
    {                                                               \
      public:                                                       \
      enum { NBDOF = 0 };                                           \
      static const std::string name;                                \
      static const int label;                                       \
      static const int positionInConf;                              \
      static const Spatial::Transform Xt;                           \
      static const Spatial::Transform sXp;                          \
      /*static Spatial::Transform Xj;*/                             \
      static Spatial::Motion cj;                                    \
      static const Spatial::Motion vj;                              \
      static const Eigen::Matrix< FloatType, 6, 0 > S;              \
      static const Eigen::Matrix< FloatType, 6, 0 > dotS;           \
      static Spatial::Force f;                                      \
      static Eigen::Matrix< FloatType, 0, 1 > torque;               \
      static Eigen::Matrix< FloatType, 6, 0 > F;                    \
                                                                    \
      static void jcalc(                                            \
          const Eigen::Matrix< FloatType, 0, 1 > & qi,              \
          const Eigen::Matrix< FloatType, 0, 1 > & dqi);            \
    };                                                              \
    inline void classname::jcalc(                                   \
        const Eigen::Matrix< FloatType, 0, 1 > & qi,                \
        const Eigen::Matrix< FloatType, 0, 1 > & dqi)               \
    {                                                               \
    }                                                               \
    struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n

  # define INITIALIZE_JOINT_FIXED(classname)                        \
    /* sXp = Xj*Xt; */                                              \
    const Spatial::Transform classname::sXp = classname::Xt;        \
    /*Spatial::Transform classname::Xj = Spatial::Transform();*/    \
    Spatial::Motion classname::cj;                                  \
    const Spatial::Motion classname::vj;                            \
    /* const Spatial::Motion classname::vj = vector6d::Zero(); */   \
    const Eigen::Matrix< FloatType, 6, 0 > classname::S;            \
    const Eigen::Matrix< FloatType, 6, 0 > classname::dotS;         \
    Spatial::Force classname::f;                                    \
    Eigen::Matrix< FloatType, 0, 1 > classname::torque;             \
    Eigen::Matrix< FloatType, 6, 0 > classname::F                                           \

  // Create a REVOLUTE_AXIS_ANY joint class
  # define JOINT_REVOLUTE_AXIS_ANY(classname, axisx, axisy, axisz)  \
    class classname                                                 \
    {                                                               \
      public:                                                       \
      enum { NBDOF = 1 };                                           \
      static const std::string name;                                \
      static const int label;                                       \
      static const int positionInConf;                              \
      static const Spatial::Transform Xt;                           \
      static Spatial::Transform sXp;                                \
      static Spatial::Transform Xj;                                 \
      static Spatial::Motion cj;                                    \
      static Spatial::Motion vj;                                    \
      static const vector6d S;                                      \
      static const vector6d dotS;                                   \
      static Spatial::Force f;                                      \
      static vector1d torque;                                       \
      static vector6d F;                                            \
                                                                    \
      static void jcalc(const vector1d & qi, const vector1d & dqi); \
    };                                                              \
    inline void classname::jcalc(const vector1d & qi,               \
                          const vector1d & dqi)                     \
    {                                                               \
      FloatType angle = qi[0];                                      \
      matrix3d localR;                                              \
      localR =                                                      \
          Eigen::AngleAxisd(-angle, vector3d(axisx, axisy, axisz)); \
      Xj = Spatial::Transform(localR, vector3d::Zero());            \
      sXp = Xj*Xt;                                                  \
                                                                    \
      /* maj vj */                                                  \
      vj.w(vector3d(dqi[0], 0, 0));                                 \
    }                                                               \
    struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n

  # define INITIALIZE_JOINT_REVOLUTE_AXIS_ANY(classname, axisx, axisy, axisz)\
    Spatial::Transform classname::sXp;                              \
    Spatial::Transform classname::Xj;                               \
    Spatial::Motion classname::cj;                                  \
    Spatial::Motion classname::vj;                                  \
    Spatial::Force classname::f;                                    \
    vector1d classname::torque;                                     \
    const vector6d classname::S =                                   \
        vector6dMaker(axisx, axisy, axisz, 0, 0, 0);                \
    const vector6d classname::dotS = vector6d::Zero();              \
    vector6d classname::F                                           \

  // Create a REVOLUTE_AXIS_X joint class
  # define JOINT_REVOLUTE_AXIS_X(classname)                         \
    class classname                                                 \
    {                                                               \
      public:                                                       \
      enum { NBDOF = 1 };                                           \
      static const std::string name;                                \
      static const int label;                                       \
      static const int positionInConf;                              \
      static const Spatial::Transform Xt;                           \
      static Spatial::Transform sXp;                                \
      static Spatial::Transform Xj;                                 \
      static Spatial::Motion cj;                                    \
      static Spatial::Motion vj;                                    \
      static const vector6d S;                                      \
      static const vector6d dotS;                                   \
      static Spatial::Force f;                                      \
      static vector1d torque;                                       \
      static vector6d F;                                            \
                                                                    \
      static void jcalc(const vector1d & qi, const vector1d & dqi); \
    };                                                              \
    inline void classname::jcalc(const vector1d & qi,               \
                          const vector1d & dqi)                     \
    {                                                               \
      FloatType angle = qi[0];                                      \
      matrix3d localR;                                              \
      FloatType c = cos(angle), s = sin(angle);                     \
      localR(0,0) =  1; localR(0,1) =  0; localR(0,2) =  0;         \
      localR(1,0) =  0; localR(1,1) =  c; localR(1,2) =  s;         \
      localR(2,0) =  0; localR(2,1) = -s; localR(2,2) =  c;         \
      Xj = Spatial::Transform(localR, vector3d::Zero());            \
      sXp = Xj*Xt;                                                  \
                                                                    \
      /* maj vj */                                                  \
      vj.w(vector3d(dqi[0], 0, 0));                                 \
    }                                                               \
    struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n

  # define INITIALIZE_JOINT_REVOLUTE_AXIS_X(classname)              \
    Spatial::Transform classname::sXp;                              \
    Spatial::Transform classname::Xj;                               \
    Spatial::Motion classname::cj;                                  \
    Spatial::Motion classname::vj;                                  \
    Spatial::Force classname::f;                                    \
    vector1d classname::torque;                                     \
    const vector6d classname::S = vector6dMaker(1, 0, 0, 0, 0, 0);  \
    const vector6d classname::dotS = vector6d::Zero();              \
    vector6d classname::F                                           \

  // Create a free flyer class
  # define JOINT_FREE_FLYER(classname)                              \
    class classname                                                 \
    {                                                               \
      public:                                                       \
      enum { NBDOF = 6 };                                           \
      static const std::string name;                                \
      static const int label;                                       \
      static const int nbDof;                                       \
      static const int positionInConf;                              \
      static const Spatial::Transform Xt;                           \
      static Spatial::Transform sXp;                                \
      static Spatial::Transform Xj;                                 \
      static Spatial::Motion cj;                                    \
      static Spatial::Motion vj;                                    \
      static matrix6d S;                                            \
      static matrix6d dotS;                                         \
      static Spatial::Force f;                                      \
      static vector6d torque;                                       \
      static matrix6d F;                                            \
                                                                    \
      static void jcalc(const vector6d & qi, const vector6d & dqi); \
    };                                                              \
    inline void classname::jcalc(const vector6d & qi,               \
                          const vector6d & dqi)                     \
    {                                                               \
      /* maj sXp */                                                 \
      matrix3d localR;                                              \
      FloatType cPsi   = cos(qi(3)), sPsi   = sin(qi(3)),           \
                cTheta = cos(qi(4)), sTheta = sin(qi(4)),           \
                cPhi   = cos(qi(5)), sPhi   = sin(qi(5));           \
      localR(0,0) = cTheta * cPhi;                                  \
      localR(0,1) = cTheta * sPhi;                                  \
      localR(0,2) = -sTheta;                                        \
      localR(1,0) = -cPsi * sPhi + cPhi * sPsi * sTheta;            \
      localR(1,1) = cPsi * cPhi + sPsi * sTheta * sPhi;             \
      localR(1,2) = cTheta * sPsi;                                  \
      localR(2,0) = cPsi * cPhi * sTheta + sPhi * sPsi;             \
      localR(2,1) = -cPhi * sPsi + cPsi * sTheta * sPhi;            \
      localR(2,2) = cPsi * cTheta;                                  \
      S.block<3,3>(0,3) = S.block<3,3>(3,0) = localR;               \
      Xj = Spatial::Transform(localR, vector3d::Zero());            \
      sXp = Xj*Spatial::Transform(matrix3d::Identity(), qi.segment<3>(0)); \
      /* maj vj */                                                  \
      vj = Spatial::Motion(S*dqi);                                  \
    }                                                               \
    struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n

  # define INITIALIZE_JOINT_FREE_FLYER(classname)                   \
    Spatial::Transform classname::sXp;                              \
    Spatial::Transform classname::Xj;                               \
    Spatial::Motion classname::cj;                                  \
    Spatial::Motion classname::vj;                                  \
    Spatial::Force classname::f;                                    \
    vector6d classname::torque;                                     \
    matrix6d classname::S = matrix6d::Zero();                       \
    matrix6d classname::dotS = matrix6d::Zero();                    \
    matrix6d classname::F                                           \

} // end of namespace metapod.

#endif
