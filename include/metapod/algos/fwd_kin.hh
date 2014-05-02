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

/*
 * Implementation of the forward kinematics routine.
 * This algorithm only works for models with a single root body.
 * It updates the bodies global transforms (iX0), the bodies transforms
 * relative to the root body (iX1), the "composite jacobian" (J) and the
 * mass*center of mass position, expressed in the root body frame (mass_com),
 * in two steps:
 *
 *  1. fwd_kin_iX1 computes iX1 for all the bodies but the first one (ie. the
 *     root one).
 *     It also computes the "composite" jacobian J, with the exception
 *     of the part relative to the first joint, which is constant anyway, and
 *     which the user is expected to set himself.
 *     It also computes the mass_com value.
 *  2. fwd_kin_iX0_from_iX1 updates 1X0 (it is sXp for the root joint) and uses
 *     it to compute iX0 for all the bodies.
 *
 * This algorithm is particularly useful for humanoid robots using the waist as
 * the root body, parametrized with a free floating joint. One could proceed as
 * follow:
 *
 * a. call jcal_pos on all joints but the root one (it updates sXp)
 * b. call fwd_kin_iX1
 * c. use sensor to know wich foot is in contact with the ground (let call it
 *    f).
 * b. update the free floating root joint from 1Xf:
 *    1X0 = Xt0 * Xj0 = 1Xf * fX0
 *    hence  Xj0 = Xt0.inv() * fX1.inv() * fX0
 * e. call jcalc_pos on the root joint
 * f. call fwd_kin_iX0_from_iX1
 *
 * The "composite jacobian" is such that each columns are basis twists of the
 * corresponding joint child body (relative to the joint parent body) expressed
 * in root body frame.
 * It follows from this definition that the first columns, corresponding to rhe
 * root joint are constant.
 * With knowledge of the kinematic tree, the matrix can be used to construct
 * other jacobians.
 *
 */

#ifndef METAPOD_FWD_KIN_HH
# define METAPOD_FWD_KIN_HH

#include <metapod/tools/depth_first_traversal.hh>
#include <metapod/tools/has_parent.hh>
#include <boost/fusion/include/vector.hpp>

namespace metapod {
namespace internal {

template< typename Robot, int node_id, bool has_parent=true >
struct SetBodyPose
{
  typedef typename Nodes<Robot, node_id>::type Node;
  static void run(Robot& robot, Spatial::Transform *iX1)
  {
    Node& node = boost::fusion::at_c<node_id>(robot.nodes);
    iX1[node_id] = node.sXp * iX1[Node::parent_id];
  }
};

// specialization for root nodes. Normally not necessary, since
template< typename Robot, int node_id>
struct SetBodyPose<Robot, node_id, false>
{
  typedef typename Nodes<Robot, node_id>::type Node;
  static void run(Robot& robot, Spatial::Transform *iX1)
  {
    Node& node = boost::fusion::at_c<node_id>(robot.nodes);
    iX1[node_id] = node.sXp;
  }
};

template< typename Robot, int node_id >
struct FwdKiniX1Visitor
{
  typedef typename Nodes<Robot, node_id>::type Node;

  template <typename Derived0, typename Derived1>
  static void discover(Robot& robot, Spatial::Transform *iX1,
                       Eigen::MatrixBase<Derived0> &J,
                       Eigen::MatrixBase<Derived1> &mass_com)
  {
    SetBodyPose<Robot, node_id, has_parent<Robot, node_id>::value>::run(robot, iX1);
    Node& node = boost::fusion::at_c<node_id>(robot.nodes);
    J.template block<6, Node::Joint::NBDOF>(0, Node::q_idx)
        = iX1[node_id].inverse().apply(node.joint.S);
    const metapod::Spatial::Inertia &I = robot.inertias[node_id];
    mass_com += I.m() * iX1[node_id].applyInv(I.h()/I.m());
  }

  template <typename Derived0, typename Derived1>
  static void finish(Robot&, Spatial::Transform *,
                     Eigen::MatrixBase<Derived0> &,
                     Eigen::MatrixBase<Derived1> &)
  {}
};

template< typename Robot, int node_id >
struct FwdKiniX0FromiX1Visitor
{
  typedef typename Nodes<Robot, node_id>::type Node;
  static void discover(Robot& robot, const Spatial::Transform *iX1, const Spatial::Transform &_1X0)
  {
    Node& node = boost::fusion::at_c<node_id>(robot.nodes);
    node.body.iX0 = iX1[node_id] * _1X0;
  }
  static void finish(Robot& , const Spatial::Transform *, const Spatial::Transform &)
  {}
};
} // end of namespace metapod::internal


template< typename Robot > struct fwd_kin_iX1
{
  METAPOD_STATIC_ASSERT(Robot::child1_id == NO_CHILD &&
                        Robot::child2_id == NO_CHILD &&
                        Robot::child3_id == NO_CHILD &&
                        Robot::child4_id == NO_CHILD,
      "fwd_kin_iX1 only supports models with a single root link");
  typedef typename Nodes<Robot, Robot::child0_id>::type Node;
  // assume sXp is up to date, for all joints but the root one
  template <typename Derived0, typename Derived1>
  static void run(Robot& robot, Spatial::Transform *iX1,
                  Eigen::MatrixBase<Derived0> &J,
                  Eigen::MatrixBase<Derived1> &mass_com)
  {
    assert(J.rows() == 6);
    assert(J.cols() ==  Robot::NBDOF);
    assert(mass_com.rows() == 3);
    assert(mass_com.cols() ==  1);

    mass_com = robot.inertias[Node::id].h(); // mass*com of the root joint in its moving frame
    // start a depth first traversal but skip the root node
    internal::depth_first_traversal_internal<internal::FwdKiniX1Visitor, Robot, Node::child0_id>::run(robot, iX1, J, mass_com);
    internal::depth_first_traversal_internal<internal::FwdKiniX1Visitor, Robot, Node::child1_id>::run(robot, iX1, J, mass_com);
    internal::depth_first_traversal_internal<internal::FwdKiniX1Visitor, Robot, Node::child2_id>::run(robot, iX1, J, mass_com);
    internal::depth_first_traversal_internal<internal::FwdKiniX1Visitor, Robot, Node::child3_id>::run(robot, iX1, J, mass_com);
    internal::depth_first_traversal_internal<internal::FwdKiniX1Visitor, Robot, Node::child4_id>::run(robot, iX1, J, mass_com);
  }
};

template< typename Robot > struct fwd_kin_iX0_from_iX1
{
  METAPOD_STATIC_ASSERT(Robot::child1_id == NO_CHILD &&
                        Robot::child2_id == NO_CHILD &&
                        Robot::child3_id == NO_CHILD &&
                        Robot::child4_id == NO_CHILD,
      "fwd_kin_iX0_from_iX1 only supports models with a single root link");
  typedef typename Nodes<Robot, Robot::child0_id>::type Node;

  // assume sXp is up to date for the root joint
  static void run(Robot& robot, const Spatial::Transform iX1[])
  {
    Node& node = boost::fusion::at_c<Robot::child0_id>(robot.nodes);
    node.body.iX0 = node.sXp;
    // start a depth first traversal but skip the root node
    internal::depth_first_traversal_internal<internal::FwdKiniX0FromiX1Visitor, Robot, Node::child0_id>::run(robot, iX1, node.body.iX0);
    internal::depth_first_traversal_internal<internal::FwdKiniX0FromiX1Visitor, Robot, Node::child1_id>::run(robot, iX1, node.body.iX0);
    internal::depth_first_traversal_internal<internal::FwdKiniX0FromiX1Visitor, Robot, Node::child2_id>::run(robot, iX1, node.body.iX0);
    internal::depth_first_traversal_internal<internal::FwdKiniX0FromiX1Visitor, Robot, Node::child3_id>::run(robot, iX1, node.body.iX0);
    internal::depth_first_traversal_internal<internal::FwdKiniX0FromiX1Visitor, Robot, Node::child4_id>::run(robot, iX1, node.body.iX0);
  }
};

} // end of namespace metapod

# endif
