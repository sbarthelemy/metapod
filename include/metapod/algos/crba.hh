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
 * Implementation of the Composite Rigid Body Algorithm,
 * based on Featherstone's Rigid Body Dynamics Algorithms.
 */

#ifndef METAPOD_CRBA_HH
# define METAPOD_CRBA_HH

# include "metapod/tools/common.hh"
# include "metapod/tools/depth_first_traversal.hh"
# include "metapod/tools/backward_traversal_prev.hh"

namespace metapod {
namespace internal {

// helper function: update Parent inertia with the contribution of child Node
template < typename Robot, int parent_id, int node_id >
struct crba_update_parent_inertia
{
  typedef typename Nodes<Robot, parent_id>::type Parent;
  typedef typename Nodes<Robot, node_id>::type Node;
  static void run(Robot& robot)
  {
    Parent& parent = get_node<parent_id>(robot);
    Node& node = get_node<node_id>(robot);
    parent.body.Iic = parent.body.Iic + node.sXp.applyInv(node.body.Iic);
  }
};
// Do nothing if parent_id is NO_PARENT
template < typename Robot, int node_id >
struct crba_update_parent_inertia<Robot, NO_PARENT, node_id>
{
  static void run(Robot&) {}
};

} // end of namespace metapod::internal

// frontend
template< typename Robot > struct crba
{
  template <typename AnyRobot, int node_id >
  struct DftVisitor
  {
    typedef typename Nodes<Robot, node_id>::type NI;
    typedef NI Node;
    // Update NJ with data from PrevNJ
    template< typename AnyyRobot, int nj_id, int prev_nj_id >
    struct BwdtVisitor
    {
      typedef typename Nodes<AnyyRobot, nj_id>::type NJ;
      typedef typename Nodes<AnyyRobot, prev_nj_id>::type PrevNJ;

      template < bool i_idx_lower_than_j_idx, typename Derived>
      struct write_block;

      template <typename Derived>
      struct write_block<true, Derived>
      {
        static void run(AnyyRobot& robot, Eigen::MatrixBase<Derived> &H,
                          Eigen::Matrix<FloatType, 6, NI::Joint::NBDOF> &F)
        {
          NJ& nj = get_node<nj_id>(robot);
          H.template block< NI::Joint::NBDOF, NJ::Joint::NBDOF >
             ( NI::q_idx, NJ::q_idx ).noalias()
                 = F.transpose() * nj.joint.S.S();
        }
      };

      template <typename Derived>
      struct write_block<false, Derived>
      {
        static void run(AnyyRobot& robot, Eigen::MatrixBase<Derived> &H,
                          Eigen::Matrix<FloatType, 6, NI::Joint::NBDOF> &F)
        {
          NJ& nj = get_node<nj_id>(robot);
          H.template block< NJ::Joint::NBDOF, NI::Joint::NBDOF >
              ( NJ::q_idx, NI::q_idx ).noalias()
                  = nj.joint.S.S().transpose() * F;
        }
      };

      template< typename Derived>
      static void discover(AnyyRobot& robot, Eigen::MatrixBase<Derived> &H,
                           Eigen::Matrix<FloatType, 6, NI::Joint::NBDOF> &F)
      {
        PrevNJ& prev_nj = get_node<prev_nj_id>(robot);
        F = prev_nj.sXp.mulMatrixTransposeBy(F);
        write_block<(NI::q_idx < NJ::q_idx), Derived>::run(robot, H, F);
      }
    };

    // forward propagation
    template< typename Derived>
    static void discover(AnyRobot& robot, Eigen::MatrixBase<Derived> &)
    {
      NI& ni = get_node<node_id>(robot);
      ni.body.Iic = robot.inertias[node_id];
    }

    template< typename Derived>
    static void finish(AnyRobot& robot, Eigen::MatrixBase<Derived> &H)
    {
      Node& node = get_node<node_id>(robot);
      internal::crba_update_parent_inertia<AnyRobot, Node::parent_id, node_id>::run(robot);
      Eigen::Matrix<FloatType, 6, Node::Joint::NBDOF> F = node.body.Iic * node.joint.S;

      H.template block<Node::Joint::NBDOF, Node::Joint::NBDOF>(
              Node::q_idx, Node::q_idx).noalias()
                       = node.joint.S.transpose() * F;
      backward_traversal_prev< BwdtVisitor, Robot, node_id >::run(robot, H, F);
    }
  };

  template< typename Derived>
  static void run(Robot& robot, Eigen::MatrixBase<Derived> &H)
  {
    assert(H.rows() ==  Robot::NBDOF);
    assert(H.cols() ==  Robot::NBDOF);
    depth_first_traversal< DftVisitor, Robot >::run(robot, H);
  }
};

} // end of namespace metapod

#endif
