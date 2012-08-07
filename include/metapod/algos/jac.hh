// Copyright 2012,
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
 * Compute the kinematic jacobian. i.e. the matrix J such that
 * J*dq is the concatenation of each body spatial motion vector (in the moving
 * body frame)
 */

#ifndef METAPOD_JAC_HH
# define METAPOD_JAC_HH

# include "metapod/tools/common.hh"

namespace metapod
{
  template< typename Robot, typename Tree > struct jac_forward_propagation;

  template< typename Robot, typename BI, typename BJ, typename Parent >
  struct jac_backward_propagation;

  template< typename Robot, bool run_jcalc = true > struct jac {};

  template< typename Robot > struct jac< Robot, false >
  {
    typedef Eigen::Matrix< FloatType, Robot::NBDOF, 1 > confVector;

    static void run(const confVector & q)
    {
      jac_forward_propagation< Robot, typename Robot::Tree >::run();
    }
  };

  template< typename Robot > struct jac< Robot, true >
  {
    typedef Eigen::Matrix< FloatType, Robot::NBDOF, 1 > confVector;

    static void run(const confVector & q)
    {
      // we do not care about velocities, so let use dq==0
      jcalc< Robot >::run(q, confVector::Zero());
      jac_forward_propagation< Robot, typename Robot::Tree >::run();
    }
  };

  template < typename Robot, typename Tree >
  struct jac_forward_propagation
  {
    typedef Tree Node;
    typedef typename Node::Body CurrentBody;

    static void run()
    {
      // contribution of the current joint to the current body velocity
      Robot::J.block(6*Node::Body::label, Node::Joint::positionInConf,
                     6, Node::Joint::NBDOF) = Node::Joint::S;

      // contribution of the ancestor joints to the current body velocity,
      // copied from the parent body
      jac_backward_propagation< Robot, CurrentBody, typename CurrentBody::Parent, typename CurrentBody::Parent >::run();

      jac_forward_propagation< Robot, typename Node::Child0 >::run();
      jac_forward_propagation< Robot, typename Node::Child1 >::run();
      jac_forward_propagation< Robot, typename Node::Child2 >::run();
      jac_forward_propagation< Robot, typename Node::Child3 >::run();
      jac_forward_propagation< Robot, typename Node::Child4 >::run();
    }
  };

  template< typename Robot >
  struct jac_forward_propagation< Robot, NC >
  {
    static void run() {}
  };

  template< typename Robot, typename CurrentBody, typename ParentBody, typename AncestorBody >
  struct jac_backward_propagation
  {
    static void run()
    {
      Robot::J.block(6*CurrentBody::label, AncestorBody::Joint::positionInConf,
                     6, AncestorBody::Joint::NBDOF)
        = CurrentBody::Joint::sXp.toMatrix()
        * Robot::J.block(6*ParentBody::label, AncestorBody::Joint::positionInConf,
                         6, AncestorBody::Joint::NBDOF);
      jac_backward_propagation< Robot,
                                CurrentBody,
                                ParentBody,
                                typename AncestorBody::Parent >::run();
    }
  };

  template< typename Robot, typename CurrentBody, typename ParentBody>
  struct jac_backward_propagation< Robot, CurrentBody, ParentBody, NP >
  {
    static void run() {}
  };

} // end of namespace metapod

#endif
