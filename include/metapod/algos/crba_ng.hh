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

#ifndef METAPOD_CRBA_NG_HH
# define METAPOD_CRBA_NG_HH

# include <metapod/tools/common.hh>
# include <metapod/tools/dft.hh>
# include <metapod/tools/bwt.hh>
# include <boost/mpl/for_each.hpp>
# include <boost/ref.hpp>

namespace mpl = boost::mpl;

namespace metapod {

template <typename Robot, int row, int col, typename Event, typename event_tag>
struct write_to_row_col {
  typedef mpl::bool_<false> type;
};

template <typename Robot, int row, int col, typename Event>
struct write_to_row_col<Robot, row, col, Event, dft_finish_tag> {
  typedef typename Nodes<Robot, Event::node_id>::type NI;
  typedef typename mpl::and_<
      boost::is_same< mpl::int_<row>, mpl::int_<NI::q_idx> >,
      boost::is_same< mpl::int_<col>, mpl::int_<NI::q_idx> > >::type type;
};

/*
template <typename Robot, int row, int col, typename Event>
struct write_to_row_col<Robot, row, col, Event, bwt_tree_tag> {
  typedef typename Nodes<Robot, Event::node_id>::type NI;
  typedef mpl::and_<boost::is_same<mpl::int_<row>, mpl::int_<Event::NI::q_idx> >,
                    boost::is_same<mpl::int_<col>, mpl::int_<Event::NI::q_idx> >,
                   >::type type;
};
*/
// frontend
template< typename Robot, typename Derived > struct crba_ng {
  template <typename NodeI>
  struct BwtVisitor {
    typedef NodeI NI;
    Robot &robot; // TODO: add const
    Eigen::Matrix<FloatType, 6, NodeI::Joint::NBDOF> &F;
    Eigen::MatrixBase<Derived> &H;

    template < bool i_idx_lower_than_j_idx, int nj, typename Derived0>
    struct write_block;

    template <int nj_id, typename Derived0>
    struct write_block<true, nj_id, Derived0>
    {
      typedef typename Nodes<Robot, nj_id>::type NJ;

      static void run(Robot& robot, Eigen::MatrixBase<Derived0> &H,
                      Eigen::Matrix<FloatType, 6, NI::Joint::NBDOF> &F) {
        NJ& nj = get_node<nj_id>(robot);
        H.template block< NI::Joint::NBDOF, NJ::Joint::NBDOF >
            ( NI::q_idx, NJ::q_idx ).noalias()
                = F.transpose() * nj.joint.S.S();
      }
    };

    template <int nj_id, typename Derived0>
    struct write_block<false, nj_id, Derived0>
    {
      typedef typename Nodes<Robot, nj_id>::type NJ;

      static void run(Robot& robot, Eigen::MatrixBase<Derived0> &H,
                      Eigen::Matrix<FloatType, 6, NI::Joint::NBDOF> &F) {
        NJ& nj = get_node<nj_id>(robot);
        H.template block< NJ::Joint::NBDOF, NI::Joint::NBDOF >
            ( NJ::q_idx, NI::q_idx ).noalias()
                = nj.joint.S.S().transpose() * F;
      }
    };



    template< typename Derived0 >
    BwtVisitor(Robot &robot,
               Eigen::Matrix<FloatType, 6, NodeI::Joint::NBDOF> &F,
               Eigen::MatrixBase<Derived0> &H) :
      robot(robot), F(F), H(H) {}

    template<typename Event, typename event_tag>
    void dispatch(event_tag) {}

    template< typename Event >
    void dispatch(bwt_tree_tag) {
      typedef typename Nodes<Robot, Event::next_node_id>::type NJ; //parent
      typedef typename Nodes<Robot, Event::prev_node_id>::type PrevNJ; //child

      const PrevNJ& prev_nj = get_node<Event::prev_node_id>(robot);
      F = prev_nj.sXp.mulMatrixTransposeBy(F);
      write_block<(NI::q_idx < NJ::q_idx), Event::next_node_id, Derived>::run(robot, H, F);
    }

    template< typename Event >
    void operator()(Event) {
      dispatch<Event>(typename Event::event_tag());
    }
  };

  Robot &robot; // TODO: add const
  Eigen::MatrixBase<Derived> &H;
  Spatial::Inertia Iic[Robot::NBBODIES];
  crba_ng(Robot& robot, Eigen::MatrixBase<Derived> &H) :
    robot(robot), H(H) {
    assert(H.rows() ==  Robot::NBDOF);
    assert(H.cols() ==  Robot::NBDOF);
  }

  // helper function:
  template <int node_id, typename has_a_parent>
  static void update_parent_Iic(has_a_parent);

  // update Parent composite rigid body inertia with the contribution of
  // child Node
  template < int node_id >
  void update_parent_Iic(mpl::bool_<true>) {
    typedef typename Nodes<Robot, node_id>::type Node;
    typedef typename Nodes<Robot, Node::parent_id>::type Parent;
    Node& node = get_node<node_id>(robot);
    Iic[Parent::id] = Iic[Parent::id] + node.sXp.applyInv(Iic[Node::id]);
  }

  // Do nothing if has_a_parent is false
  template < int node_id >
  void update_parent_Iic(mpl::bool_<false>) {}

  template<typename Event, typename event_tag>
  void dispatch(event_tag) {}

  template< typename Event >
  void dispatch(dft_discover_tag) {
    typedef typename Nodes<Robot, Event::node_id>::type NI;
    Iic[NI::id] = robot.inertias[NI::id];
  }

  template< typename Event >
  void dispatch(dft_finish_tag) {
    typedef typename Nodes<Robot, Event::node_id>::type NI;

    NI& ni = get_node<NI::id>(robot);

    typedef typename boost::mpl::if_<
        boost::is_same<mpl::int_<NI::parent_id>,
                       mpl::int_<metapod::NO_PARENT> >,
        mpl::bool_<false>,
        mpl::bool_<true> >::type has_parent;
    update_parent_Iic<NI::id>(has_parent()); // update Iic[NI::parent_id]
    Eigen::Matrix<FloatType, 6, NI::Joint::NBDOF> F = Iic[NI::id] * ni.joint.S;

    // FIXME: test
    typedef typename write_to_row_col<Robot, NI::q_idx, NI::q_idx, Event, dft_finish_tag>::type write;
    //mpl::find_if<Robot, find::>

    H.template block<NI::Joint::NBDOF, NI::Joint::NBDOF>(
        NI::q_idx, NI::q_idx).noalias() = ni.joint.S.transpose() * F;
    boost::mpl::for_each< bwt< Robot, NI::id, metapod::NO_PARENT> >(
        BwtVisitor<NI>(robot, F, H));
  }

  // handle a dft event
  template< typename Event >
  void operator()(Event) {
    dispatch<Event>(typename Event::event_tag());
  }

  // run the whole algorithm
  void operator()() {
    // TODO: rm child0_id
    boost::mpl::for_each< dft< Robot, Robot::child0_id> >(boost::ref(*this));
  }
};

} // end of namespace metapod

#endif
