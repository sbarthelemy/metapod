// Copyright 2012, 2013
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

//
// Implementation of the "backward" traversal algorithm: from a start node,
// traverse the tree toward the root until reaching the end node.
//
// To use it, create a visitor
//
//     template<typename Robot, int node_id> class MyVisitor
//     {
//       static void discover(MyArg & arg)
//       {
//         //... do something
//       };
//       static void finish(MyArg & arg)
//       {
//         //... do something
//       };
//     }
//
// where node_id is the id of the node being visited.
//
// Then visit each node with it:
//
//     MyArg myarg;
//     backward_traversal<MyVisitor,
//                        Robot,
//                        start_node_id,
//                        end_node_id>::run(myarg);
//
// This will walk the tree jumping from a start_node to its ancestors, and
// call discover(myarg) and finish(myarg) at each visited node, until it
// reaches the end node. If you do not provide the end_note_id argument, it
// defaults to NO_PARENT (no parent), the root of the kinematic tree.
//
// There are variants with 0, 1, 2 and 3 arguments (for run(), discover() and
// finish())

#ifndef METAPOD_BWT_HH
# define METAPOD_BWT_HH

# include "metapod/tools/common.hh"
# include <metapod/tools/is_ancestor.hh>
# include <boost/mpl/deref.hpp>
# include <boost/mpl/equal_to.hpp>

namespace mpl = boost::mpl;

namespace metapod {

struct bwt_discover_tag : boost::mpl::int_<0> {
  typedef bwt_discover_tag type; };
struct bwt_tree_tag : boost::mpl::int_<1> {
  typedef bwt_tree_tag type; };

template <typename Robot_, int node_id_>
struct bwt_discover {
  typedef bwt_discover_tag event_tag;
  typedef Robot_ Robot;
  static const int node_id = node_id_;
};

template <typename Robot_, int prev_node_id_, int next_node_id_>
struct bwt_tree {
  typedef bwt_tree_tag event_tag;
  typedef Robot_ Robot;
  static const int prev_node_id = prev_node_id_;
  static const int next_node_id = next_node_id_;
};

namespace internal {

struct bwt_tag {};

struct bwt_end_it {};

template <typename Robot, int descendant_id, int ancestor_id,
          int prev_node_id, int next_node_id>
struct bwt_tree_it;

template <typename Robot, int descendant_id, int ancestor_id, int node_id>
struct bwt_discover_it {
  typedef boost::mpl::forward_iterator_tag category;

  // needed for deref<>:
  typedef bwt_discover<Robot, node_id> type;

  // needed for next<>:
  typedef typename metapod::Nodes<Robot, node_id>::type Node;
  // TODO: discover the last node too!
  //typedef typename mpl::if_<
  //  boost::is_same< mpl::int_<node_id>, mpl::int_<ancestor_id> >,
  //  bwt_end_it,
  //  bwt_tree_it<Robot,  descendant_id, ancestor_id, node_id, Node::parent_id> >::type next;
  typedef typename mpl::if_<
    boost::is_same< mpl::int_<Node::parent_id>, mpl::int_<ancestor_id> >,
    bwt_end_it,
    bwt_tree_it<Robot,  descendant_id, ancestor_id, node_id, Node::parent_id> >::type next;
};

template <typename Robot, int descendant_id, int ancestor_id,
          int prev_node_id, int next_node_id>
struct bwt_tree_it {
  typedef boost::mpl::forward_iterator_tag category;

  // needed for deref<>:
  typedef bwt_tree<Robot, prev_node_id, next_node_id> type;

  // needed for next<>:
  typedef bwt_discover_it<Robot, descendant_id, ancestor_id, next_node_id> next;
};

} // end of namespace metapod::internal

template <typename Robot, int descendant_id, int ancestor_id>
struct bwt {
  METAPOD_STATIC_ASSERT(( is_ancestor<Robot, ancestor_id, descendant_id>::value ),
                        "end node should be an ancestor of start node");
};
} // end of namespace metapod

namespace boost {
namespace mpl {

template <typename Robot, int descendant_id, int ancestor_id>
struct begin< metapod::bwt<Robot, descendant_id, ancestor_id> >
{
  typedef metapod::internal::bwt_discover_it<Robot, descendant_id, ancestor_id, descendant_id> type;
};

template <typename Robot, int descendant_id, int ancestor_id>
struct end< metapod::bwt<Robot, descendant_id, ancestor_id> >
{
  typedef metapod::internal::bwt_end_it type;
};

}
} // end of namespace boost
#endif
