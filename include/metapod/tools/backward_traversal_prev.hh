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

// Implementation of the "backward" traversal algorithm: from a node,
// traverse the tree until reaching the root.
//
// To use it, create a visitor
//
//     template<typename Robot, int node_id, int prev_node_id> class MyVisitor
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
// where node_id is the id of the node being visited and prev_node_id is
// the id of the previous one.
//
// Given that the tree is traversed backward, the previous node is always
// a child of the current one.
//
// Then visit each node with it:
//
//     MyArg myarg;
//     backward_traversal_prev<MyVisitor,
//                             Robot,
//                             start_node_id,
//                             end_node_id>::run(myarg);
//
// This will walk the tree jumping from a start_node to its ancestors, and
// call discover(myarg) and finish(myarg) at each visited node, until it
// reaches the end node. If you do not provide the end_note_id argument, it
// defaults to NO_PARENT (no parent), the root of the kinematic tree.
//
// There are variants with 0, 1 and 2 arguments for both run() and visit()
//
// There is also a variant algorithm, backward_traversal_previous, which
// does not skip the start node, but visit it usion NO_NODE as previous node
// instead.
//
//     A--B--G   H
//         \    /
//          C--D--E--F
// calling
//
//     backward_traversal_prev<MyVisitor,
//                             Robot,
//                             e_id,
//                             a_id>::run();
//
// will result in the following calls being made
//
//     MyVisitor<Robot, d_id, e_id>::discover();
//     MyVisitor<Robot, c_id, d_id>::discover();
//     MyVisitor<Robot, b_id, c_id>::discover();
//     MyVisitor<Robot, b_id, c_id>::finish();
//     MyVisitor<Robot, c_id, d_id>::finish();
//     MyVisitor<Robot, d_id, e_id>::finish();
//
// and calling
//
//     backward_traversal_previous<MyVisitor,
//                                 Robot,
//                                 e_id,
//                                 a_id>::run();
//
// will result in the following calls being made
//
//     MyVisitor<Robot, e_id, NO_NODE>::discover();
//     MyVisitor<Robot, d_id, e_id>::discover();
//     MyVisitor<Robot, c_id, d_id>::discover();
//     MyVisitor<Robot, b_id, c_id>::discover();
//     MyVisitor<Robot, b_id, c_id>::finish();
//     MyVisitor<Robot, c_id, d_id>::finish();
//     MyVisitor<Robot, d_id, e_id>::finish();
//     MyVisitor<Robot, e_id, NO_NODE>::finish();

#ifndef METAPOD_BACKWARD_TRAVERSAL_PREV_HH
# define METAPOD_BACKWARD_TRAVERSAL_PREV_HH

# include "metapod/tools/common.hh"
# include "metapod/tools/is_ancestor.hh"

namespace metapod {
namespace internal {

template< template <typename AnyRobot, int any_node_id, int any_prev_node_id> class Visitor,
          typename Robot, int node_id, int prev_node_id, int end_node_id>
struct backward_traversal_prev_internal
{
  typedef typename Nodes<Robot, node_id>::type Node;

  static void run()
  {
    Visitor<Robot, node_id, prev_node_id>::discover();
    backward_traversal_prev_internal<Visitor, Robot, Node::parent_id, node_id, end_node_id>::run();
    Visitor<Robot, node_id, prev_node_id>::finish();
  }

  template<typename Arg0>
  static void run(Arg0& arg0)
  {
    Visitor<Robot, node_id, prev_node_id>::discover(arg0);
    backward_traversal_prev_internal<Visitor, Robot, Node::parent_id, node_id, end_node_id>::run(arg0);
    Visitor<Robot, node_id, prev_node_id>::finish(arg0);
  }

  template<typename Arg0, typename Arg1>
  static void run(Arg0& arg0, Arg1& arg1)
  {
    Visitor<Robot, node_id, prev_node_id>::discover(arg0, arg1);
    backward_traversal_prev_internal<Visitor, Robot, Node::parent_id, node_id, end_node_id>::run(arg0, arg1);
    Visitor<Robot, node_id, prev_node_id>::finish(arg0, arg1);
  }
};

// end recursion when we reach the end node
template< template <typename AnyRobot, int any_node_id, int any_prev_node_id> class Visitor,
          typename Robot, int prev_node_id, int end_node_id>
struct backward_traversal_prev_internal<Visitor, Robot, end_node_id, prev_node_id, end_node_id>
{
  static void run() {}

  template<typename Arg0>
  static void run(Arg0&) {}

  template<typename Arg0, typename Arg1>
  static void run(Arg0& , Arg1&) {}
};

} // end of namespace metapod::internal

template< template <typename AnyRobot, int any_node_id, int any_prev_node_id> class Visitor,
          typename Robot,
          int start_node_id,
          int end_node_id=NO_PARENT >
struct backward_traversal_previous
{
  METAPOD_STATIC_ASSERT(( is_ancestor<Robot, end_node_id, start_node_id>::value ),
                        "end node should be an ancestor if start node");
  static void run()
  {
    internal::backward_traversal_prev_internal<Visitor, Robot, start_node_id, NO_NODE, end_node_id>::run();
  }

  template<typename Arg0>
  static void run(Arg0& arg0)
  {
    internal::backward_traversal_prev_internal<Visitor, Robot, start_node_id, NO_NODE, end_node_id>::run(arg0);
  }

  template<typename Arg0, typename Arg1>
  static void run(Arg0& arg0, Arg1& arg1)
  {
    internal::backward_traversal_prev_internal<Visitor, Robot, start_node_id, NO_NODE, end_node_id>::run(arg0, arg1);
  }
};

// Generic implementation: requires that start_node_id != end_node_id
template< template <typename AnyRobot, int any_node_id, int any_prev_node_id> class Visitor,
          typename Robot,
          int start_node_id,
          int end_node_id=NO_PARENT >
struct backward_traversal_prev
{
  METAPOD_STATIC_ASSERT(( is_ancestor<Robot, end_node_id, start_node_id>::value ),
                        "end node should be an ancestor if start node");
  typedef typename Nodes<Robot, start_node_id>::type StartNode;
  static void run()
  {
    internal::backward_traversal_prev_internal<Visitor, Robot, StartNode::parent_id, start_node_id, end_node_id>::run();
  }

  template<typename Arg0>
  static void run(Arg0& arg0)
  {
    internal::backward_traversal_prev_internal<Visitor, Robot, StartNode::parent_id, start_node_id, end_node_id>::run(arg0);
  }

  template<typename Arg0, typename Arg1>
  static void run(Arg0& arg0, Arg1& arg1)
  {
    internal::backward_traversal_prev_internal<Visitor, Robot, StartNode::parent_id, start_node_id, end_node_id>::run(arg0, arg1);
  }
};

// Specialization: deal with the case where the user called the algorithm
// with end_node_id == start_node_id
template< template <typename AnyRobot, int any_node_id, int any_prev_node_id> class Visitor,
          typename Robot,
          int end_node_id >
struct backward_traversal_prev<Visitor, Robot, end_node_id, end_node_id>
{
  static void run() {}

  template<typename Arg0>
  static void run(Arg0&) {}

  template<typename Arg0, typename Arg1>
  static void run(Arg0& , Arg1&) {}
};
} // end of namespace metapod
#endif
