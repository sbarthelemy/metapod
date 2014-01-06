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

#ifndef METAPOD_BACKWARD_TRAVERSAL_NG_HH
# define METAPOD_BACKWARD_TRAVERSAL_NG_HH

# include "metapod/tools/common.hh"
# include <metapod/tools/is_ancestor.hh>
# include <metapod/tools/bwt.hh>
# include <boost/mpl/for_each.hpp>
# include <boost/mpl/pop_back.hpp>

namespace metapod {
namespace internal {

template< template <typename AnyRobot, int any_node_id> class Visitor,
          typename Robot,
          typename Arg0, typename Arg1 >
class BtwVisitorWrapper {
private:
  Arg0 &arg0_;
  Arg1 &arg1_;

  template <typename Event>
  void visit_dispatch(Event, bwt_discover_tag) {
    Visitor<Robot, Event::node_id>::discover(arg0_, arg1_);
  }

  template <typename Event>
  void visit_dispatch(Event, bwt_tree_tag) {}

public:
  BtwVisitorWrapper(Arg0 &arg0, Arg1 &arg1) : arg0_(arg0), arg1_(arg1) {}
  template <typename Event>
  void operator()(Event event) {
    typedef typename Event::event_tag event_tag;
    visit_dispatch(event, event_tag());
  }
};
} // end of namespace metapod::internal

template< template <typename AnyRobot, int any_node_id> class Visitor,
          typename Robot,
          int start_node_id,
          int end_node_id=NO_PARENT >
struct backward_traversal_ng
{
  template<typename Arg0, typename Arg1>
  static void run(Arg0 &arg0, Arg1 &arg1)
  {
    boost::mpl::for_each< bwt<Robot, start_node_id, end_node_id> >(
        internal::BtwVisitorWrapper<Visitor, Robot, Arg0, Arg1>(arg0, arg1));
  }
};

} // end of namespace metapod
#endif
