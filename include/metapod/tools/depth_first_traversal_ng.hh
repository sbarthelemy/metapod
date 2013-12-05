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
// Implementation of the depth-first traversal algorithm
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
// Then visit each node with it:
//
//     MyArg myarg;
//     depth_first_traversal<MyVisitor, simple_arm>::run(myarg);
//
// This will walk down the tree, and call its discover(myarg) and
// finish(myarg) methods at each node.
//
// There are variants with 0, 1, 2, 3 and 4 arguments for both run() and visit()
//
// Note: we might add a third visitor hook called "examine" which would be
//       called on each child node before recursing deeper in the tree.

#ifndef METAPOD_DEPTH_FIRST_TRAVERSAL_NG_HH
# define METAPOD_DEPTH_FIRST_TRAVERSAL_NG_HH

# include "metapod/tools/dft.hh"
# include <boost/mpl/for_each.hpp>

namespace metapod {
namespace internal {

template< template <typename AnyRobot, int any_node_id> class Visitor,
          typename Robot,
          typename Arg0, typename Arg1 >
class DftVisitorWrapper {
private:
  Arg0 &arg0_;
  Arg1 &arg1_;

  template <typename Event>
  void visit_dispatch(Event, dft_discover_tag) {
    Visitor<Robot, Event::node_id>::discover(arg0_, arg1_);
  }

  template <typename Event>
  void visit_dispatch(Event, dft_finish_tag) {
    Visitor<Robot, Event::node_id>::finish(arg0_, arg1_);
  }

public:
  DftVisitorWrapper(Arg0 &arg0, Arg1 &arg1) : arg0_(arg0), arg1_(arg1) {}
  template <typename Event>
  void operator()(Event event) {
    typedef typename Event::event_tag event_tag;
    visit_dispatch(event, event_tag());
  }
};
} // end of namespace metapod::internal
template< template <typename AnyRobot, int any_node_id> class Visitor,
          typename Robot >
struct depth_first_traversal_ng
{
  template<typename Arg0, typename Arg1>
  static void run(Arg0 &arg0, Arg1 &arg1)
  {
    // TODO: remove child0_id
    boost::mpl::for_each< dft<Robot, Robot::child0_id> >(
        internal::DftVisitorWrapper<Visitor, Robot, Arg0, Arg1>(arg0, arg1));
  }
};
} // end of namespace metapod
#endif
