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

#ifndef METAPOD_DFT_HH
# define METAPOD_DFT_HH

# include "metapod/tools/common.hh"
# include <boost/mpl/deref.hpp>
# include <boost/mpl/equal_to.hpp>
//#include <boost/mpl/next_prior.hpp>
# include <boost/type_traits/is_same.hpp>
# include <boost/mpl/vector_c.hpp>
//#include <boost/fusion/algorithm/iteration/iter_fold.hpp>
# include<boost/mpl/placeholders.hpp>
# include<boost/mpl/lambda.hpp>
//# include <boost/mpl/iterator_category.hpp>


namespace mpl = boost::mpl;

namespace metapod {

struct dft_discover_tag : boost::mpl::int_<0> {
  typedef dft_discover_tag type; };
struct dft_finish_tag : boost::mpl::int_<1> {
  typedef dft_finish_tag type; };


template <typename Robot_, int node_id_>
struct dft_discover {
  typedef dft_discover_tag event_tag;
  typedef Robot_ Robot;
  static const int node_id = node_id_;
};

template <typename Robot_, int node_id_>
struct dft_finish {
  typedef dft_finish_tag event_tag;
  typedef Robot_ Robot;
  static const int node_id = node_id_;
};


namespace internal {

struct dft_tag {};

struct dft_end_it {};

template <typename Robot, int root_id, int node_id>
struct dft_finish_it;

template <typename Robot, int root_id, int node_id>
struct dft_discover_it {
  typedef boost::mpl::forward_iterator_tag category;

  // needed for deref<>:
  typedef dft_discover<Robot, node_id> type;

  // needed for next<>:
  typedef typename metapod::Nodes<Robot, node_id>::type Node;
  typedef typename mpl::if_<
    boost::is_same< mpl::int_<Node::child0_id>, mpl::int_<metapod::NO_CHILD> >,
    dft_finish_it<Robot, root_id, node_id>,
    dft_discover_it<Robot, root_id, Node::child0_id> >::type next;
};

template <typename Robot, int root_id, int node_id>
struct dft_finish_it {
  typedef mpl::forward_iterator_tag category;

  // needed for deref<>:
  typedef dft_finish<Robot, node_id> type;

  // needed for next<>:
  typedef typename metapod::Nodes<Robot, node_id>::type Node;
  typedef typename boost::mpl::if_<
    boost::is_same< mpl::int_<Node::sibling_id>, mpl::int_<metapod::NO_CHILD> >,
    typename mpl::if_< // no sibling, visit parent if any
      boost::is_same< mpl::int_<node_id>, mpl::int_<root_id> >,
      dft_end_it,
      dft_finish_it<Robot, root_id, Node::parent_id> >::type,
    dft_discover_it<Robot, root_id, Node::sibling_id> >::type next;
};
} // end of namespace metapod::internal

template <typename Robot, int root_id>
struct dft {};
} // end of namespace metapod

namespace boost {
namespace mpl {

template <typename Robot, int root_id>
struct begin< metapod::dft<Robot, root_id> >
{
  typedef metapod::internal::dft_discover_it<Robot, root_id, root_id> type;
};

template <typename Robot, int root_id>
struct end< metapod::dft<Robot, root_id> >
{
  typedef metapod::internal::dft_end_it type;//metapod::internal::dft_finish_it<Robot, root_id, root_id> type;
};

}
} // end of namespace boost
#endif
