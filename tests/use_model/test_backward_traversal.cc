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

// This test traverses the tree backward and prints events

// Common test tools
#include "common.hh"
#include <metapod/tools/backward_traversal.hh>
#include <metapod/tools/backward_traversal_ng.hh>
#include <metapod/tools/bwt.hh>
#include <boost/mpl/for_each.hpp>

using namespace metapod;

// start at the hand and finish with the arm
#ifdef CURRENT_MODEL_IS_SIMPLE_HUMANOID
const int start_node = CURRENT_MODEL_ROBOT::LARM_LINK7;
const int end_node = CURRENT_MODEL_ROBOT::LARM_LINK3;
#else
const int start_node = CURRENT_MODEL_ROBOT::HAND;
const int end_node = CURRENT_MODEL_ROBOT::ARM;
#endif

// Print events while traversing the tree with indentation showing the level
// of recursion
template < typename Robot, int node_id > struct PrintBwdTraversalVisitor
{
  typedef typename Nodes<Robot, node_id>::type Node;
  static void discover(std::ostream &os, int &depth)
  {
    print<Node>(os, depth, "discover curr");
    ++depth;
  }
};

class PrintBWT {
private:
  std::ostream &os_;
  int depth_;

  template< typename Event >
  void visit_dispatch(Event, bwt_discover_tag) {
    typedef typename Nodes<typename Event::Robot, Event::node_id>::type Node;
    print<Node>(os_, depth_, "discover");
    ++depth_;
  }

  template< typename Event >
  void visit_dispatch(Event, bwt_tree_tag) {
    typedef typename Nodes<typename Event::Robot, Event::prev_node_id>::type PrevNode;
    typedef typename Nodes<typename Event::Robot, Event::next_node_id>::type NextNode;
    print<PrevNode>(os_, depth_, "tree prev");
    print<NextNode>(os_, depth_, "     next");
    ++depth_;
  }

public:
  PrintBWT(std::ostream &os, int &depth) : os_(os), depth_(depth) {}
  template< typename Event >
  void operator()(Event event) {
    typedef typename Event::event_tag event_tag;
    visit_dispatch(event, event_tag());
  }
};

BOOST_AUTO_TEST_CASE (test_backward_traversal)
{
  const char result_file[] = "backward_traversal.log";
  std::ofstream log(result_file, std::ofstream::out);
  int depth = 0;
  backward_traversal<PrintBwdTraversalVisitor, CURRENT_MODEL_ROBOT,
      start_node>::run(log, depth);
  log.close();
  // Compare results with reference file
  compareTexts(result_file, TEST_DIRECTORY "/backward_traversal.ref");
}

BOOST_AUTO_TEST_CASE (test_backward_traversal_ng)
{
  const char result_file[] = "backward_traversal_ng.log";
  std::ofstream log(result_file, std::ofstream::out);
  int depth = 0;
  backward_traversal_ng<PrintBwdTraversalVisitor, CURRENT_MODEL_ROBOT,
      start_node>::run(log, depth);
  log.close();
  // Compare results with reference file
  compareTexts(result_file, TEST_DIRECTORY "/backward_traversal.ref");
}

BOOST_AUTO_TEST_CASE (test_backward_traversal_end)
{
  const char result_file[] = "backward_traversal_end.log";
  std::ofstream log(result_file, std::ofstream::out);
  int depth = 0;
  backward_traversal<PrintBwdTraversalVisitor, CURRENT_MODEL_ROBOT,
      start_node, end_node>::run(log, depth);
  log.close();
  // Compare results with reference file
  compareTexts(result_file, TEST_DIRECTORY "/backward_traversal_end.ref");
}

BOOST_AUTO_TEST_CASE (test_backward_traversal_ng_end)
{
  const char result_file[] = "backward_traversal_ng_end.log";
  std::ofstream log(result_file, std::ofstream::out);
  int depth = 0;
  backward_traversal_ng<PrintBwdTraversalVisitor, CURRENT_MODEL_ROBOT,
      start_node, end_node>::run(log, depth);
  log.close();
  // Compare results with reference file
  compareTexts(result_file, TEST_DIRECTORY "/backward_traversal_end.ref");
}

BOOST_AUTO_TEST_CASE (test_bwt_end)
{
  const char result_file[] = "bwt_end.log";
  std::ofstream log(result_file, std::ofstream::out);
  int depth = 0;
  boost::mpl::for_each< bwt<CURRENT_MODEL_ROBOT, start_node, end_node> >(PrintBWT(log, depth));
  log.close();
  // Compare results with reference file
  compareTexts(result_file, TEST_DIRECTORY "/bwt_end.ref");
}
