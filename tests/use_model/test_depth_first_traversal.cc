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

// This test traverses the tree depth-first and prints events

// Common test tools
#include "common.hh"
#include <metapod/tools/depth_first_traversal.hh>
#include <metapod/tools/depth_first_traversal_ng.hh>
#include <metapod/tools/dft.hh>
#include <boost/mpl/for_each.hpp>

using namespace metapod;

// Print events while traversing the tree with indentation showing the level
// of recursion
template < typename Robot, int node_id > struct PrintDFTraversalVisitor
{
  typedef typename Nodes<Robot, node_id>::type Node;

  static void discover(std::ostream &os, int &depth)
  {
    print<Node>(os, depth, "discover");
    ++depth;
  }
  static void finish(std::ostream &os, int &depth)
  {
    --depth;
    print<Node>(os, depth, "finish");
  }
};

class PrintDFT {
private:
  std::ostream &os_;
  int depth_;

  template< typename Event >
  void visit_dispatch(Event, dft_discover_tag) {
    typedef typename Nodes<typename Event::Robot, Event::node_id>::type Node;
    print<Node>(os_, depth_, "discover");
    ++depth_;
  }

  template< typename Event >
  void visit_dispatch(Event, dft_finish_tag) {
    --depth_;
    typedef typename Nodes<typename Event::Robot, Event::node_id>::type Node;
    print<Node>(os_, depth_, "finish");
  }

public:
  PrintDFT(std::ostream &os, int &depth) : os_(os), depth_(depth) {}
  template< typename Event >
  void operator()(Event event) {
    typedef typename Event::event_tag event_tag;
    visit_dispatch(event, event_tag());
  }
};

BOOST_AUTO_TEST_CASE (test_depth_first_traversal)
{
  const char result_file[] = "depth_first_traversal.log";
  std::ofstream log(result_file, std::ofstream::out);
  int depth = 0;
  depth_first_traversal<PrintDFTraversalVisitor, CURRENT_MODEL_ROBOT>::run(log, depth);
  log.close();
  // Compare results with reference file
  compareTexts(result_file, TEST_DIRECTORY "/depth_first_traversal.ref");
}

BOOST_AUTO_TEST_CASE (test_depth_first_traversal_ng)
{
  const char result_file[] = "depth_first_traversal_ng.log";
  std::ofstream log(result_file, std::ofstream::out);
  int depth = 0;
  depth_first_traversal_ng<PrintDFTraversalVisitor, CURRENT_MODEL_ROBOT>::run(log, depth);
  log.close();
  // Compare results with reference file
  compareTexts(result_file, TEST_DIRECTORY "/depth_first_traversal.ref");
}

BOOST_AUTO_TEST_CASE (test_dft)
{
  const char result_file[] = "dft.log";
  std::ofstream log(result_file, std::ofstream::out);
  int depth = 0;
  // TODO: remove child0_id
  boost::mpl::for_each< dft<CURRENT_MODEL_ROBOT, CURRENT_MODEL_ROBOT::child0_id> >(PrintDFT(log, depth));
  log.close();
  // Compare results with reference file
  compareTexts(result_file, TEST_DIRECTORY "/depth_first_traversal.ref");
}

template <typename Robot>
void dft_rt(std::ostream &os, int &depth,
            Robot &robot, int node_id) {
  const RtNode &node = robot.get_rtnode(node_id);
  print_(os, depth, "discover", node);
  ++depth;
  if (node.gchild0_id() != NO_CHILD)
    dft_rt(os, depth, robot, node.gchild0_id());
  --depth;
  print_(os, depth, "finish", node);
  if (node.gsibling_id() != NO_CHILD)
    dft_rt(os, depth, robot, node.gsibling_id());
}

BOOST_AUTO_TEST_CASE (test_dft_rt)
{
  const char result_file[] = "dft_rt.log";
  std::ofstream log(result_file, std::ofstream::out);
  int depth = 0;
  CURRENT_MODEL_ROBOT robot;
  // TODO: remove child0_id
  dft_rt(log, depth, robot, CURRENT_MODEL_ROBOT::child0_id);
  log.close();
  // Compare results with reference file
  compareTexts(result_file, TEST_DIRECTORY "/depth_first_traversal.ref");
}
