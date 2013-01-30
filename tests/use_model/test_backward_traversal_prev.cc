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

//Common test tools
#include "common.hh"
#include <metapod/tools/backward_traversal_prev.hh>

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
// Generic implementation: assumes prev_node_id != NO_NODE
template < typename Robot, int node_id, int prev_node_id > struct PrintBwdTraversalVisitor
{
  typedef typename Nodes<Robot, node_id>::type Node;
  typedef typename Nodes<Robot, prev_node_id>::type PrevNode;
  static void discover(std::ostream & os, int & depth)
  {
    const std::string prefix(depth, '\t');
    os << prefix << "discover: curr:"
        << Node::joint_name << " -- " << Node::body_name << "\n";
    os << prefix << "          prev: "
        << PrevNode::joint_name << " -- " << PrevNode::body_name << "\n";
    ++depth;
  }
  static void finish(std::ostream & os, int & depth)
  {
    --depth;
    const std::string prefix(depth, '\t');
    os << prefix << "finish: curr:"
        << Node::joint_name << " -- " << Node::body_name << "\n";
    os << prefix << "        prev: "
        << PrevNode::joint_name << " -- " << PrevNode::body_name << "\n";
  }
};

BOOST_AUTO_TEST_CASE (test_backward_traversal)
{
  const char result_file[] = "backward_traversal_prev.log";
  std::ofstream log(result_file, std::ofstream::out);
  int depth = 0;
  backward_traversal_prev<PrintBwdTraversalVisitor, CURRENT_MODEL_ROBOT,
      start_node>::run(log, depth);
  log.close();
  // Compare results with reference file
  compareTexts(result_file, TEST_DIRECTORY "/backward_traversal_prev.ref");
}

BOOST_AUTO_TEST_CASE (test_backward_traversal_end)
{
  const char result_file[] = "backward_traversal_prev_end.log";
  std::ofstream log(result_file, std::ofstream::out);
  int depth = 0;
  backward_traversal_prev<PrintBwdTraversalVisitor, CURRENT_MODEL_ROBOT,
      start_node, end_node>::run(log, depth);
  log.close();
  // Compare results with reference file
  compareTexts(result_file, TEST_DIRECTORY "/backward_traversal_prev_end.ref");
}
