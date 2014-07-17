// Copyright 2014
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

#include <metapod/graph.hh>
#include <boost/scoped_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/bind.hpp>
#include <stdexcept>
#include <Eigen/Dense>
#include <algorithm>

#include <boost/graph/depth_first_search.hpp>

namespace {


class DrawVisitor : public boost::default_dfs_visitor {
public:
  DrawVisitor() : indent_(0) {}

  template <typename Edge, typename Graph>
  void examine_edge(Edge u, const Graph &g) {
    //Vertex source = boost::source(u, g);
    //Vertex target = boost::target(u, g);
    std::cerr << g[u].name << "\n";
    Draw(g[u].transform);
    std::cerr << "\n";
  }

  template <typename Vertex, typename Graph>
  void discover_vertex(Vertex u, const Graph &g) {
    std::cerr << g[u].name << "\n";
    ++indent_;
    std::cerr << "\n";
  }

private:
  size_t indent_;
};

}
namespace metapod {

template <typename T>
Tr<T> *Tr<T>::clone() const { return new Tr(data); }

template <typename T>
void Tr<T>::Accept(TransformVisitor &vis) const { vis.Visit(*this); }
}
/*

namespace AL {
namespace Model {

SceneGraph::SceneGraph() {}

SceneGraph::~SceneGraph() {}

SceneGraph::Vertex SceneGraph::AddFrame(const std::string &name) {
  return boost::add_vertex(VertexProp(name), g_);
}


SceneGraph::Edge SceneGraph::AddTransform(Vertex source, Vertex target,
                                          Transform tr,
                                          const std::string &name)
{
  Edge edge;
  bool success = false;
  boost::tie(edge, success) =
      boost::add_edge(source, target, EdgeProp(tr, name), g_);
  if (!success) {
    throw std::runtime_error("could not add edge");
  }
  return edge;
}

void SceneGraph::Draw(Vertex root) const {
  std::vector<boost::default_color_type> colors(num_vertices(g_));
  boost::depth_first_visit(g_, root, ::DrawVisitor(), &(colors[0]));
}
}
}
*/

