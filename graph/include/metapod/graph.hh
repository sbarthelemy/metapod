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

/*
 * This file provides tools to deal with robot models.
 */

#ifndef METAPOD_GRAPH_HH
#define METAPOD_GRAPH_HH

#include <metapod/graph/config.hh>
#include <boost/scoped_ptr.hpp>
#include <vector>
#include <list>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <boost/graph/adjacency_list.hpp>
#include <boost/preprocessor/seq.hpp>

namespace metapod
{

// handle a tree/graph of transform

// vertice: frame, which can also hold a name and leaves (inertia, meshes)
// edge: transform

// rotation
// translation
// displacement
// transform (non rigid)

// mass
// mass + inertia

// point
// free vector
// mesh?
// scale?
// analytical shapes?

// Transport(Transform, Leaf)
// Multiply(Transform left, Transform right, Transform result)

class METAPOD_GRAPH_DLLAPI Identity {
 public:
  Identity() {}
};
#define TRANSFORM_TYPES_RAW\
  (Identity)\
  (Eigen::AffineCompact3d)\
  (Eigen::Translation3d)\
  (RevoluteJoint3d)\

#define WRAP_TRANSFORM(s, data, elem) Tr<elem>
#define TRANSFORM_TYPES_WRAPPED \
  BOOST_PP_SEQ_TRANSFORM(WRAP_TRANSFORM, ~, TRANSFORM_TYPES_RAW)

struct METAPOD_GRAPH_DLLAPI RevoluteJoint3d {
 public:
  RevoluteJoint3d(const Eigen::Vector3d &axis) : axis(axis) {}

  Eigen::Vector3d axis;
};
void Draw(const Identity &x) {
  std::cerr << "Identity\n";
}
void Draw(const RevoluteJoint3d &x) {
  std::cerr << "RevoluteJoint3d:\n" << x.axis.transpose() << "\n";
}
void Draw(const Eigen::Matrix3d &x) {
  std::cerr << "Matrix3d:\n" << x << "\n";
}
void Draw(const Eigen::AffineCompact3d &x) {
  std::cerr << "AffineCompact3d:\n" << x.matrix() << "\n";
}
void Draw(const Eigen::Affine3d &x) {
  std::cerr << "Affine3d:\n" << x.matrix() << "\n";
}
void Draw(const Eigen::Translation3d &x) {
  std::cerr << "Translation3d:\n" << x.vector().transpose() << "\n";
}
void Draw(const Eigen::Translation3f &x) {
  std::cerr << "Translation3f:\n" << x.vector().transpose() << "\n";
}

class METAPOD_GRAPH_DLLAPI TransformVisitor;

// Transform Interface class
class Transform {
 public:

  virtual ~Transform() {}
  virtual Transform *clone() const = 0;
  virtual void Accept(TransformVisitor &) const = 0;
};

// Transform implementations Tr<T>
template <typename T>
class METAPOD_GRAPH_DLLAPI Tr : public Transform {
 public:
 T data;
 Tr(T x) : data(x) {}

 Tr<T> *clone() const;
 void Accept(TransformVisitor &vis) const;
};

#define DECL_TR_INSTANCE(r, data, elem)\
  template class METAPOD_GRAPH_DLLAPI elem;
BOOST_PP_SEQ_FOR_EACH(DECL_TR_INSTANCE, ~, TRANSFORM_TYPES_WRAPPED)
#undef DECL_TR_INSTANCE

// TransformVisitor
#define DECL_VISIT_METHOD(r, data, elem)\
  virtual void Visit(const elem &) = 0;

class METAPOD_GRAPH_DLLAPI TransformVisitor {
 public:
  virtual ~TransformVisitor() {}
  BOOST_PP_SEQ_FOR_EACH(DECL_VISIT_METHOD, ~, TRANSFORM_TYPES_WRAPPED)
};
#undef DECL_VISIT_METHOD

template <typename T>
struct SimplifyOperator {
  typedef T Left;
  template <typename Right>
  Transform *operator()(const Left &, const Right &);
};

template <>
struct SimplifyOperator< Tr<Identity> > {
  typedef Tr<Identity> Left;
  template <typename Right>
  Transform *operator()(const Left &, const Right &right) {
    return right.clone();
  }
};

template <>
struct SimplifyOperator< Tr<Eigen::AffineCompact3d> > {
  typedef Tr<Eigen::AffineCompact3d> Left;

  template <typename Right>
  Transform *operator()(const Left &left, const Right &right) {
    return new Left(left.data * right.data);
  }

  Transform *operator()(const Left &left, const Tr<Identity> &right) {
    return left.clone();
  }

  Transform *operator()(const Left &left, const Tr<RevoluteJoint3d> &right) {
    throw std::runtime_error("");
  }
};

template <>
struct SimplifyOperator< Tr<Eigen::Translation3d> > {
  typedef Tr<Eigen::Translation3d> Left;

  Transform *operator()(const Left &left, const Tr<Eigen::AffineCompact3d> &right) {
    return new Tr<Eigen::AffineCompact3d>(left.data * right.data);
  }

  Transform *operator()(const Left &left, const Left &right) {
    return new Left(left.data * right.data);
  }

  Transform *operator()(const Left &left, const Tr<Identity> &right) {
    return left.clone();
  }

  Transform *operator()(const Left &left, const Tr<RevoluteJoint3d> &right) {
    throw std::runtime_error("");
  }
};


template <>
struct SimplifyOperator< Tr<RevoluteJoint3d> > {
  typedef Tr<RevoluteJoint3d> Left;

  Transform *operator()(const Left &left, const Tr<Identity> &) {
    return left.clone();
  }

  template <typename Right>
  Transform *operator()(const Left &, const Right &right) {
    throw std::runtime_error("");
  }
};


#define DEF_VISIT_METHOD(r, data, elem)\
  void Visit(const elem &right) {result.reset(SimplifyOperator<Left>()(left, right));}

template <typename T>
class TypedSimplifyVisitor : public TransformVisitor {
 public:
  typedef T Left;
  Left left;
  typename std::auto_ptr<Transform> result;
 public:
  TypedSimplifyVisitor(T left) : left(left) {}
  BOOST_PP_SEQ_FOR_EACH(DEF_VISIT_METHOD, ~, TRANSFORM_TYPES_WRAPPED)
};
#undef DEF_VISIT_METHOD

#define DEF_VISIT_METHOD(r, data, elem)\
  void Visit(const elem &left) {\
    TypedSimplifyVisitor<elem> vis(left);\
    right.Accept(vis);\
    result = vis.result;\
  }

class SimplifyVisitor : public TransformVisitor {
 public:
  const Transform &right;
  std::auto_ptr<Transform> result;
  SimplifyVisitor(const Transform &right) : right(right) {}
  BOOST_PP_SEQ_FOR_EACH(DEF_VISIT_METHOD, ~, TRANSFORM_TYPES_WRAPPED)
};
#undef DEF_VISIT_METHOD

Transform *Simplify(const Transform &left, const Transform &right) {
  SimplifyVisitor vis(right);
  left.Accept(vis);
  return vis.result.release();
}

/*
Transform Multiply(const Transform &left, const Transform &right){
  return;
}
Transform Multiply(const Transform &left, const Transform &right){
  return;
}
Transform Multiply(const Transform &left, const Transform &right){
  return;
}
Transform Multiply(const Transform &left, const Transform &right){
  return;
}
Transform Multiply(const Transform &left, const Transform &right){
  return;
}
Transform Multiply(const Transform &left, const Transform &right){
  return;
}
Transform Multiply(const Transform &left, const Transform &right){
  return;
}
void Draw(const Eigen::Matrix3d &x) {
  std::cerr << "Matrix3d:\n" << x << "\n";
}
void Draw(const Eigen::AffineCompact3d &x) {
  std::cerr << "AffineCompact3d:\n" << x.matrix() << "\n";
}
void Draw(const Eigen::Affine3d &x) {
  std::cerr << "Affine3d:\n" << x.matrix() << "\n";
}
void Draw(const Eigen::Translation3d &x) {
  std::cerr << "Translation3d:\n" << x.vector().transpose() << "\n";
}
void Draw(const Eigen::Translation3f &x) {
  std::cerr << "Translation3f:\n" << x.vector().transpose() << "\n";
}
*/
/*

class SceneGraph
{
private:
  class VertexProp {
   public:
    VertexProp(const std::string &name = "")
      : color(boost::white_color), name(name) {}
    mutable boost::default_color_type color;
    std::string name;
  };

  class EdgeProp {
   public:
    EdgeProp(const AL::Model::Transform &transform,
             const std::string &name = "")
      : transform(transform), name(name) {}
    AL::Model::Transform transform;
    std::string name;
  };

  typedef boost::adjacency_list<boost::listS, boost::vecS,
                                boost::bidirectionalS,
                                VertexProp, EdgeProp> Graph;

public:
  typedef boost::graph_traits<Graph>::edge_descriptor Edge;
  typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

  SceneGraph();
  ~SceneGraph();
  Vertex AddFrame(const std::string &name = "");
  Edge AddTransform(Vertex source, Vertex target, Transform tr, const std::string &name = "");
  void Draw(Vertex root) const;
private:
  Graph g_;
};
*/
}

#endif
