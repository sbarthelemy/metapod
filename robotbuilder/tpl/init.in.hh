@LICENSE@
// This file has been generated by the metapod robotbuilder library.

#ifndef @LIBRARY_NAME@_INIT_HH
# define @LIBRARY_NAME@_INIT_HH

# ifdef _MSC_VER
#  pragma warning( push )
// disable warning C4251: need to have DLL interface
// disable warning C4099: struct/class discrepancies
#  pragma warning( disable: 4251 4099 )
# endif

# include "config.hh"

# include <metapod/tools/common.hh>
# include <metapod/tools/joint.hh>

// by default, boost fusion vector only provides constructor for vectors with
// up to 10 elements.
# if !defined(FUSION_MAX_VECTOR_SIZE) && (@ROBOT_NB_BODIES@ > 10)
#  define FUSION_MAX_VECTOR_SIZE @ROBOT_NB_BODIES@
# endif
# if defined(FUSION_MAX_VECTOR_SIZE) && (@ROBOT_NB_BODIES@ > FUSION_MAX_VECTOR_SIZE)
// todo: warn or stop
# endif
# include <boost/fusion/sequence.hpp>
# include <boost/fusion/include/sequence.hpp>
# include <boost/fusion/include/vector.hpp>

namespace metapod {

class @LIBRARY_NAME@_DLLAPI @ROBOT_CLASS_NAME@ {
public:
  // Global constants or variable of the robot
  enum { NBDOF = @ROBOT_NB_DOF@ };
  enum { NBBODIES = @ROBOT_NB_BODIES@ };

  typedef Eigen::Matrix< FloatType, NBDOF, 1 > confVector;

  enum NodeId
  {
@nodeid_enum_definition@
  };

  // children of the root/NP node
  static const int child0_id = @root_child0_id@;
  static const int child1_id = @root_child1_id@;
  static const int child2_id = @root_child2_id@;
  static const int child3_id = @root_child3_id@;
  static const int child4_id = @root_child4_id@;

  // definition of the node classes (except the root/NP node)
@node_type_definitions@

  // vector of the robot nodes
  typedef boost::fusion::vector@ROBOT_NB_BODIES@<
@nodes_type_list@>
  NodeVector;

  // member variables

  // inertias expressed in body frames
  static Spatial::Inertia inertias[@ROBOT_NB_BODIES@];
  Body bodies[@ROBOT_NB_BODIES@];
  NodeVector nodes;
  Eigen::Matrix< FloatType, NBDOF, NBDOF > H; // used by crba

  @ROBOT_CLASS_NAME@():
    H(Eigen::Matrix< FloatType, NBDOF, NBDOF >::Zero())
  {}

  Spatial::Inertia & I(int node_id) {
    return inertias[node_id];
  }

  const Spatial::Inertia & I(int node_id) const {
    return inertias[node_id];
  }

  Spatial::Inertia & Iic(int node_id) {
    return bodies[node_id].Iic;
  }

  const Spatial::Inertia & Iic(int node_id) const {
    return bodies[node_id].Iic;
  }

  Spatial::Motion & vi(int node_id) {
    return bodies[node_id].vi;
  }

  const Spatial::Motion & vi(int node_id) const {
    return bodies[node_id].vi;
  }

  Spatial::Motion & ai(int node_id) {
    return bodies[node_id].ai;
  }

  const Spatial::Motion & ai(int node_id) const {
    return bodies[node_id].ai;
  }

  const Spatial::Force & Fext(int node_id) const {
    return bodies[node_id].Fext;
  }

  Spatial::Transform & iX0(int node_id) {
    return bodies[node_id].iX0;
  }

  const Spatial::Transform & iX0(int node_id) const {
    return bodies[node_id].iX0;
  }
};

// map node id to node type
@map_node_id_to_type@

} // closing namespace metapod

# ifdef _MSC_VER
#  pragma warning( pop )
# endif

#endif
