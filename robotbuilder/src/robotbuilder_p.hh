// Copyright 2011, 2012, 2013
//
// Maxime Reis (JRL/LAAS, CNRS/AIST)
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

#ifndef METAPOD_ROBOT_BUILDER_P_HH
# define METAPOD_ROBOT_BUILDER_P_HH

# include <metapod/robotbuilder/robotbuilder.hh>

# include <string>
# include <utility>
# include <fstream>
# include <sstream>
# include <vector>
# include <map>
# include "robotmodel.hh"

namespace metapod {

class RobotBuilderP {
 public:
  RobotBuilderP();
  ~RobotBuilderP();
  RobotBuilder::Status set_name(const std::string &name);
  RobotBuilder::Status set_libname(const std::string &libname);
  RobotBuilder::Status set_directory(const std::string &directory);
  RobotBuilder::Status set_root_body_name(const std::string &name);
  RobotBuilder::Status set_license(const std::string &text);
  RobotBuilder::Status Init();
  // Ensure there is a joint variable bound to the joints whose names are
  // given, using the (optionally) provided dof_index.
  // If no such a variable already exists, it will be created. If several
  // variables are bound to the listed joints, they will get merged.
  // The call will fail if several conflicting dof_indexes were requested.
  //
  // Note that calling this method is only needed if you want to declare
  // joints as coupled (they will share a single joint variable) or force the
  // dof index.
  RobotBuilder::Status RequireJointVariable(
      const std::vector<std::string> &joints_names,
      unsigned int nb_dof,
      int dof_index=-1);
  // R_joint_parent is the rotation matrix which converts vector from the
  // parent body frame to the joint frame coordinate systems.
  // r_parent_joint is a 3D vector giving the position of the joint frame
  // origin in the parent frame coordinate system.
  //
  // Thus if p_parent is the coordinates vector of the point p in the parent
  // frame, then p_joint is given by
  //
  //   p_joint = R_joint_parent * (p_parent - r_parent_joint)
  RobotBuilder::Status AddLink(
      const std::string &parent_body_name,
      const std::string &joint_name,
      const RobotBuilder::Joint &joint,
      const Eigen::Matrix3d &R_joint_parent,
      const Eigen::Vector3d &r_parent_joint,
      const std::string &body_name,
      double body_mass,
      const Eigen::Vector3d &body_center_of_mass,
      const Eigen::Matrix3d &body_rotational_inertia);
  RobotBuilder::Status Write();

 private:
  typedef std::map<std::string, std::string> ReplMap;
  typedef std::pair<int, int> IntPair;
  void GenCrbaLink(std::ostream &os, std::set<IntPair> &written_blocks,
                   int node_id) const;
  std::string GenCrba(const ReplMap &replacements) const;
  // Tuple-like to hold the stream in which the link are written
  struct TmpStreams {
    // for content in robot.hh
    std::ostringstream node_type_definitions;
    std::ostringstream nodeid_enum_definition;
    std::ostringstream nodes_type_list;
    std::ostringstream map_node_id_to_rtnode;
    // for content in init.cc
    std::ostringstream init_nodes;
    std::ostringstream init_inertias;
  };
  void WriteLink(int link_id, const ReplMap &replacements,
                 TmpStreams &out) const;
  void WriteTemplate(const std::string &output_filename,
                     const std::string &input_template,
                     const ReplMap &replacements) const;
  RobotBuilderP(const RobotBuilderP &); // forbid copy-constuction
  bool is_initialized_;
  RobotModel model_;
  std::string name_;
  std::string libname_;
  std::string directory_;
  std::string root_body_name_;
  std::string license_;
};
}
#endif
