// Copyright 2011, 2012,
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

/*
 * This file provides tools to generate metapod robot models.
 */

#ifndef METAPOD_ROBOT_BUILDER_HH
# define METAPOD_ROBOT_BUILDER_HH
# include <metapod/robotbuilder/config.hh>
# include <string>
# include <utility>
# include <fstream>
# include <sstream>
# include <vector>
# include <Eigen/Eigen>

namespace metapod {

class  METAPOD_ROBOTBUILDER_DLLAPI RobotBuilder
{
public:
  enum JointType { FREE_FLYER, REVOLUTE_AXIS_X, REVOLUTE_AXIS_ANY };
  enum Status
  {
    STATUS_SUCCESS = 0,
    STATUS_FAILURE = 1
  };
  RobotBuilder();
  ~RobotBuilder();
  Status set_name(const std::string& name);
  Status set_libname(const std::string& libname);
  Status set_directory(const std::string& directory);
  Status set_namespace(const std::string& combined_namespace);
  Status set_reinclusion_guard_prefix(const std::string& text);
  Status set_license(const std::string& text);
  Status init();

  // R_joint_parent is the rotation matrix which converts vector from the
  // parent body frame to the joint frame coordinate systems.
  // r_parent_joint is a 3D vector giving the position of the joint frame
  // origin in the parent frame coordinate system.
  //
  // Thus if p_parent is the coordinates vector of the point p in the parent
  // frame, then p_joint is given by
  //
  //   p_joint = R_joint_parent * (p_parent - r_parent_joint)
  Status addLink(
      const std::string& parent_body_name,
      const std::string& joint_name,
      unsigned int joint_type,
      const Eigen::Matrix3d & R_joint_parent,
      const Eigen::Vector3d & r_parent_joint,
      const std::string& body_name,
      double body_mass,
      const Eigen::Vector3d & body_center_of_mass,
      const Eigen::Matrix3d & body_rotational_inertia,
      const Eigen::Vector3d & joint_axis=defaultAxis());
private:
  static Eigen::Vector3d defaultAxis();
  void openInclusionGuard(std::ostream& stream, const char* name);
  void closeInclusionGuard(std::ostream& stream, const char* name);
  void openNamespace(std::ostream& stream);
  void closeNamespace(std::ostream& stream);
  void closeNode();
  RobotBuilder(const RobotBuilder&); // forbid copy-constuction
  unsigned int nb_dof_;
  unsigned int nb_bodies_;
  unsigned int node_depth_;
  bool is_initialized_;
  const size_t tab_size_;
  const size_t node_tab_size_;
  const std::string tab_;
  const std::string warning_;
  std::vector< std::pair<std::string, int> > bodies_stack_;
  std::string name_;
  std::string libname_;
  std::string directory_;
  std::string reinclusion_guard_prefix_;
  std::string namespace_;
  std::string license_;
  std::vector<std::string> namespaces_;
  std::string namespaces_opening_;
  std::string namespaces_closing_;
  std::ofstream body_hh_;
  std::ofstream init_cc_;
  std::ofstream init_hh_;
  std::ofstream joint_hh_;
  std::ofstream robot_hh_;
  std::ostringstream tree_;
};
}
#endif
