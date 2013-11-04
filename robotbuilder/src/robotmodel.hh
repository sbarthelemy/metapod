// Copyright 2013
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

#ifndef METAPOD_ROBOTMODEL_HH
# define METAPOD_ROBOTMODEL_HH

# include <string>
# include <Eigen/Dense>
# include <vector>
# include <set>
# include <boost/shared_ptr.hpp>
# include <metapod/robotbuilder/robotbuilder.hh>

namespace metapod {

class Link {
 public:
  int id_;
  int parent_id_;
  std::string joint_name_;
  boost::shared_ptr<RobotBuilder::Joint> joint_;
  Eigen::Matrix3d R_joint_parent_;
  Eigen::Vector3d r_parent_joint_;
  std::string body_name_;
  double body_mass_;
  Eigen::Vector3d body_center_of_mass_;
  Eigen::Matrix3d body_rotational_inertia_;
  Eigen::Vector3d joint_axis_;
  std::vector<int> child_id_; // children

  Link(
    int id,
    int parent_id,
    const std::string& joint_name,
    const RobotBuilder::Joint &joint,
    const Eigen::Matrix3d & R_joint_parent,
    const Eigen::Vector3d & r_parent_joint,
    const std::string& body_name,
    double body_mass,
    const Eigen::Vector3d & body_center_of_mass,
    const Eigen::Matrix3d & body_rotational_inertia);
};

class Variable {
 public:
  std::set<std::string> joint_names;
  unsigned int nb_dof;
  int dof_index;
  Variable(const std::set<std::string> &joint_names, unsigned int nb_dof,
           int dof_index=-1);
  Variable(const std::vector<std::string> &joint_names, unsigned int nb_dof,
           int dof_index=-1);
};


class RobotModel {
 public:
  int nb_links() const; // root body (mass-less galilean frame) does not count
  int parent_id(int link_id) const;
  const std::string &joint_name(int link_id) const;
  const RobotBuilder::Joint &joint(int link_id) const;
  const Eigen::Matrix3d &R_joint_parent(int link_id) const;
  const Eigen::Vector3d &r_parent_joint(int link_id) const;
  const std::string &body_name(int link_id) const;
  double body_mass(int link_id) const;
  const Eigen::Vector3d &body_center_of_mass(int link_id) const;
  const Eigen::Matrix3d &body_rotational_inertia(int link_id) const;
  const Eigen::Vector3d &joint_axis(int link_id) const;
  int nb_children(int link_id) const;
  int child_id(int link_id, unsigned int rank) const;
  bool RequireVariable(const std::vector<std::string> &joint_names,
                       unsigned int nb_dof, int dof_index=-1);
  // return the number of degrees of freedom of the joint variable bound to the
  // joint, or -1 if the joint is not found
  int nb_dof(const std::string &joint_name) const;
  int dof_index(const std::string &joint_name) const;
  int dof_index(int link_id) const;
  // return the number of degrees of freedom of all the joint variables
  unsigned int nb_dof() const;
  bool FindVariableByJointName(const std::string &name) const;
  void AddLink(const Link &link);
  int FindLinkByBodyName(const std::string &name) const;
  int FindLinkByJointName(const std::string &name) const;
  // For all joint variables, where dof_index==-1, assign a dof_index.
  // Could fail if the dof indexes it chooses overlap. In such a case the user
  // Can add the variable in another order or set the dof_index explicitly.
  // Retrun true in case of success
  bool AssignDofIndexes();

 private:
  std::vector<int> roots_id_;
  std::vector<Link> links_; // link_id -> Link mapping
  std::vector<Variable> variables_;
};

}
#endif
