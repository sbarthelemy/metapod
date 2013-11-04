#include "robotmodel.hh"
#include <cassert>
#include <metapod/tools/constants.hh>
#include <limits>
#include <algorithm>

namespace {

class CompareLinkBodyName {
 public:
  CompareLinkBodyName(const std::string& name)
      : name_(name)
  {}
  bool operator()(const metapod::Link& link)
  {
    if (link.body_name_ == name_)
      return true;
    else
      return false;
  }

 private:
  const std::string name_;
};

class CompareLinkJointName {
 public:
  CompareLinkJointName(const std::string& name)
      : name_(name)
  {}
  bool operator()(const metapod::Link& link)
  {
    if (link.joint_name_ == name_)
      return true;
    else
      return false;
  }

 private:
  const std::string name_;
};

}

namespace metapod {

Link::Link(
    int id,
    int parent_id,
    const std::string& joint_name,
    const RobotBuilder::Joint &joint,
    const Eigen::Matrix3d & R_joint_parent,
    const Eigen::Vector3d & r_parent_joint,
    const std::string& body_name,
    double body_mass,
    const Eigen::Vector3d & body_center_of_mass,
    const Eigen::Matrix3d & body_rotational_inertia)
    : id_(id),
      parent_id_(parent_id),
      joint_name_(joint_name),
      joint_(joint.clone()),
      R_joint_parent_(R_joint_parent),
      r_parent_joint_(r_parent_joint),
      body_name_(body_name),
      body_mass_(body_mass),
      body_center_of_mass_(body_center_of_mass),
      body_rotational_inertia_(body_rotational_inertia) {}

Variable::Variable(const std::set<std::string> &joint_names,
                   unsigned int nb_dof, int dof_index)
    : joint_names(joint_names),
      nb_dof(nb_dof),
      dof_index(dof_index) {}

Variable::Variable(const std::vector<std::string> &joint_names,
                   unsigned int nb_dof, int dof_index)
    : joint_names(joint_names.begin(), joint_names.end()),
      nb_dof(nb_dof),
      dof_index(dof_index) {}

int RobotModel::nb_links() const {
  return static_cast<int>(links_.size());
}

int RobotModel::parent_id(int link_id) const {
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return links_[link_id].parent_id_;
}

const std::string& RobotModel::joint_name(int link_id) const {
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return links_[link_id].joint_name_;
}

const RobotBuilder::Joint &RobotModel::joint(int link_id) const {
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return *(links_[link_id].joint_);
}

const Eigen::Matrix3d& RobotModel::R_joint_parent(int link_id) const {
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return links_[link_id].R_joint_parent_;
}

const Eigen::Vector3d& RobotModel::r_parent_joint(int link_id) const {
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return links_[link_id].r_parent_joint_;
}

const std::string& RobotModel::body_name(int link_id) const {
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return links_[link_id].body_name_;
}

double RobotModel::body_mass(int link_id) const {
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return links_[link_id].body_mass_;
}

const Eigen::Vector3d& RobotModel::body_center_of_mass(int link_id) const {
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return links_[link_id].body_center_of_mass_;
}

const Eigen::Matrix3d& RobotModel::body_rotational_inertia(int link_id) const {
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return links_[link_id].body_rotational_inertia_;
}

int RobotModel::nb_children(int link_id) const {
  if (link_id == NO_PARENT)
    return static_cast<int>(roots_id_.size());
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return static_cast<int>(links_[link_id].child_id_.size());
}

int RobotModel::child_id(int link_id, unsigned int rank) const {
  if (link_id == NO_PARENT) {
    if (rank < roots_id_.size())
      return roots_id_[rank];
    return NO_CHILD;
  }
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  if (rank < links_[link_id].child_id_.size())
    return links_[link_id].child_id_[rank];
  return NO_CHILD;
}

bool RobotModel::RequireVariable(const std::vector<std::string> &joint_names,
                                 unsigned int nb_dof, int dof_index) {
  std::vector<std::vector<Variable>::iterator> sets;
  for (std::vector<Variable>::iterator var = variables_.begin();
       var != variables_.end();
       ++var) {
    bool will_merge = false;
    for (std::vector<std::string>::const_iterator joint = joint_names.begin();
         joint != joint_names.end();
         ++joint) {
      if (var->joint_names.find(*joint) != var->joint_names.end()) {
        // "joint" is bound to "var".
        will_merge = true;
        break;
      }
    }
    if (will_merge) {
      if (nb_dof != var->nb_dof)
        return false; // cannot merge two variables with different nb_dof
      if (var->dof_index != -1) {
        if (dof_index == -1)
          dof_index = var->dof_index;
        else if (dof_index != var->dof_index)
          return false; // cannot merge two variables with different dof indexes
      }
      sets.push_back(var);
    } else {
      if (dof_index != -1 && var-> dof_index !=1 &&
          (dof_index + static_cast<int>(nb_dof) > var-> dof_index &&
           var-> dof_index + static_cast<int>(var->nb_dof) > dof_index))
        return false; // another unrelated variable overlaps our dof indexes
    }
  }
  if (sets.empty()) {
    // none of the provided joints is bound to an existing variable.
    // Create a new one.
    variables_.push_back(Variable(joint_names, nb_dof, dof_index));
  } else {
    // some of the provided joints are bound to one or more variables.
    // We need to merge them, and add the new joint names.
    std::vector<std::vector<Variable>::iterator>::reverse_iterator bwd_it;
    for (bwd_it = sets.rbegin(); bwd_it != sets.rend()-1; ++bwd_it) {
      sets[0]->joint_names.insert((**bwd_it).joint_names.begin(),
                                  (**bwd_it).joint_names.end());
      variables_.erase(*bwd_it);
    }
    sets[0]->joint_names.insert(joint_names.begin(), joint_names.end());
    sets[0]->nb_dof = nb_dof;
    sets[0]->dof_index = dof_index;
  }
  return true;
}

unsigned int RobotModel::nb_dof() const {
  unsigned int nb_dof = 0;
  for (std::vector<Variable>::const_iterator var = variables_.begin();
       var != variables_.end();
       ++var) {
    nb_dof += var->nb_dof;
  }
  return nb_dof;
}

// TODO: unsigned int?
int RobotModel::nb_dof(const std::string &joint_name) const {
  for (std::vector<Variable>::const_iterator var = variables_.begin();
       var != variables_.end();
       ++var) {
    if (std::find(var->joint_names.begin(), var->joint_names.end(), joint_name)
        != var->joint_names.end())
      return var->nb_dof;
  }
  return -1;
}

int RobotModel::dof_index(const std::string &joint_name) const {
  for (std::vector<Variable>::const_iterator var = variables_.begin();
       var != variables_.end();
       ++var) {
    if (std::find(var->joint_names.begin(), var->joint_names.end(), joint_name)
        != var->joint_names.end())
      return var->dof_index;
  }
  return -1;
}

int RobotModel::dof_index(int link_id) const {
  return dof_index(joint_name(link_id));
}

// TODO: keep?
bool RobotModel::FindVariableByJointName(const std::string &name) const {
  for (std::vector<Variable>::const_iterator var = variables_.begin();
       var != variables_.end();
       ++var) {
    if (std::find(var->joint_names.begin(), var->joint_names.end(), name)
        != var->joint_names.end())
      return true;
  }
  return false;
}

bool RobotModel::AssignDofIndexes() {
  //std::vector<int> dofs(nb_dof(), -1);
  std::vector<int> dof_indexes(variables_.size(), -1);
  int dof_index = 0;
  int sup_dof_index = 0;
  unsigned int nb_dof = 0;
  for (size_t i=0; i<variables_.size(); ++i) {
    if (variables_[i].dof_index != -1)
      dof_index = variables_[i].dof_index;
    // If the user has not provided a dof_index, we pick one. Just
    // check the new dof_index does not overlap with the previous ones
    for (size_t j=0; j<i; ++j) {
      if (dof_index+static_cast<int>(variables_[i].nb_dof) > dof_indexes[j] &&
          dof_indexes[j]+static_cast<int>(variables_[j].nb_dof) > dof_index)
        return false; // overlap occurs, fail!
    }
    dof_indexes[i] = dof_index;
    nb_dof += variables_[i].nb_dof;
    dof_index += static_cast<int>(variables_[i].nb_dof); // prepare next round
    sup_dof_index = std::max(sup_dof_index, dof_index);
  }
  if (static_cast<int>(nb_dof) != sup_dof_index)
    return false; // dof indexes are not contiguous
  // success! let commit.the new indexes.
  for (size_t i=0; i<variables_.size(); ++i)
    variables_[i].dof_index = dof_indexes[i];
  return true;
}

void RobotModel::AddLink(const Link &link) {
  // we assume the caller as filled the link with the proper id.
  // We might want to change this policy in the following way: set the id
  // ourselves and return it to the caller.
  assert(link.id_ == static_cast<int>(links_.size()));
  // we use int as index type but store links in a std::vector which uses
  // size_t as index type. Check we won't overflow.
  assert(links_.size() < static_cast<size_t>(std::numeric_limits<int>::max()));
  // there should be a corresponding joint variable, with the good nb_dof
  assert(nb_dof(link.joint_name_) >= 0);
  assert(static_cast<unsigned int>(nb_dof(link.joint_name_)) ==
         link.joint_->nb_dof());

  const int parent_id = link.parent_id_;
  if (parent_id == NO_PARENT) {
    // we use int as index type but store links in a std::vector which uses
    // size_t as index type. Check we won't overflow.
    assert(roots_id_.size() <
        static_cast<size_t>(std::numeric_limits<int>::max()));
    roots_id_.push_back(link.id_);
  } else {
    assert(parent_id >= 0 && static_cast<size_t>(parent_id) < links_.size());
    // we use int as index type but store links in a std::vector which uses
    // size_t as index type. Check we won't overflow.
    assert(links_[parent_id].child_id_.size() <
        static_cast<size_t>(std::numeric_limits<int>::max()));
    links_[parent_id].child_id_.push_back(link.id_);
  }
  links_.push_back(link);
}

int RobotModel::FindLinkByBodyName(const std::string &name) const {
  std::vector<Link>::const_iterator it =
      std::find_if(links_.begin(), links_.end(), ::CompareLinkBodyName(name));
  if (it == links_.end())
    return NO_NODE;
  return it->id_;
}

int RobotModel::FindLinkByJointName(const std::string &name) const {
  std::vector<Link>::const_iterator it =
      std::find_if(links_.begin(), links_.end(), ::CompareLinkJointName(name));
  if (it == links_.end())
    return NO_NODE;
  return it->id_;
}
}
