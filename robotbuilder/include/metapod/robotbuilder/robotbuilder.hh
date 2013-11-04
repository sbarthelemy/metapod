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

/*
 * This file provides tools to generate metapod robot models.
 */

#ifndef METAPOD_ROBOT_BUILDER_HH
# define METAPOD_ROBOT_BUILDER_HH

# include <metapod/robotbuilder/config.hh>
# include <string>
# include <vector>
# include <Eigen/Dense>

namespace metapod {

class RobotBuilderP; // private implementation

class  METAPOD_ROBOTBUILDER_DLLAPI RobotBuilder {
 public:
  class Joint {
   public:
    virtual Joint* clone() const = 0;
    virtual unsigned int nb_dof() const = 0;
    virtual std::string joint_class() const = 0;
    virtual std::string rotation_class() const = 0;
    virtual std::string ctor_args() const {return std::string();}
  };

  class FreeFlyerJoint : public Joint {
   public:
    FreeFlyerJoint() {}
    Joint* clone() const {return new FreeFlyerJoint(*this);}
    unsigned int nb_dof() const {return 6;}
    std::string joint_class() const {return "FreeFlyerJoint";}
    std::string rotation_class() const {
      return "Spatial::RotationMatrix";}
  };

  class FreeFlyerBodyJoint : public Joint {
   public:
    FreeFlyerBodyJoint() {}
    Joint* clone() const {return new FreeFlyerBodyJoint(*this);}
    unsigned int nb_dof() const {return 6;}
    std::string joint_class() const {return "FreeFlyerBodyJoint";}
    std::string rotation_class() const {
      return "Spatial::RotationMatrix";}
  };

  class RevoluteAxisXJoint : public Joint {
   public:
    RevoluteAxisXJoint() {}
    Joint* clone() const {return new RevoluteAxisXJoint(*this);}
    unsigned int nb_dof() const {return 1;}
    std::string joint_class() const {return "RevoluteAxisXJoint";}
    std::string rotation_class() const {
      return "Spatial::RotationMatrixAboutX";}
  };

  class RevoluteAxisYJoint : public Joint {
   public:
    RevoluteAxisYJoint() {}
    Joint* clone() const {return new RevoluteAxisYJoint(*this);}
    unsigned int nb_dof() const {return 1;}
    std::string joint_class() const {return "RevoluteAxisYJoint";}
    std::string rotation_class() const {
      return "Spatial::RotationMatrixAboutY";}
  };

  class RevoluteAxisZJoint : public Joint {
   public:
    RevoluteAxisZJoint() {}
    Joint* clone() const {return new RevoluteAxisZJoint(*this);}
    unsigned int nb_dof() const {return 1;}
    std::string joint_class() const {return "RevoluteAxisZJoint";}
    std::string rotation_class() const {
      return "Spatial::RotationMatrixAboutZ";}
  };

  class RevoluteAxisAnyJoint : public Joint {
   public:
    RevoluteAxisAnyJoint(const Eigen::Vector3d &axis):
      axis_(axis) {}
    Joint* clone() const {return new RevoluteAxisAnyJoint(*this);}
    unsigned int nb_dof() const {return 1;}
    std::string joint_class() const {return "RevoluteAxisAnyJoint";}
    std::string rotation_class() const {
      return "Spatial::RotationMatrix";}
    std::string ctor_args() const {
      Eigen::IOFormat comma_fmt(Eigen::StreamPrecision, Eigen::DontAlignCols,
                                ", ", ", ");
      std::stringstream ss;
      ss << axis_.format(comma_fmt);
      return ss.str();
    }

   private:
    const Eigen::Vector3d axis_;
  };

  enum JointType {
    FREE_FLYER,
    FREE_FLYER_BODY,
    REVOLUTE_AXIS_X,
    REVOLUTE_AXIS_Y,
    REVOLUTE_AXIS_Z,
    REVOLUTE_AXIS_ANY };
  enum Status {
    STATUS_SUCCESS = 0,
    STATUS_FAILURE = 1
  };
  RobotBuilder();
  ~RobotBuilder();
  Status set_name(const std::string &name);
  Status set_libname(const std::string &libname);
  Status set_directory(const std::string &directory);
  Status set_license(const std::string &text);
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
  // parent_body_name: "NP" (no parent) or the parent body name
  //
  // R_joint_parent is the rotation matrix which converts vector from the
  // parent body frame to the joint frame coordinate systems.
  // r_parent_joint is a 3D vector giving the position of the joint frame
  // origin in the parent frame coordinate system.
  //
  // Thus if p_parent is the coordinates vector of the point p in the parent
  // frame, then p_joint is given by
  //
  //   p_joint = R_joint_parent * (p_parent - r_parent_joint)
  Status AddLink(
      const std::string &parent_body_name,
      const std::string &joint_name,
      const Joint &joint,
      const Eigen::Matrix3d &R_joint_parent,
      const Eigen::Vector3d &r_parent_joint,
      const std::string &body_name,
      double body_mass,
      const Eigen::Vector3d &body_center_of_mass,
      const Eigen::Matrix3d &body_rotational_inertia);
  Status Write();
 private:
  // returns [1.; 0.; 0.]
  static Eigen::Vector3d axisX();
  RobotBuilderP *const pimpl_;
  RobotBuilder(const RobotBuilder &); // forbid copy-constuction
  RobotBuilder& operator=(const RobotBuilder &); // forbid assignment
};
}
#endif
