@LICENSE@
// This file has been generated by the metapod robotbuilder library.

# include "@ROBOT_NAME@.hh"

namespace metapod {
namespace @ROBOT_NAMESPACE@ {

  // Initialization of the robot global constants
  Eigen::Matrix< FloatType, Robot::NBDOF, Robot::NBDOF > Robot::H;

@init_joints_and_bodies@

} // closing namespace @ROBOT_NAMESPACE@
} // closing namespace metapod
