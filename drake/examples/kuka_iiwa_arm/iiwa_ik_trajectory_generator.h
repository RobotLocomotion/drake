# pragma once

#include <iostream>

#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/polynomial.h"
#include "drake/systems/plants/IKoptions.h"
#include "drake/systems/plants/RigidBodyIK.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/constraint/RigidBodyConstraint.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/systems/vector.h"

#include "lcmtypes/drake/lcmt_iiwa_command.hpp"
#include "lcmtypes/drake/lcmt_iiwa_status.hpp"

#include "iiwa_status.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

const char *kLcmCommandChannel = "IIWA_COMMAND";

/// This is a really simple demo class to run a trajectory which is
/// the output of an IK plan.  It lacks a lot of useful things, like a
/// controller which does a remotely good job of mapping the
/// trajectory onto the robot.  The paramaters @p nT and @p t are
/// identical to their usage for inverseKinPointwise (@p nT is number
/// of time samples and @p t is an array of times in seconds).
class TrajectoryGenerator {
 public:

  typedef PiecewisePolynomial<double> PPType;
  typedef PPType::PolynomialType PPPoly;
  typedef PPType::PolynomialMatrix PPMatrix;

  TrajectoryGenerator(
      const std::vector<RigidBodyConstraint*> &constraint_array,
      const std::vector<double> &time_stamps,
      const std::shared_ptr<RigidBodyTree> &iiwa_tree
      );


  PiecewisePolynomial<double> GenerateTrajectoryPolynomial();

//  bool ResetConstraintArray(
//      const std::vector<RigidBodyConstraint*> &constraint_array) {
//    return(true);
//  }


  std::vector<RigidBodyConstraint*> constraint_array_;
  static const int kNumJoints = 7;
//  std::shared_ptr <lcm::LCM> lcm_;
//  const int nT_;
//  const double *t_;
  std::vector<double> time_stamps_;
  //Eigen::MatrixXd traj_;
  std::shared_ptr<RigidBodyTree> iiwa_tree_;
  int num_constraints_;
//  lcmt_iiwa_status iiwa_status_;
};

}
}
}