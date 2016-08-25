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
class TrajectoryRunner {
 public:
  typedef PiecewisePolynomial<double> PPType;
  typedef PPType::PolynomialType PPPoly;
  typedef PPType::PolynomialMatrix PPMatrix;

  TrajectoryRunner(
      const std::vector<RigidBodyConstraint*> &constraint_array)
      : constraint_array_(constraint_array) {

  }

  PPType GenerateTrajectoryPolynomial{
      IKoptions ikoptions(&tree);
      int info[kNumTimesteps];
      MatrixXd q_sol(tree.number_of_positions(), kNumTimesteps);
      std::vector<std::string> infeasible_constraint;

      inverseKinPointwise(&tree, kNumTimesteps, t, q0, q0, constraint_array.size(),
      constraint_array.data(), ikoptions, &q_sol, info,
      &infeasible_constraint);
      bool info_good = true;
      for (int i = 0; i < kNumTimesteps; i++) {
        printf("INFO[%d] = %d ", i, info[i]);
        if (info[i] != 1) {
          info_good = false;
        }
      }
      printf("\n");

      if (!info_good) {
        std::cerr << "Solution failed, not sending." << std::endl;
        return 1;
      }


      std::vector <PPMatrix> polys;
      std::vector<double> times;

      // For each timestep, create a PolynomialMatrix for each joint
      // position.  Each column of traj_ represents a particular time,
      // and the rows of that column contain values for each joint
      // coordinate.
      for (int i = 0; i < nT_; i++) {
        PPMatrix poly_matrix(traj_.rows(), 1);
        const auto traj_now = traj_.col(i);

        // Produce interpolating polynomials for each joint coordinate.
        for (int row = 0; row < traj_.rows(); row++) {
          Eigen::Vector2d coeffs(0, 0);
          coeffs[0] = traj_now(row);
          if (i != nT_ - 1) {
            // Set the coefficient such that it will reach the value of
            // the next timestep at the time when we advance to the next
            // piece.  In the event that we're at the end of the
            // trajectory, this will be left 0.
            coeffs[1] = (traj_(row, i + 1) - coeffs[0]) / (t_[i + 1] - t_[i]);
          }
          poly_matrix(row) = PPPoly(coeffs);
        }
        polys.push_back(poly_matrix);
        times.push_back(t_[i]);
      }

      PPType pp_traj(polys, times);

      bool time_initialized = false;
      int64_t start_time_ms = -1;
      int64_t cur_time_ms = -1;
      const int64_t end_time_offset_ms = (t_[nT_ - 1] * 1e3);
      DRAKE_ASSERT(end_time_offset_ms > 0);

      return(pp_traj);
  }

  bool ResetConstraintArray(
      const std::vector<RigidBodyConstraint*> &constraint_array) {
    return(true);
  }




//  TrajectoryRunner(std::shared_ptr <lcm::LCM> lcm, int nT, const double *t,
//                   const Eigen::MatrixXd &traj)
//      : lcm_(lcm), nT_(nT), t_(t), traj_(traj) {
//    lcm_->subscribe(IiwaStatus<double>::channel(),
//                    &TrajectoryRunner::HandleStatus, this);
//    DRAKE_ASSERT(traj_.cols() == nT);
//  }
//
//  void Run() {
//    typedef PiecewisePolynomial<double> PPType;
//    typedef PPType::PolynomialType PPPoly;
//    typedef PPType::PolynomialMatrix PPMatrix;
//    std::vector <PPMatrix> polys;
//    std::vector<double> times;
//
//    // For each timestep, create a PolynomialMatrix for each joint
//    // position.  Each column of traj_ represents a particular time,
//    // and the rows of that column contain values for each joint
//    // coordinate.
//    for (int i = 0; i < nT_; i++) {
//      PPMatrix poly_matrix(traj_.rows(), 1);
//      const auto traj_now = traj_.col(i);
//
//      // Produce interpolating polynomials for each joint coordinate.
//      for (int row = 0; row < traj_.rows(); row++) {
//        Eigen::Vector2d coeffs(0, 0);
//        coeffs[0] = traj_now(row);
//        if (i != nT_ - 1) {
//          // Set the coefficient such that it will reach the value of
//          // the next timestep at the time when we advance to the next
//          // piece.  In the event that we're at the end of the
//          // trajectory, this will be left 0.
//          coeffs[1] = (traj_(row, i + 1) - coeffs[0]) / (t_[i + 1] - t_[i]);
//        }
//        poly_matrix(row) = PPPoly(coeffs);
//      }
//      polys.push_back(poly_matrix);
//      times.push_back(t_[i]);
//    }
//
//    PPType pp_traj(polys, times);
//
//    bool time_initialized = false;
//    int64_t start_time_ms = -1;
//    int64_t cur_time_ms = -1;
//    const int64_t end_time_offset_ms = (t_[nT_ - 1] * 1e3);
//    DRAKE_ASSERT(end_time_offset_ms > 0);
//
//    lcmt_iiwa_command iiwa_command;
//    iiwa_command.num_joints = kNumJoints;
//    iiwa_command.joint_position.resize(kNumJoints, 0.);
//    iiwa_command.num_torques = 0;
//    iiwa_command.joint_torque.resize(kNumJoints, 0.);
//
//    while (!time_initialized ||
//        cur_time_ms < (start_time_ms + end_time_offset_ms)) {
//      // The argument to handleTimeout is in msec, and should be
//      // safely bigger than e.g. a 200Hz input rate.
//      int handled = lcm_->handleTimeout(10);
//      if (handled <= 0) {
//        std::cerr << "Failed to receive LCM status." << std::endl;
//        return;
//      }
//
//      if (!time_initialized) {
//        start_time_ms = iiwa_status_.timestamp;
//        time_initialized = true;
//      }
//      cur_time_ms = iiwa_status_.timestamp;
//
//      const double cur_traj_time_s =
//          static_cast<double>(cur_time_ms - start_time_ms) / 1e3;
//      const auto desired_next = pp_traj.value(cur_traj_time_s);
//
//      iiwa_command.timestamp = iiwa_status_.timestamp;
//
//      // This is totally arbitrary.  There's no good reason to
//      // implement this as a maximum delta to submit per tick.  What
//      // we actually need is something like a proper
//      // planner/interpolater which spreads the motion out over the
//      // entire duration from current_t to next_t, and commands the
//      // next position taking into account the velocity of the joints
//      // and the distance remaining.
//      const double max_joint_delta = 0.1;
//      for (int joint = 0; joint < kNumJoints; joint++) {
//        double joint_delta =
//            desired_next(joint) - iiwa_status_.joint_position_measured[joint];
//        joint_delta = std::max(-max_joint_delta,
//                               std::min(max_joint_delta, joint_delta));
//        iiwa_command.joint_position[joint] =
//            iiwa_status_.joint_position_measured[joint] + joint_delta;
//      }
//
//      lcm_->publish(kLcmCommandChannel, &iiwa_command);
//    }
//  }

 private:

  void InterpolateTrajectory() const;
//  void HandleStatus(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
//                    const lcmt_iiwa_status *status) {
//    iiwa_status_ = *status;
//  }

  std::vector<RigidBodyConstraint*> constraint_array_;
  static const int kNumJoints = 7;
//  std::shared_ptr <lcm::LCM> lcm_;
//  const int nT_;
//  const double *t_;
  const Eigen::MatrixXd &traj_;
//  lcmt_iiwa_status iiwa_status_;
};

}
}
}