#include "./kuka_plan_listener.h"

#include <iostream>
#include <memory>

#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/polynomial.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/systems/vector.h"

#include "lcmtypes/drake/lcmt_iiwa_command.hpp"
#include "lcmtypes/drake/lcmt_iiwa_status.hpp"
#include "lcmtypes/drake/robot_plan_t.hpp"

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

RobotPlanRunner::RobotPlanRunner(std::shared_ptr<lcm::LCM> lcm, const RigidBodyTree& tree)
    : lcm_(lcm), tree_(tree), plan_number_(0) {
//  lcm_->subscribe(IiwaStatus<double>::channel(),
//                  &RobotPlanRunner::HandleStatus, this);
  lcm_->subscribe(kLcmPlanChannel,
                  &RobotPlanRunner::HandlePlan, this);
}

void RobotPlanRunner::Run() {
//  int cur_plan_number = plan_number_;
//  int64_t cur_time_ms = -1;
//  int64_t start_time_ms = -1;

  // Initialize the timestamp to an invalid number so we can detect
  // the first message.
//  iiwa_status_.timestamp = cur_time_ms;

//  lcmt_iiwa_command iiwa_command;
//
//  iiwa_command.num_joints = kNumJoints;
//  iiwa_command.joint_position.resize(kNumJoints, 0.);
//  iiwa_command.num_torques = 0;
//  iiwa_command.joint_torque.resize(kNumJoints, 0.);

  while(true) {
    // The argument to handleTimeout is in msec, and should be
    // safely bigger than e.g. a 200Hz input rate.
    int handled  = lcm_->handleTimeout(1000);
    if (handled <= 0) {
      std::cerr << "Failed to receive LCM status. Polling again for 1 second" << std::endl;
      continue;
    }

//  DRAKE_ASSERT(iiwa_status_.timestamp != -1);
//  cur_time_ms = iiwa_status_.timestamp;


  if (plan_.get() != nullptr) {
    std::cout << "Plan received!" << std::endl;
    break;
//      if (plan_number_ != cur_plan_number) {
//          std::cout << "Starting new plan." << std::endl;
//          start_time_ms = cur_time_ms;
//          cur_plan_number = plan_number_;
//        }
//
//        const double cur_traj_time_s =
//            static_cast<double>(cur_time_ms - start_time_ms) / 1e3;
//        const auto desired_next = plan_->value(cur_traj_time_s);
//
//        iiwa_command.timestamp = iiwa_status_.timestamp;
//
//        // This is totally arbitrary.  There's no good reason to
//        // implement this as a maximum delta to submit per tick.  What
//        // we actually need is something like a proper
//        // planner/interpolater which spreads the motion out over the
//        // entire duration from current_t to next_t, and commands the
//        // next position taking into account the velocity of the joints
//        // and the distance remaining.
//        const double max_joint_delta = 0.1;
//        for (int joint = 0; joint < kNumJoints; joint++) {
//          double joint_delta =
//              desired_next(joint) - iiwa_status_.joint_position_measured[joint];
//          joint_delta = std::max(-max_joint_delta,
//                                 std::min(max_joint_delta, joint_delta));
//          iiwa_command.joint_position[joint] =
//              iiwa_status_.joint_position_measured[joint] + joint_delta;
//        }
//
//        lcm_->publish(kLcmCommandChannel, &iiwa_command);
      }
    }
  }


  void RobotPlanRunner::HandleStatus(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                    const lcmt_iiwa_status* status) {
    iiwa_status_ = *status;
  }

  void RobotPlanRunner::HandlePlan(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                  const robot_plan_t* plan) {
    std::cout << "New plan received." << std::endl;

    int ii = 0;

    //Eigen::MatrixXd traj_mat(kNumJoints, plan->num_states);
    std::cout << "RobotPlanPlanner::HandlePlan: Instantiating traj_mat of size [7, "
              << plan->num_states << "]" << std::endl;
    Eigen::MatrixXd traj_mat(7, plan->num_states);

    std::cout << "RobotPlanPlanner::HandlePlan: Zeroing traj_mat." << std::endl;
    traj_mat.fill(0);

    std::cout << "RobotPlanPlanner::HandlePlan: " << ii++ << std::endl;

    std::cout << "RobotPlanPlanner::HandlePlan: calling tree_.computePositionNameToIndexMap()..." << std::endl;
    std::map<std::string, int> name_to_idx =
        tree_.computePositionNameToIndexMap();

    std::cout << "RobotPlanPlanner::HandlePlan: plan->num_states = " << plan->num_states << std::endl;

    for (int i = 0; i < plan->num_states; ++i) {
      const auto& state = plan->plan[i];

      std::cout << "RobotPlanPlanner::HandlePlan: state " << i
                << ", state->num_joints = " << state.num_joints << std::endl;
      for (int j = 0; j < state.num_joints; ++j) {
        if (name_to_idx.count(state.joint_name[j]) == 0) {
          continue;
        }
        traj_mat(name_to_idx[state.joint_name[j]], i) =
            state.joint_position[j];
      }
    }

    std::cout << "RobotPlanPlanner::HandlePlan: " << ii++ << std::endl;

    std::cout << traj_mat << std::endl;

std::cout << "RobotPlanPlanner::HandlePlan: " << ii++ << std::endl;

    std::vector<PPMatrix> polys;
    std::vector<double> times;
    const double kPlanTime = 2.5;

std::cout << "RobotPlanPlanner::HandlePlan: " << ii++ << std::endl;

    // For each timestep, create a PolynomialMatrix for each joint
    // position.  Each column of traj_ represents a particular time,
    // and the rows of that column contain values for each joint
    // coordinate.
    for (int i = 0; i < plan->num_states; i++) {
      PPMatrix poly_matrix(traj_mat.rows(), 1);
      const auto traj_now = traj_mat.col(i);

      // Produce interpolating polynomials for each joint coordinate.
      for (int row = 0; row < traj_mat.rows(); row++) {
        Eigen::Vector2d coeffs(0, 0);
        coeffs[0] = traj_now(row);
        if (i != plan->num_states - 1) {
          // Set the coefficient such that it will reach the value of
          // the next timestep at the time when we advance to the next
          // piece.  In the event that we're at the end of the
          // trajectory, this will be left 0.
          coeffs[1] = ((traj_mat(row, i + 1) - coeffs[0]) /
              (((plan->plan[i + 1].utime * kPlanTime) -
                  (plan->plan[i].utime * kPlanTime)) / 1e6));

        }
        poly_matrix(row) = PPPoly(coeffs);
      }
      polys.push_back(poly_matrix);
      times.push_back((plan->plan[i].utime * kPlanTime) / 1e6);
    }

    std::cout << "RobotPlanPlanner::HandlePlan: " << ii++ << std::endl;


    times.push_back(times.back() + 0.01);
    plan_.reset(new PPType(polys, times));
    ++plan_number_;

    std::cout << "RobotPlanPlanner::HandlePlan: DONE!" << std::endl;

  }

std::unique_ptr<PiecewisePolynomial<double>> RobotPlanRunner::GetReceivedPlan()
{
  auto pp_traj = std::make_unique<PiecewisePolynomial<double>>();
  *pp_traj.get() = *plan_.get();
  return std::move(pp_traj);
}

//void RobotPlanner::ExtractPlanInfo(Eigen::MatrixXd *knot_points,
//                           std::vector<double> *segment_times ) {
//
//  *knot_points = po
//}
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
