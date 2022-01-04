/// @file
///
/// kuka_plan_runner is designed to wait for LCM messages constraining
/// a lcmt_robot_plan message, and then execute the plan on an iiwa arm
/// (also communicating via LCM using the
/// lcmt_iiwa_command/lcmt_iiwa_status messages).
///
/// When a plan is received, it will immediately begin executing that
/// plan on the arm (replacing any plan in progress).
///
/// If a stop message is received, it will immediately discard the
/// current plan and wait until a new plan is received.

#include <string.h>

#include <iostream>
#include <memory>

#include "lcm/lcm-cpp.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_robot_plan.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmCommandChannel = "IIWA_COMMAND";
const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";
const char* const kLcmStopChannel = "STOP";
const int kNumJoints = 7;

using trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;
typedef Polynomial<double> PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

class RobotPlanRunner {
 public:
  /// plant is aliased
  explicit RobotPlanRunner(const multibody::MultibodyPlant<double>& plant)
      : plant_(plant), plan_number_(0) {
    lcm_.subscribe(kLcmStatusChannel,
                    &RobotPlanRunner::HandleStatus, this);
    lcm_.subscribe(kLcmPlanChannel,
                    &RobotPlanRunner::HandlePlan, this);
    lcm_.subscribe(kLcmStopChannel,
                    &RobotPlanRunner::HandleStop, this);
  }

  void Run() {
    int cur_plan_number = plan_number_;
    int64_t cur_time_us = -1;
    int64_t start_time_us = -1;

    // Initialize the timestamp to an invalid number so we can detect
    // the first message.
    iiwa_status_.utime = cur_time_us;

    lcmt_iiwa_command iiwa_command;
    iiwa_command.num_joints = kNumJoints;
    iiwa_command.joint_position.resize(kNumJoints, 0.);
    iiwa_command.num_torques = 0;
    iiwa_command.joint_torque.resize(kNumJoints, 0.);

    while (true) {
      // Call lcm handle until at least one status message is
      // processed.
      while (0 == lcm_.handleTimeout(10) || iiwa_status_.utime == -1) { }

      cur_time_us = iiwa_status_.utime;

      if (plan_) {
        if (plan_number_ != cur_plan_number) {
          std::cout << "Starting new plan." << std::endl;
          start_time_us = cur_time_us;
          cur_plan_number = plan_number_;
        }

        const double cur_traj_time_s =
            static_cast<double>(cur_time_us - start_time_us) / 1e6;
        const auto desired_next = plan_->value(cur_traj_time_s);

        iiwa_command.utime = iiwa_status_.utime;

        for (int joint = 0; joint < kNumJoints; joint++) {
          iiwa_command.joint_position[joint] = desired_next(joint);
        }

        lcm_.publish(kLcmCommandChannel, &iiwa_command);
      }
    }
  }

 private:
  void HandleStatus(const ::lcm::ReceiveBuffer*, const std::string&,
                    const lcmt_iiwa_status* status) {
    iiwa_status_ = *status;
  }

  void HandlePlan(const ::lcm::ReceiveBuffer*, const std::string&,
                  const lcmt_robot_plan* plan) {
    std::cout << "New plan received." << std::endl;
    if (iiwa_status_.utime == -1) {
      std::cout << "Discarding plan, no status message received yet"
                << std::endl;
      return;
    } else if (plan->num_states < 2) {
      std::cout << "Discarding plan, Not enough knot points." << std::endl;
      return;
    }

    std::vector<Eigen::MatrixXd> knots(plan->num_states,
                                       Eigen::MatrixXd::Zero(kNumJoints, 1));
    for (int i = 0; i < plan->num_states; ++i) {
      const auto& state = plan->plan[i];
      for (int j = 0; j < state.num_joints; ++j) {
        if (!plant_.HasJointNamed(state.joint_name[j])) {
          continue;
        }
        const multibody::Joint<double>& joint =
            plant_.GetJointByName(state.joint_name[j]);
        DRAKE_DEMAND(joint.num_positions() == 1);
        const int idx = joint.position_start();
        DRAKE_DEMAND(idx < kNumJoints);

        // Treat the matrix at knots[i] as a column vector.
        if (i == 0) {
          // Always start moving from the position which we're
          // currently commanding.
          DRAKE_DEMAND(iiwa_status_.utime != -1);
          knots[0](idx, 0) = iiwa_status_.joint_position_commanded[j];

        } else {
          knots[i](idx, 0) = state.joint_position[j];
        }
      }
    }

    for (int i = 0; i < plan->num_states; ++i) {
      std::cout << knots[i] << std::endl;
    }

    std::vector<double> input_time;
    for (int k = 0; k < static_cast<int>(plan->plan.size()); ++k) {
      input_time.push_back(plan->plan[k].utime / 1e6);
    }
    const Eigen::MatrixXd knot_dot = Eigen::MatrixXd::Zero(kNumJoints, 1);
    plan_.reset(new PiecewisePolynomial<double>(
        PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
            input_time, knots, knot_dot, knot_dot)));
    ++plan_number_;
  }

  void HandleStop(const ::lcm::ReceiveBuffer*, const std::string&,
                  const lcmt_robot_plan*) {
    std::cout << "Received stop command. Discarding plan." << std::endl;
    plan_.reset();
  }

  ::lcm::LCM lcm_;
  const multibody::MultibodyPlant<double>& plant_;
  int plan_number_{};
  std::unique_ptr<PiecewisePolynomial<double>> plan_;
  lcmt_iiwa_status iiwa_status_;
};

int do_main() {
  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser(&plant).AddModelFromFile(
      FindResourceOrThrow("drake/manipulation/models/iiwa_description/urdf/"
                          "iiwa14_no_collision.urdf"));
  plant.WeldFrames(plant.world_frame(),
                   plant.GetBodyByName("base").body_frame());
  plant.Finalize();

  RobotPlanRunner runner(plant);
  runner.Run();
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake


int main(int /* argc */, char* argv[]) {
  // TODO(jwnimmer-tri) On 2022-01-01 once this deprecation date has passed,
  // revert the portion of the patch that changed this file.
  if (::strstr(argv[0], "bazel-bin/") == NULL) {
    drake::log()->warn(
        "The use of kuka_plan_runner outside of Drake"
        " (i.e., via 'make install'  or a pre-compiled release image)"
        " is deprecated and will be removed from the install"
        " on or after 2022-01-01");
  }
  return drake::examples::kuka_iiwa_arm::do_main();
}
