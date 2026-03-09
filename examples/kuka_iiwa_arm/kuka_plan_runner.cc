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

#include <iostream>
#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/lcmt_robot_plan.hpp"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

using drake::Vector1d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXi;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

const char* const kLcmStatusChannel = "IIWA_STATUS";
const char* const kLcmCommandChannel = "IIWA_COMMAND";
const char* const kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";
const char* const kLcmStopChannel = "STOP";
const int kNumJoints = 7;

using lcm::DrakeLcm;
using lcm::Subscriber;
using trajectories::PiecewisePolynomial;
typedef PiecewisePolynomial<double> PPType;
typedef Polynomial<double> PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

class RobotPlanRunner {
 public:
  /// plant is aliased
  explicit RobotPlanRunner(const multibody::MultibodyPlant<double>& plant)
      : plant_(plant),
        plan_number_(0),
        status_subscriber_(&lcm_, kLcmStatusChannel),
        plan_subscriber_(&lcm_, kLcmPlanChannel),
        stop_subscriber_(&lcm_, kLcmStopChannel) {}

  void Run() {
    int cur_plan_number = plan_number_;
    int64_t cur_plan_start_time_us = -1;

    lcmt_iiwa_command iiwa_command{};
    iiwa_command.num_joints = kNumJoints;
    iiwa_command.joint_position.resize(kNumJoints, 0.0);

    // Wait until at least one status message is processed.
    while (status_subscriber_.count() == 0) {
      lcm_.HandleSubscriptions(10);
    }

    // Run forever.
    while (true) {
      // Wait for the next message (of any kind).
      while (lcm_.HandleSubscriptions(10) == 0) {
      }

      // Check for stopping.
      if (stop_subscriber_.count() > 0) {
        HandleStop();
        stop_subscriber_.clear();
      }

      // Check for starting.
      const int64_t cur_time_us = status_subscriber_.message().utime;
      if (plan_subscriber_.count() > 0) {
        HandlePlan();
        plan_subscriber_.clear();
        if (plan_number_ != cur_plan_number) {
          std::cout << "Starting new plan." << std::endl;
          cur_plan_number = plan_number_;
          cur_plan_start_time_us = cur_time_us;
        }
      }

      // If no plan is active, go back to sleep.
      if (plan_ == nullptr) {
        continue;
      }

      // Transcribe the desired command at cur_time_us.
      iiwa_command.utime = cur_time_us;
      const Eigen::MatrixXd desired_next =
          plan_->value((cur_time_us - cur_plan_start_time_us) / 1e6);
      for (int joint = 0; joint < kNumJoints; ++joint) {
        iiwa_command.joint_position[joint] = desired_next(joint);
      }

      // Send the command.
      Publish(&lcm_, kLcmCommandChannel, iiwa_command);
    }
  }

 private:
  void HandlePlan() {
    const lcmt_robot_plan* plan = &plan_subscriber_.message();
    std::cout << "New plan received." << std::endl;
    if (plan->num_states < 2) {
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
          DRAKE_DEMAND(status_subscriber_.count() > 0);
          knots[0](idx, 0) =
              status_subscriber_.message().joint_position_commanded[j];

        } else {
          knots[i](idx, 0) = state.joint_position[j];
        }
      }
    }

    for (int i = 0; i < plan->num_states; ++i) {
      fmt::print("{}\n", fmt_eigen(knots[i]));
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

  void HandleStop() {
    std::cout << "Received stop command. Discarding plan." << std::endl;
    plan_.reset();
  }

  DrakeLcm lcm_;
  const multibody::MultibodyPlant<double>& plant_;
  int plan_number_{};
  std::unique_ptr<PiecewisePolynomial<double>> plan_;
  Subscriber<lcmt_iiwa_status> status_subscriber_;
  Subscriber<lcmt_robot_plan> plan_subscriber_;
  Subscriber<lcmt_robot_plan> stop_subscriber_;
};

int do_main() {
  multibody::MultibodyPlant<double> plant(0.0);
  multibody::Parser(&plant).AddModelsFromUrl(
      "package://drake_models/iiwa_description/urdf/"
      "iiwa14_no_collision.urdf");
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

int main() {
  return drake::examples::kuka_iiwa_arm::do_main();
}
