/// @file
///
/// kuka_plan_runner is designed to wait for LCM messages contraining
/// a robot_plan_t message, and then execute the plan on an iiwa arm
/// (also communicating via LCM using the
/// lcmt_iiwa_command/lcmt_iiwa_status messages).
///
/// When a plan is received, it will immediately begin executing that
/// plan on the arm (replacing any plan in progress).

#include <iostream>
#include <memory>

#include <lcm/lcm-cpp.hpp>

#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/polynomial.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"

#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"

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
const int kNumJoints = 7;

typedef PiecewisePolynomial<double> PPType;
typedef PPType::PolynomialType PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

class RobotPlanRunner {
 public:
  /// tree is aliased
  explicit RobotPlanRunner(const RigidBodyTree& tree)
      : tree_(tree), plan_number_(0) {
    VerifyIiwaTree(tree);
    lcm_.subscribe(kLcmStatusChannel,
                    &RobotPlanRunner::HandleStatus, this);
    lcm_.subscribe(kLcmPlanChannel,
                    &RobotPlanRunner::HandlePlan, this);
  }

  void Run() {
    int cur_plan_number = plan_number_;
    int64_t cur_time_ms = -1;
    int64_t start_time_ms = -1;

    // Initialize the timestamp to an invalid number so we can detect
    // the first message.
    iiwa_status_.timestamp = cur_time_ms;

    lcmt_iiwa_command iiwa_command;
    iiwa_command.num_joints = kNumJoints;
    iiwa_command.joint_position.resize(kNumJoints, 0.);
    iiwa_command.num_torques = 0;
    iiwa_command.joint_torque.resize(kNumJoints, 0.);

    while (true) {
      // The argument to handleTimeout is in msec, and should be
      // safely bigger than e.g. a 200Hz input rate.
      int handled  = lcm_.handleTimeout(10);
      if (handled <= 0) {
        std::cerr << "Failed to receive LCM status." << std::endl;
        return;
      }

      DRAKE_ASSERT(iiwa_status_.timestamp != -1);
      cur_time_ms = iiwa_status_.timestamp;

      if (plan_) {
        if (plan_number_ != cur_plan_number) {
          std::cout << "Starting new plan." << std::endl;
          start_time_ms = cur_time_ms;
          cur_plan_number = plan_number_;
        }

        const double cur_traj_time_s =
            static_cast<double>(cur_time_ms - start_time_ms) / 1e3;
        const auto desired_next = plan_->value(cur_traj_time_s);

        iiwa_command.timestamp = iiwa_status_.timestamp;

        // This is totally arbitrary.  There's no good reason to
        // implement this as a maximum delta to submit per tick.  What
        // we actually need is something like a proper
        // planner/interpolater which spreads the motion out over the
        // entire duration from current_t to next_t, and commands the
        // next position taking into account the velocity of the joints
        // and the distance remaining.
        const double max_joint_delta = 0.1;
        for (int joint = 0; joint < kNumJoints; joint++) {
          double joint_delta =
              desired_next(joint) - iiwa_status_.joint_position_measured[joint];
          joint_delta = std::max(-max_joint_delta,
                                 std::min(max_joint_delta, joint_delta));
          iiwa_command.joint_position[joint] =
              iiwa_status_.joint_position_measured[joint] + joint_delta;
        }

        lcm_.publish(kLcmCommandChannel, &iiwa_command);
      }
    }
  }

 private:
  void HandleStatus(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                    const lcmt_iiwa_status* status) {
    iiwa_status_ = *status;
  }

  void HandlePlan(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                  const robotlocomotion::robot_plan_t* plan) {
    std::cout << "New plan received." << std::endl;
    Eigen::MatrixXd traj_mat(kNumJoints, plan->num_states);
    traj_mat.fill(0);

    std::map<std::string, int> name_to_idx =
        tree_.computePositionNameToIndexMap();
    for (int i = 0; i < plan->num_states; ++i) {
      const auto& state = plan->plan[i];
      for (int j = 0; j < state.num_joints; ++j) {
        if (name_to_idx.count(state.joint_name[j]) == 0) {
          continue;
        }
        traj_mat(name_to_idx[state.joint_name[j]], i) =
            state.joint_position[j];
      }
    }

    std::cout << traj_mat << std::endl;

    std::vector<PPMatrix> polys;
    std::vector<double> times;

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
                       ((plan->plan[i + 1].utime  -
                         plan->plan[i].utime) / 1e6));
        }
        poly_matrix(row) = PPPoly(coeffs);
      }
      polys.push_back(poly_matrix);
      times.push_back(plan->plan[i].utime / 1e6);
    }
    times.push_back(times.back() + 0.01);
    plan_.reset(new PPType(polys, times));
    ++plan_number_;
  }

  lcm::LCM lcm_;
  const RigidBodyTree& tree_;
  int plan_number_{};
  std::unique_ptr<PPType> plan_;
  lcmt_iiwa_status iiwa_status_;
};

int do_main(int argc, const char* argv[]) {
  RigidBodyTree tree(
      drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
      drake::systems::plants::joints::kFixed);

  RobotPlanRunner runner(tree);
  runner.Run();
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake


int main(int argc, const char* argv[]) {
  return drake::examples::kuka_iiwa_arm::do_main(argc, argv);
}
