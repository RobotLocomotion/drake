#pragma once

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/generic_plan.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

// This is a common test fixture for all tests for derived classes of
// GenericPlan. It wraps around couple common objects typically used for
// making / evaluating a Plan.
class GenericPlanTest : public ::testing::Test {
 protected:
  // Assumes robot_ and dut_ have already allocated.
  void Initialize(const std::string& alias_groups_path,
                  const std::string& control_param_path) {
    DRAKE_DEMAND(robot_ != nullptr);
    DRAKE_DEMAND(dut_ != nullptr);

    alias_groups_ =
        std::make_unique<param_parsers::RigidBodyTreeAliasGroups<double>>(
            *robot_);
    alias_groups_->LoadFromFile(alias_groups_path);

    params_ = std::make_unique<param_parsers::ParamSet>();
    params_->LoadFromFile(control_param_path, *alias_groups_);

    robot_status_ = std::make_unique<HumanoidStatus>(*robot_, *alias_groups_);
    std::default_random_engine generator(123);
    std::normal_distribution<double> normal;
    VectorX<double> q = robot_->getRandomConfiguration(generator);
    VectorX<double> v(robot_->get_num_velocities());
    for (int i = 0; i < v.size(); i++) {
      v[i] = normal(generator);
    }
    double time_now = 0.2;
    robot_status_->UpdateKinematics(time_now, q, v);

    // Initializes the plan using the current robot status.
    dut_->Initialize(*robot_status_, *params_, *alias_groups_);
  }

  std::unique_ptr<RigidBodyTree<double>> robot_{nullptr};
  std::unique_ptr<param_parsers::RigidBodyTreeAliasGroups<double>>
      alias_groups_{nullptr};
  std::unique_ptr<param_parsers::ParamSet> params_{nullptr};
  std::unique_ptr<HumanoidStatus> robot_status_{nullptr};

  std::unique_ptr<GenericPlan<double>> dut_{nullptr};
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
