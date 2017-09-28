#pragma once

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/controllers/plan_eval/generic_plan.h"
#include "drake/systems/controllers/setpoint.h"

namespace drake {
namespace systems {
namespace controllers {
namespace plan_eval {

constexpr double kSmallTolerance = 1e-12;

// This is a common test fixture for all tests for derived classes of
// GenericPlan. It wraps around couple common objects typically used for
// making / evaluating a Plan.
class GenericPlanTest : public ::testing::Test {
 protected:
  // Allocates robot_, alias_groups_, params_, and robot_status_. dut_ should
  // be allocated in individual tests depending on which plan is under test.
  void AllocateResources(const std::string& robot_path,
                         const std::string& alias_groups_path,
                         const std::string& control_param_path) {
    robot_ = std::make_unique<RigidBodyTree<double>>();
    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        robot_path, multibody::joints::kFixed, robot_.get());

    alias_groups_ =
        std::make_unique<RigidBodyTreeAliasGroups<double>>(robot_.get());
    alias_groups_->LoadFromFile(alias_groups_path);

    params_ =
        std::make_unique<systems::controllers::qp_inverse_dynamics::ParamSet>();
    params_->LoadFromFile(control_param_path, *alias_groups_);

    robot_status_ = std::make_unique<
        systems::controllers::qp_inverse_dynamics::RobotKinematicState<double>>(
        robot_.get());
  }

  // Uses the given random number generator to set q and v in robot_status_.
  // Also sets time = 0.2s.
  void SetRandomConfiguration(std::default_random_engine* generator) {
    std::normal_distribution<double> normal;
    VectorX<double> q = robot_->getRandomConfiguration(*generator);
    VectorX<double> v(robot_->get_num_velocities());
    for (int i = 0; i < v.size(); i++) {
      v[i] = normal(*generator);
    }
    double time_now = 0.2;
    robot_status_->UpdateKinematics(time_now, q, v);
  }

  // Returns kp * (q_d - q) + kd* (v_d - v) + vd_d, where q v are from
  // robot_status_ and kp and kd are from params_.
  VectorX<double> ComputeExpectedDoFAcceleration(
      const VectorX<double>& q_d, const VectorX<double>& v_d,
      const VectorX<double>& vd_d) const {
    VectorX<double> kp, kd;
    params_->LookupDesiredDofMotionGains(&kp, &kd);

    VectorX<double> expected_vd =
        (kp.array() * (q_d - robot_status_->get_cache().getQ()).array() +
         kd.array() * (v_d - robot_status_->get_cache().getV()).array())
            .matrix() +
        vd_d;

    return expected_vd;
  }

  // Returns a spatial acceleration using CartesianSetpoint, it is conceptually
  // the same as ComputeExpectedDoFAcceleration(), with the exception of the
  // rotation difference between pose_d and pose. Details are in control_utils.h
  Vector6<double> ComputeExpectedBodyAcceleration(
      const RigidBody<double>* body, const Isometry3<double> pose_d,
      const Vector6<double> vel_d, const Vector6<double> acc_d) const {
    // Finds the kp and kd gains from param.
    Vector6<double> kp, kd;
    params_->LookupDesiredBodyMotionGains(*body, &kp, &kd);
    systems::controllers::CartesianSetpoint<double> tracker(pose_d, vel_d,
                                                            acc_d, kp, kd);

    // Computes current body pose and velocity.
    Isometry3<double> pose =
        robot_status_->get_robot().CalcBodyPoseInWorldFrame(
            robot_status_->get_cache(), *body);
    Vector6<double> vel =
        robot_status_->get_robot().CalcBodySpatialVelocityInWorldFrame(
            robot_status_->get_cache(), *body);

    Vector6<double> expected_pose_acc =
        tracker.ComputeTargetAcceleration(pose, vel);

    return expected_pose_acc;
  }

  // Tests clone for the any derived classes that does not introduce new member
  // fields.
  void TestGenericClone() const {
    std::unique_ptr<GenericPlan<double>> clone = dut_->Clone();
    EXPECT_EQ(dut_->get_planned_contact_state(),
              clone->get_planned_contact_state());
    EXPECT_TRUE(dut_->get_dof_trajectory().is_approx(
        clone->get_dof_trajectory(), kSmallTolerance));

    const auto& trajs = dut_->get_body_trajectories();
    const auto& cloned_trajs = clone->get_body_trajectories();
    EXPECT_EQ(trajs.size(), cloned_trajs.size());
    for (const auto& traj_pair : trajs) {
      auto it = cloned_trajs.find(traj_pair.first);
      EXPECT_TRUE(it != cloned_trajs.end());
      EXPECT_TRUE(it->second.is_approx(traj_pair.second, kSmallTolerance));
    }
  }

  std::unique_ptr<RigidBodyTree<double>> robot_{nullptr};
  std::unique_ptr<RigidBodyTreeAliasGroups<double>> alias_groups_{nullptr};
  std::unique_ptr<systems::controllers::qp_inverse_dynamics::ParamSet> params_{
      nullptr};
  std::unique_ptr<
      systems::controllers::qp_inverse_dynamics::RobotKinematicState<double>>
      robot_status_{nullptr};

  // Plan under test.
  std::unique_ptr<GenericPlan<double>> dut_{nullptr};
};

}  // namespace plan_eval
}  // namespace controllers
}  // namespace systems
}  // namespace drake
