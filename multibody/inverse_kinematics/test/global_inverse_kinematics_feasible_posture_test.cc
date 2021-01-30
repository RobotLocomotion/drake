#include "drake/multibody/inverse_kinematics/test/global_inverse_kinematics_test_util.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace multibody {
TEST_F(KukaTest, FeasiblePostureTest) {
  // For feasible postures, check if global IK also thinks it is a feasible
  // solution. For each posture q, we compute the forward kinematics, and
  // constrain the body pose in global IK, to the pose from the forward
  // kinematics. Global IK should always think the problem is feasible.

  // First sample some postures, within the joint limits.
  const int kNumSamplePerJoint = 3;
  Eigen::Matrix<double, 7, kNumSamplePerJoint> q_grid;
  for (int i = 0; i < 7; ++i) {
    q_grid(i, 0) = plant_->GetPositionLowerLimits()(i) * 0.95 +
                   plant_->GetPositionUpperLimits()(i) * 0.05;
    q_grid(i, 1) = plant_->GetPositionLowerLimits()(i) * 0.5 +
                   plant_->GetPositionUpperLimits()(i) * 0.5;
    q_grid(i, 2) = plant_->GetPositionLowerLimits()(i) * 0.05 +
                   plant_->GetPositionUpperLimits()(i) * 0.95;
  }
  auto context = plant_->CreateDefaultContext();

  std::vector<solvers::Binding<solvers::BoundingBoxConstraint>>
      body_position_constraint;
  std::vector<solvers::Binding<solvers::BoundingBoxConstraint>>
      body_orientation_constraint;
  for (BodyIndex body{1}; body < plant_->num_bodies(); ++body) {
    // body = 0 is the WORLD link, we should not constrain the pose of the
    // WORLD.
    body_position_constraint.push_back(
        global_ik_.get_mutable_prog()->AddBoundingBoxConstraint(
            0, 0, global_ik_.body_position(body)));
    const auto& body_rotmat = global_ik_.body_rotation_matrix(body);
    Eigen::Matrix<symbolic::Variable, 9, 1> body_rotmat_flat;
    body_rotmat_flat << body_rotmat.col(0), body_rotmat.col(1),
        body_rotmat.col(2);
    body_orientation_constraint.push_back(
        global_ik_.get_mutable_prog()->AddBoundingBoxConstraint(
            0, 0, body_rotmat_flat));
  }

  // Now generate the feasible posture q
  // q(i) = q_grid(i, joint_grid_idx(i));
  Eigen::Matrix<int, 7, 1> joint_grid_idx;
  joint_grid_idx.setZero();
  int num_q = std::pow<int>(kNumSamplePerJoint, 7);
  int q_count = 0;
  while (q_count < num_q) {
    Eigen::Matrix<double, 7, 1> q;
    for (int joint = 0; joint < 7; ++joint) {
      q(joint) = q_grid(joint, joint_grid_idx(joint));
    }
    // Now increment joint_grid_idx;
    // joint_grid_idx is a 7-digit number, each digit can have value
    // {0, 1, ..., kNumSamplePerJoint - 1}. We increment the last digit by 1.
    // If that digit is larger than kNumSamplePerJoint - 1, we set that digit
    // to 0, and add the carry bit to the next digit.
    ++joint_grid_idx(6);
    for (int joint = 6; joint >= 0; --joint) {
      if (joint_grid_idx(joint) >= kNumSamplePerJoint) {
        joint_grid_idx(joint) = 0;
        if (joint > 0) {
          ++joint_grid_idx(joint - 1);
        }
      }
    }
    ++q_count;

    // Now compute the forward kinematics for posture q, and then fix the body
    // position and orientation of the global IK to the body pose of q.
    plant_->SetPositions(context.get(), q);
    for (int body = 1; body < plant_->num_bodies(); ++body) {
      const math::RigidTransformd body_pose = plant_->CalcRelativeTransform(
          *context, plant_->world_frame(),
          plant_->get_body(BodyIndex{body}).body_frame());
      const Eigen::Vector3d pos_lb = body_pose.translation();
      const Eigen::Vector3d pos_ub = body_pose.translation();
      body_position_constraint[body - 1].evaluator()->UpdateLowerBound(pos_lb);
      body_position_constraint[body - 1].evaluator()->UpdateUpperBound(pos_ub);
      Eigen::Matrix<double, 9, 1> rotmat_lb_flat;
      Eigen::Matrix<double, 9, 1> rotmat_ub_flat;
      rotmat_lb_flat << body_pose.rotation().matrix().col(0),
          body_pose.rotation().matrix().col(1),
          body_pose.rotation().matrix().col(2);
      rotmat_ub_flat << body_pose.rotation().matrix().col(0),
          body_pose.rotation().matrix().col(1),
          body_pose.rotation().matrix().col(2);
      body_orientation_constraint[body - 1].evaluator()->UpdateLowerBound(
          rotmat_lb_flat);
      body_orientation_constraint[body - 1].evaluator()->UpdateUpperBound(
          rotmat_ub_flat);
    }
    solvers::GurobiSolver gurobi_solver;
    const auto result = gurobi_solver.Solve(global_ik_.prog());
    EXPECT_TRUE(result.is_success());
  }
}
}  // namespace multibody
}  // namespace drake
