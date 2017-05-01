#include "drake/multibody/dev/test/global_inverse_kinematics_test_util.h"
#include "drake/solvers/gurobi_solver.h"

#include <cmath>

namespace drake {
namespace multibody {
TEST_F(KukaTest, FeasiblePostureTest) {
  // For feasible postures, check if global IK also thinks it is a feasible
  // solution. For each posture q, we compute the forward kinematics, and
  // constraint the body pose in global IK, to the pose from the forward
  // kinematics. global IK should always think the problem is feasible.

  // First sample some postures, within the joint limits.
  const int kNumSamplePerJoint = 3;
  Eigen::Matrix<double, 7, kNumSamplePerJoint> q_grid;
  for (int i = 0; i < 7; ++i) {
    q_grid(i, 0) = rigid_body_tree_->joint_limit_min(i) * 0.95
        + rigid_body_tree_->joint_limit_max(i) * 0.05;
    q_grid(i, 1) = rigid_body_tree_->joint_limit_min(i) * 0.5
        + rigid_body_tree_->joint_limit_max(i) * 0.5;
    q_grid(i, 2) = rigid_body_tree_->joint_limit_min(i) * 0.05
        + rigid_body_tree_->joint_limit_max(i) * 0.95;
  }
  KinematicsCache<double> cache = rigid_body_tree_->CreateKinematicsCache();

  std::vector<solvers::Binding<solvers::BoundingBoxConstraint>>
      body_position_constraint;
  std::vector<solvers::Binding<solvers::BoundingBoxConstraint>>
      body_orientation_constraint;
  for (int body = 1; body < rigid_body_tree_->get_num_bodies(); ++body) {
    body_position_constraint.push_back(global_ik_.AddBoundingBoxConstraint(0,
                                                                             0,
                                                                             global_ik_.body_position(
                                                                                 body)));
    const auto &body_rotmat = global_ik_.body_rotation_matrix(body);
    Eigen::Matrix<symbolic::Variable, 9, 1> body_rotmat_flat;
    body_rotmat_flat << body_rotmat.col(0), body_rotmat.col(1), body_rotmat.col(
        2);
    body_orientation_constraint.push_back(
        global_ik_.AddBoundingBoxConstraint(0, 0, body_rotmat_flat));
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
    for (int joint = 0; joint < 7; ++joint) {
      ++joint_grid_idx(joint);
      if (joint_grid_idx(joint) >= kNumSamplePerJoint) {
        joint_grid_idx(joint) -= kNumSamplePerJoint;
        if (joint < 6) {
          ++joint_grid_idx(joint + 1);
        }
      }
    }
    ++q_count;

    // Now compute the forward kinematics for posture q, and then fix the body
    // position and orientation of the global IK to the body pose of q.
    cache.initialize(q);
    rigid_body_tree_->doKinematics(cache);
    for (int body = 1; body < rigid_body_tree_->get_num_bodies(); ++body) {
      const Eigen::Isometry3d body_pose =
          rigid_body_tree_->CalcBodyPoseInWorldFrame(cache,
                                                     rigid_body_tree_->get_body(
                                                         body));
      const Eigen::Vector3d
          pos_lb = body_pose.translation();
      const Eigen::Vector3d
          pos_ub = body_pose.translation();
      body_position_constraint[body - 1].constraint()->UpdateLowerBound(pos_lb);
      body_position_constraint[body - 1].constraint()->UpdateUpperBound(pos_ub);
      Eigen::Matrix<double, 9, 1> rotmat_lb_flat;
      Eigen::Matrix<double, 9, 1> rotmat_ub_flat;
      rotmat_lb_flat
          << body_pose.linear().col(0),
          body_pose.linear().col(1),
          body_pose.linear().col(2);
      rotmat_ub_flat
          << body_pose.linear().col(0),
          body_pose.linear().col(1),
          body_pose.linear().col(2);
      body_orientation_constraint[body - 1].constraint()->UpdateLowerBound(
          rotmat_lb_flat);
      body_orientation_constraint[body - 1].constraint()->UpdateUpperBound(
          rotmat_ub_flat);
    }
    solvers::GurobiSolver gurobi_solver;
    const solvers::SolutionResult
        sol_result = gurobi_solver.Solve(global_ik_);
    EXPECT_EQ(sol_result, solvers::SolutionResult::kSolutionFound);
  }
}
}  // namespace multibody
}  // namespace drake
