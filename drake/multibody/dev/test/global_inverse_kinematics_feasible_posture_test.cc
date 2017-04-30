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
  const int kNumSamplePerJoint = 2;
  Eigen::Matrix<double, 7, kNumSamplePerJoint> q_grid;
  for (int i = 0; i < 7; ++i) {
    q_grid(i, 0) = rigid_body_tree_->joint_limit_min(i) * 0.95
        + rigid_body_tree_->joint_limit_max(i) * 0.05;
    q_grid(i, 1) = rigid_body_tree_->joint_limit_min(i) * 0.5
        + rigid_body_tree_->joint_limit_max(i) * 0.5;
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
  for (int i = 0; i < static_cast<int>(std::pow<int>(kNumSamplePerJoint, 7));
       ++i) {
    Eigen::Matrix<double, 7, 1> q;
    for (int joint = 0; joint < 7; ++joint) {
      q(joint) = (1 << joint) && i ? q_grid(joint, 0) : q_grid(joint, 1);
    }
    cache.initialize(q);
    rigid_body_tree_->doKinematics(cache);
    for (int body = 1; body < rigid_body_tree_->get_num_bodies(); ++body) {
      const Eigen::Isometry3d body_pose =
          rigid_body_tree_->CalcBodyPoseInWorldFrame(cache,
                                                     rigid_body_tree_->get_body(
                                                         body));
      const Eigen::Vector3d
          pos_lb = body_pose.translation() - 0.01 * Eigen::Vector3d::Ones();
      const Eigen::Vector3d
          pos_ub = body_pose.translation() + 0.01 * Eigen::Vector3d::Ones();
      body_position_constraint[body - 1].constraint()->UpdateLowerBound(pos_lb);
      body_position_constraint[body - 1].constraint()->UpdateUpperBound(pos_ub);
      Eigen::Matrix<double, 9, 1> rotmat_lb_flat;
      Eigen::Matrix<double, 9, 1> rotmat_ub_flat;
      rotmat_lb_flat
          << body_pose.linear().col(0) - 0.01 * Eigen::Vector3d::Ones(),
          body_pose.linear().col(1) - 0.01 * Eigen::Vector3d::Ones(),
          body_pose.linear().col(2) - 0.01 * Eigen::Vector3d::Ones();
      rotmat_ub_flat
          << body_pose.linear().col(0) + 0.01 * Eigen::Vector3d::Ones(),
          body_pose.linear().col(1) + 0.01 * Eigen::Vector3d::Ones(),
          body_pose.linear().col(2) + 0.01 * Eigen::Vector3d::Ones();
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
