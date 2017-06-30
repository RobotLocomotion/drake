#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/dev/test/global_inverse_kinematics_test_util.h"
#include "drake/solvers/gurobi_solver.h"

using Eigen::Vector3d;
using Eigen::Isometry3d;

using drake::solvers::SolutionResult;

namespace drake {
namespace multibody {
namespace {
// Return true if the point with position `pt_pos` is not colliding with a box.
// The center of the box is in `box_pos`, the size of the box is `box_size`. The
// box is axis-aligned.
bool CheckPtCollisionFree(const Eigen::Ref<const Eigen::Vector3d>& pt_pos,
                          const Eigen::Ref<const Eigen::Vector3d>& box_pos,
                          const Eigen::Ref<const Eigen::Vector3d>& box_size) {
  if ((pt_pos.array() >= (box_pos - box_size / 2.0).array()).all() &&
      (pt_pos.array() >= (box_pos + box_size / 2.0).array()).all()) {
    return false;
  }
  return true;
}

// Check if the collision free binary variable is right. Namely one of the
// binary variable equals to 1. This means that the point is in one of the free
// space regions. (The point can be in the intersection of the free space
// regions. In that case we just set one of the region binary variable to 1)
void CheckPtCollisionFreeBinary(
    const Eigen::Ref<const Eigen::VectorXd>& collision_avoidance_binary_val) {
  int in_one_free_region = 0;
  for (int i = 0; i < collision_avoidance_binary_val.rows(); ++i) {
    if (std::abs(collision_avoidance_binary_val(i) - 1) < 1E-3) {
      ++in_one_free_region;
    }
  }
  EXPECT_EQ(in_one_free_region, 1);
}

TEST_F(KukaTest, CollisionAvoidanceTest) {
  // Suppose there is a box with length 0.15m, located at [-0.5, 0, 0.4]
  // The goal is to reach the goal position, without colliding with the box
  Eigen::Vector3d box_size(0.15, 0.15, 0.15);
  Eigen::Vector3d box_pos(-0.5, 0, 0.4);
  double margin = 0.05;

  // Now first partition the free space into 6 regions.
  std::vector<Eigen::Matrix3Xd> region_vertices(6);

  // region_vertices[0] are the vertices of the free space region in the +z
  // direction above the box.
  region_vertices[0].resize(3, 8);
  region_vertices[0].row(0) << -0.5, -0.5, 0.5, 0.5, -0.5, -0.5, 0.5, 0.5;
  region_vertices[0].row(1) << -1, 1, -1, 1, -1, 1, -1, 1;
  region_vertices[0].row(2)
      << Eigen::RowVector4d::Constant(box_pos(2) + box_size(2) / 2 + margin),
      Eigen::RowVector4d::Constant(box_pos(2) + box_size(2) / 2 + margin + 0.6);

  // region_vertices[1] are the vertices of the free space region in the -z
  // direction below the box
  region_vertices[1].resize(3, 8);
  region_vertices[1].row(0) = region_vertices[0].row(0);
  region_vertices[1].row(1) = region_vertices[0].row(1);
  region_vertices[1].row(2)
      << Eigen::RowVector4d::Constant(box_pos(2) - box_size(2) / 2 - margin),
      Eigen::RowVector4d::Constant(box_pos(2) - box_size(2) / 2 - margin - 0.4);

  // region_vertices[2] are the vertices of the free space region in the -x
  // direction, before the box
  region_vertices[2].resize(3, 8);
  region_vertices[2].row(0) << Eigen::RowVector4d::Constant(
      box_pos(0) - box_size(0) / 2 - margin - 0.4),
      Eigen::RowVector4d::Constant(box_pos(0) - box_size(0) / 2 - margin);
  region_vertices[2].row(1) << -1, 1, -1, 1, -1, 1, -1, 1;
  region_vertices[2].row(2) << 0, 0, 1, 1, 0, 0, 1, 1;

  // region_vertices[3] are the vertices of the free space region in the +x
  // direction, behind the box
  region_vertices[3].resize(3, 8);
  region_vertices[3].row(0)
      << Eigen::RowVector4d::Constant(box_pos(0) + box_size(0) / 2 + margin),
      Eigen::RowVector4d::Constant(1.2);
  region_vertices[3].row(1) = region_vertices[2].row(1);
  region_vertices[3].row(2) = region_vertices[2].row(2);

  // region_vertices[4] are the vertices of the free space region in the +y
  // direction, to the right of the box.
  region_vertices[4].resize(3, 8);
  region_vertices[4].row(0) << 0, 1.2, 0, 1.2, 0, 1.2, 0, 1.2;
  region_vertices[4].row(1)
      << Eigen::RowVector4d::Constant(box_pos(1) + box_size(1) / 2 + margin),
      Eigen::RowVector4d::Constant(1);
  region_vertices[4].row(2) << 0, 0, 1, 1, 0, 0, 1, 1;

  // region_vertices[5] are the vertices of the free space region in the -y
  // direction, to the left of the box.
  region_vertices[5].resize(3, 8);
  region_vertices[5].row(0) = region_vertices[4].row(0);
  region_vertices[5].row(1)
      << Eigen::RowVector4d::Constant(box_pos(1) - box_size(1) / 2 - margin),
      Eigen::RowVector4d::Constant(-1);
  region_vertices[5].row(2) = region_vertices[4].row(2);

  Eigen::Vector3d ee_pos(-0.7, 0, 0.45);
  global_ik_.AddWorldPositionConstraint(ee_idx_, Vector3d::Zero(), ee_pos,
                                        ee_pos);

  // First run the global IK without collision avoidance.
  solvers::GurobiSolver gurobi_solver;
  global_ik_.SetSolverOption(solvers::SolverType::kGurobi, "OutputFlag", 1);
  SolutionResult sol_result = gurobi_solver.Solve(global_ik_);
  EXPECT_EQ(sol_result, SolutionResult::kSolutionFound);
  const auto& q_without_collision_avoidance =
      global_ik_.ReconstructGeneralizedPositionSolution();
  auto cache = rigid_body_tree_->CreateKinematicsCache();
  cache.initialize(q_without_collision_avoidance);
  rigid_body_tree_->doKinematics(cache);
  const auto ee_pose_ik_without_collision_avoidance =
      rigid_body_tree_->CalcBodyPoseInWorldFrame(
          cache, rigid_body_tree_->get_body(ee_idx_));
  EXPECT_LE(
      (ee_pose_ik_without_collision_avoidance.translation() - ee_pos).norm(),
      0.11);

  int link6_idx = rigid_body_tree_->FindBodyIndex("iiwa_link_6");
  int link5_idx = rigid_body_tree_->FindBodyIndex("iiwa_link_5");
  std::vector<Eigen::Vector3d> link5_pts;
  std::vector<Eigen::Vector3d> link6_pts;
  // Currently only make sure the origin of the body is collision free. We can
  // add more points if it is not sufficient to guarantee the whole body is
  // collision free.
  link5_pts.emplace_back(0, 0, 0);
  link6_pts.emplace_back(0, 0, 0);
  std::vector<solvers::VectorXDecisionVariable>
      link5_collision_avoidance_binary(link5_pts.size());
  std::vector<solvers::VectorXDecisionVariable>
      link6_collision_avoidance_binary(link6_pts.size());
  for (int i = 0; i < static_cast<int>(link5_pts.size()); ++i) {
    link5_collision_avoidance_binary[i] = global_ik_.BodyPointInOneOfRegions(
        link5_idx, link5_pts[i], region_vertices);
  }
  for (int i = 0; i < static_cast<int>(link6_pts.size()); ++i) {
    link6_collision_avoidance_binary[i] = global_ik_.BodyPointInOneOfRegions(
        link6_idx, link6_pts[i], region_vertices);
  }

  sol_result = gurobi_solver.Solve(global_ik_);
  EXPECT_EQ(sol_result, SolutionResult::kSolutionFound);
  const auto& q_with_collision_avoidance =
      global_ik_.ReconstructGeneralizedPositionSolution();
  cache.initialize(q_with_collision_avoidance);
  rigid_body_tree_->doKinematics(cache);
  const auto ee_pose_ik_with_collision_avoidance =
      rigid_body_tree_->CalcBodyPoseInWorldFrame(
          cache, rigid_body_tree_->get_body(ee_idx_));
  // TODO(eric.cousineau): Revisit loose tolerance (PR #6162).
  EXPECT_LE((ee_pose_ik_with_collision_avoidance.translation() - ee_pos).norm(),
            0.1);

  // Now check to make sure the points are collision free.
  const auto& link5_pose_ik_with_collision_avoidance =
      rigid_body_tree_->CalcBodyPoseInWorldFrame(
          cache, rigid_body_tree_->get_body(link5_idx));
  const auto& link6_pose_ik_with_collision_avoidance =
      rigid_body_tree_->CalcBodyPoseInWorldFrame(
          cache, rigid_body_tree_->get_body(link6_idx));
  for (int i = 0; i < static_cast<int>(link5_pts.size()); ++i) {
    CheckPtCollisionFree(
        link5_pose_ik_with_collision_avoidance.linear() * link5_pts[i] +
            link5_pose_ik_with_collision_avoidance.translation(),
        box_pos, box_size);
    const auto& link5_collision_avoidance_binary_val =
        global_ik_.GetSolution(link5_collision_avoidance_binary[i]);
    CheckPtCollisionFreeBinary(link5_collision_avoidance_binary_val);
  }
  for (int i = 0; i < static_cast<int>(link6_pts.size()); ++i) {
    CheckPtCollisionFree(
        link6_pose_ik_with_collision_avoidance.linear() * link6_pts[i] +
            link6_pose_ik_with_collision_avoidance.translation(),
        box_pos, box_size);
    const auto& link6_collision_avoidance_binary_val =
        global_ik_.GetSolution(link6_collision_avoidance_binary[i]);
    CheckPtCollisionFreeBinary(link6_collision_avoidance_binary_val);
  }
}
}  // namespace
}  // namespace multibody
}  // namespace drake
