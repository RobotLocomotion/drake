#include "drake/multibody/global_inverse_kinematics.h"

#include <gtest/gtest.h>
#include <drake/solvers/gurobi_solver.h>

#include "drake/common/drake_path.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree_construction.h"

using drake::solvers::SolutionResult;

namespace drake {
namespace multibody {
namespace {
GTEST_TEST(TestGlobakIK, KukaTest) {
  auto rigid_body_tree = std::make_unique<RigidBodyTree<double>>();
  const std::string model_path = drake::GetDrakePath() +
      "/examples/kuka_iiwa_arm/models/iiwa14/"
          "iiwa14_simplified_collision.urdf";

  parsers::urdf::AddModelInstanceFromUrdfFile(
    model_path,
    drake::multibody::joints::kFixed,
    nullptr,
    rigid_body_tree.get());

  AddFlatTerrainToWorld(rigid_body_tree.get());

  GlobalInverseKinematics global_ik(std::move(rigid_body_tree));

  solvers::GurobiSolver gurobi_solver;

  global_ik.SetSolverOption(solvers::SolverType::kGurobi, "OutputFlag", 1);
  SolutionResult sol_result = gurobi_solver.Solve(global_ik);

  EXPECT_EQ(sol_result, SolutionResult::kSolutionFound);
}
}  // namespace
}  // namespace multibody
}  // namespace drake