#include "drake/multibody/optimization/static_equilibrium_problem.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/optimization/test/optimization_with_contact_utilities.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace multibody {
GTEST_TEST(StaticEquilibriumProblemTest, SphereOnGroundTest) {
  const CoulombFriction<double> ground_friction(1.5 /* static friction */,
                                                0.8 /* dynamic friction */);

  const double radius = 0.1;
  const CoulombFriction<double> sphere_friction = ground_friction;

  test::FreeSpheresAndBoxes<AutoDiffXd> dut(
      {test::SphereSpecification(radius, 1E3, sphere_friction)},
      {} /* no box. */, ground_friction);

  StaticEquilibriumProblem problem(&(dut.plant()),
                                   dut.get_mutable_plant_context(), {});

  // Add the constraint that quaternion should have unit length.
  problem.get_mutable_prog()->AddConstraint(
      solvers::internal::ParseQuadraticConstraint(
          problem.q_vars().head<4>().cast<symbolic::Expression>().squaredNorm(),
          1, 1));

  // Set the initial guess.
  Eigen::VectorXd x_init(problem.prog().num_vars());
  x_init.setZero();
  problem.prog().SetDecisionVariableValueInVector(
      problem.q_vars().head<4>(), Eigen::Vector4d(1, 0, 0, 0), &x_init);

  // Now solve the static equilibrium problem.
  const auto result = solvers::Solve(problem.prog(), x_init);
  EXPECT_TRUE(result.is_success());

  // Check the solution.
  const auto q_sol = result.GetSolution(problem.q_vars());
  const double tol = 2E-5;
  EXPECT_NEAR(q_sol.head<4>().squaredNorm(), 1, tol);
  // The sphere must be on the ground.
  EXPECT_NEAR(q_sol(6), radius, tol);
  // Evaluate the contact wrench.
  const std::vector<ContactWrench> contact_wrench_sol =
      problem.GetContactWrenchSolution(result);
  EXPECT_EQ(contact_wrench_sol.size(), 1);
  EXPECT_TRUE(CompareMatrices(contact_wrench_sol[0].p_WCb_W,
                              Eigen::Vector3d(q_sol(4), q_sol(5), 0), tol));
  dut.plant().SetPositions(dut.get_mutable_plant_context(),
                           q_sol.cast<AutoDiffXd>());
  const Vector6<double> F_Cb_W_expected = math::autoDiffToValueMatrix(
      dut.plant().CalcGravityGeneralizedForces(dut.plant_context()));
  EXPECT_TRUE(
      CompareMatrices(contact_wrench_sol[0].F_Cb_W, F_Cb_W_expected, tol));
}
}  // namespace multibody
}  // namespace drake
