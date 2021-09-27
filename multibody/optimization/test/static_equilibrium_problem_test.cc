#include "drake/multibody/optimization/static_equilibrium_problem.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/optimization/test/optimization_with_contact_utilities.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace multibody {
GTEST_TEST(StaticEquilibriumProblemTest, SphereOnGroundTest) {
  // Find the equilibrium pose for a single sphere on the ground.
  const CoulombFriction<double> ground_friction(1. /* static friction */,
                                                0.8 /* dynamic friction */);

  const double radius = 0.1;
  const CoulombFriction<double> sphere_friction = ground_friction;

  test::FreeSpheresAndBoxes<AutoDiffXd> free_spheres(
      {test::SphereSpecification(radius, 1E3 /* density of the sphere */,
                                 sphere_friction)},
      {} /* no box. */, ground_friction);

  StaticEquilibriumProblem dut(&(free_spheres.plant()),
                               free_spheres.get_mutable_plant_context(), {});

  // Add the constraint that quaternion should have unit length.
  dut.get_mutable_prog()->AddConstraint(
      solvers::internal::ParseQuadraticConstraint(
          dut.q_vars().head<4>().cast<symbolic::Expression>().squaredNorm(), 1,
          1));

  auto check_static_equilibrium_problem_solve =
      [&dut, &free_spheres, radius](const Eigen::VectorXd& x_init) {
        // Now solve the static equilibrium problem.
        // Our formulation of the complementary constraint is suitable for
        // elastic SQP solver, such as Snopt. For more details, please refer to
        // section 3.2 of A Direct Method for Trajectory Optimization of
        // RigidBodies Through Contact by Michael Posa, Cecilia Cantu and Russ
        // Tedrake.
        solvers::SnoptSolver snopt_solver;
        if (snopt_solver.available()) {
          const auto result = snopt_solver.Solve(dut.prog(), x_init, {});
          ASSERT_TRUE(result.is_success());

          // Check the solution.
          const auto q_sol = result.GetSolution(dut.q_vars());
          const double tol = 2E-5;
          EXPECT_NEAR(q_sol.head<4>().squaredNorm(), 1, tol);
          // The sphere must be on the ground.
          EXPECT_NEAR(q_sol(6), radius, tol);
          // Evaluate the contact wrench.
          const std::vector<ContactWrench> contact_wrench_sol =
              dut.GetContactWrenchSolution(result);
          EXPECT_EQ(contact_wrench_sol.size(), 1);
          EXPECT_TRUE(CompareMatrices(contact_wrench_sol[0].p_WCb_W,
                                      Eigen::Vector3d(q_sol(4), q_sol(5), 0),
                                      tol));
          free_spheres.plant().SetPositions(
              free_spheres.get_mutable_plant_context(),
              q_sol.cast<AutoDiffXd>());
          const Vector6<double> F_Cb_W_expected = math::ExtractValue(
                  free_spheres.plant().CalcGravityGeneralizedForces(
                      free_spheres.plant_context()));
          EXPECT_TRUE(CompareMatrices(contact_wrench_sol[0].F_Cb_W.get_coeffs(),
                                      F_Cb_W_expected, tol));
        }
      };

  // Set the initial guess.
  Eigen::VectorXd x_init(dut.prog().num_vars());
  x_init.setZero();

  // The sphere is in penetration with the ground in the initial pose.
  dut.prog().SetDecisionVariableValueInVector(
      dut.q_vars().head<4>(), Eigen::Vector4d(1, 0, 0, 0), &x_init);
  check_static_equilibrium_problem_solve(x_init);

  // The sphere is above the ground in the initial pose.
  dut.prog().SetDecisionVariableValueInVector(dut.q_vars()(6), radius + 1,
                                              &x_init);
  check_static_equilibrium_problem_solve(x_init);

  // The quaternion in the initial pose is not normalized.
  dut.prog().SetDecisionVariableValueInVector(
      dut.q_vars().head<4>(), Eigen::Vector4d(1, 0.2, 0.3, 0.4), &x_init);
  check_static_equilibrium_problem_solve(x_init);
}

GTEST_TEST(TestStaticEquilibriumProblem, TwoSpheresWithinBin) {
  // Find the equilibrium pose for two spheres within a bin.
  const CoulombFriction<double> ground_friction(0.1 /* static friction */,
                                                0.05 /* dynamic friction */);

  const std::array<double, 2> radii = {0.1, 0.2};
  const CoulombFriction<double> sphere_friction(0.4 /* static friction */,
                                                0.3 /* dynamic friction */);

  const double sphere_density = 1E3;

  // Add the wall of the bin. The diagonal line of the bin's interior is smaller
  // than the sum of the sphere diameters, hence not all spheres can be on the
  // ground.
  const double bin_length = 0.5;
  const Eigen::Vector3d wall_size(bin_length, 0.01, 0.4);
  const double wall_density = 1E3;
  const CoulombFriction<double> wall_friction = sphere_friction;
  std::vector<test::BoxSpecification> walls;
  math::RigidTransformd X_WWall1(
      Eigen::Vector3d(0, bin_length / 2, wall_size(2) / 2));
  walls.emplace_back(wall_size, wall_density, wall_friction, X_WWall1);
  math::RigidTransformd X_WWall2(
      Eigen::Vector3d(0, -bin_length / 2, wall_size(2) / 2));
  walls.emplace_back(wall_size, wall_density, wall_friction, X_WWall2);
  math::RigidTransformd X_WWall3(
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()),
      Eigen::Vector3d(bin_length / 2, 0, wall_size(2) / 2));
  walls.emplace_back(wall_size, wall_density, wall_friction, X_WWall3);
  math::RigidTransformd X_WWall4(
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()),
      Eigen::Vector3d(-bin_length / 2, 0, wall_size(2) / 2));
  walls.emplace_back(wall_size, wall_density, wall_friction, X_WWall4);

  std::vector<test::SphereSpecification> spheres;
  spheres.emplace_back(radii[0], sphere_density, sphere_friction);
  spheres.emplace_back(radii[1], sphere_density, sphere_friction);
  test::FreeSpheresAndBoxes<AutoDiffXd> free_spheres(spheres, walls,
                                                     ground_friction);

  // free_spheres_double is for visualization.
  test::FreeSpheresAndBoxes<double> free_spheres_double(spheres, walls,
                                                        ground_friction);

  StaticEquilibriumProblem dut(&(free_spheres.plant()),
                               free_spheres.get_mutable_plant_context(), {});

  // The decision variables are q (dim = 14), and the decision variables for
  // 11 pairs of contact, each pair introduce lambda (dim = 3), alpha (dim = 1)
  // and beta (dim = 1).
  EXPECT_EQ(dut.prog().num_vars(), 14 + (3 + 2) * (5 + 5 + 1));

  // Add the constraint that quaternion should have unit length.
  dut.get_mutable_prog()->AddConstraint(
      solvers::internal::ParseQuadraticConstraint(
          dut.q_vars().head<4>().cast<symbolic::Expression>().squaredNorm(), 1,
          1));
  dut.get_mutable_prog()->AddConstraint(
      solvers::internal::ParseQuadraticConstraint(
          dut.q_vars().segment<4>(7).cast<symbolic::Expression>().squaredNorm(),
          1, 1));

  // Add the constraint that the spheres should be within the bin.
  dut.get_mutable_prog()->AddBoundingBoxConstraint(
      Eigen::Vector2d::Constant(-bin_length / 2),
      Eigen::Vector2d::Constant(bin_length / 2), dut.q_vars().segment<2>(4));
  dut.get_mutable_prog()->AddBoundingBoxConstraint(
      Eigen::Vector2d::Constant(-bin_length / 2),
      Eigen::Vector2d::Constant(bin_length / 2), dut.q_vars().segment<2>(11));

  auto check_static_equilibrium_problem_solve =
      [&dut, &free_spheres_double, &radii, &bin_length,
       &wall_size](const Eigen::VectorXd& x_init) {
        // Now solve the static equilibrium problem.
        solvers::SnoptSolver snopt_solver;
        if (snopt_solver.available()) {
          solvers::SolverOptions solver_options;
          const auto result =
              snopt_solver.Solve(dut.prog(), x_init, solver_options);
          EXPECT_TRUE(result.is_success());

          // Check the solution.
          const auto q_sol = result.GetSolution(dut.q_vars());
          const std::vector<ContactWrench> contact_wrench_sol =
              dut.GetContactWrenchSolution(result);

          free_spheres_double.plant().SetPositions(
              free_spheres_double.get_mutable_plant_context(), q_sol);
          free_spheres_double.get_mutable_diagram()->Publish(
              free_spheres_double.diagram_context());
          const double tol = 2E-5;
          EXPECT_NEAR(q_sol.head<4>().squaredNorm(), 1, tol);
          EXPECT_NEAR(q_sol.segment<4>(7).squaredNorm(), 1, tol);

          // Now check that both spheres are above the ground and within the
          // box.
          EXPECT_GE(q_sol(6), radii[0] - tol);
          EXPECT_GE(q_sol(13), radii[1] - tol);
          EXPECT_LE(std::abs(q_sol(4)),
                    bin_length - wall_size(1) / 2 - radii[0] + tol);
          EXPECT_LE(std::abs(q_sol(5)),
                    bin_length - wall_size(1) / 2 - radii[0] + tol);
          EXPECT_LE(std::abs(q_sol(11)),
                    bin_length - wall_size(1) / 2 - radii[1] + tol);
          EXPECT_LE(std::abs(q_sol(12)),
                    bin_length - wall_size(1) / 2 - radii[1] + tol);

          // Now check that the two spheres are not penetrating each other.
          EXPECT_GE((q_sol.segment<3>(4) - q_sol.segment<3>(11)).norm(),
                    radii[0] + radii[1] - tol);
        }
      };

  // Set the initial guess.
  Eigen::VectorXd x_init(dut.prog().num_vars());
  x_init.setZero();

  // The initial poses for both spheres are above the ground.
  Eigen::VectorXd q_init(14);
  q_init.head<4>() << 1, 0, 0, 0;
  q_init.segment<3>(4) << 0.2, 0, radii[0] + 0.6;
  q_init.segment<4>(7) << 1, 0, 0, 0;
  q_init.tail<3>() << -0.1, 0, radii[1] + 0.01;
  dut.prog().SetDecisionVariableValueInVector(dut.q_vars(), q_init, &x_init);
  check_static_equilibrium_problem_solve(x_init);
}
}  // namespace multibody
}  // namespace drake
