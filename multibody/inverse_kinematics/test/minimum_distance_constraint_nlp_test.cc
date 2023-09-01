/**
Solves optimization programs with InCollisionConstraint. We want to make
sure that our formulation of the InCollisionConstraint is suitable for the
nonlinear optimization solvers.
*/
#include <limits>

#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/inverse_kinematics/minimum_distance_constraint.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace multibody {
namespace {
const double kInf = std::numeric_limits<double>::infinity();

TEST_F(TwoFreeSpheresTest, NoCollisionTest) {
  auto diagram_context = diagram_double_->CreateDefaultContext();
  systems::Context<double>* plant_context =
      &(plant_double_->GetMyMutableContextFromRoot(diagram_context.get()));
  InverseKinematics ik(*plant_double_, plant_context);
  MinimumDistancePenaltyFunction penalty_function{};
  double influence_distance = 0.01;
  auto no_collision_constraint = std::make_shared<MinimumDistanceConstraint>(
      plant_double_, 0, kInf, plant_context, penalty_function,
      influence_distance);
  ik.get_mutable_prog()->AddConstraint(no_collision_constraint, ik.q());

  Eigen::VectorXd q_guess(ik.q().rows());
  q_guess.setZero();
  q_guess.head<4>() << 1, 0, 0, 0;
  q_guess.segment<4>(7) << 1, 0, 0, 0;
  Eigen::VectorXd sphere2_position_x = Eigen::VectorXd::LinSpaced(
      10, 0.01, radius1_ + radius2_ + influence_distance + 1);
  for (int i = 0; i < sphere2_position_x.rows(); ++i) {
    q_guess(11) = sphere2_position_x(i);
    const auto result = solvers::Solve(ik.prog(), q_guess);
    EXPECT_TRUE(result.is_success());
  }
}

TEST_F(TwoFreeSpheresTest, InCollisionTest) {
  auto diagram_context = diagram_double_->CreateDefaultContext();
  systems::Context<double>* plant_context =
      &(plant_double_->GetMyMutableContextFromRoot(diagram_context.get()));
  InverseKinematics ik(*plant_double_, plant_context);
  MinimumDistancePenaltyFunction penalty_function{};
  auto in_collision_constraint = std::make_shared<InCollisionConstraint>(
      plant_double_, 0, 10, plant_context);
  ik.get_mutable_prog()->AddConstraint(in_collision_constraint, ik.q());

  Eigen::VectorXd q_guess(ik.q().rows());
  q_guess.setZero();
  q_guess.head<4>() << 1, 0, 0, 0;
  q_guess.segment<4>(7) << 1, 0, 0, 0;
  Eigen::VectorXd sphere2_position_x =
      Eigen::VectorXd::LinSpaced(10, 0.01, radius1_ + radius2_ + 100 - 0.01);
  solvers::SolverOptions solver_options;
  solver_options.SetOption(solvers::IpoptSolver::id(), "max_iter", 1000);
  if (solvers::SnoptSolver::is_available() &&
      solvers::SnoptSolver::is_enabled()) {
    // TODO(hongkai.dai): For some reasons IPOPT often fails to solve the
    // problem and exceeds the maximal iteration. I will look into this later.
    for (int i = 0; i < sphere2_position_x.rows(); ++i) {
      q_guess(11) = sphere2_position_x(i);
      Eigen::VectorXd y_guess;
      in_collision_constraint->Eval(q_guess, &y_guess);
      const auto result = solvers::Solve(ik.prog(), q_guess, solver_options);
      EXPECT_TRUE(result.is_success());
      if (!result.is_success()) {
        drake::log()->info(result.get_solution_result());
        const Eigen::VectorXd q_sol = result.GetSolution(ik.q());
        drake::log()->info("q_sol: {}", fmt_eigen(q_sol.transpose()));
        Eigen::VectorXd in_collision_constraint_val;
        auto q_sol_ad = math::InitializeAutoDiff(q_sol);
        AutoDiffVecXd y_sol_ad;
        in_collision_constraint->Eval(q_sol_ad, &y_sol_ad);
        drake::log()->info("y_sol value {}, derivatives {}",
                           y_sol_ad(0).value(),
                           fmt_eigen(y_sol_ad(0).derivatives().transpose()));
      }
    }
  }
}

TEST_F(SpheresAndWallsTest, InCollisionTest) {
  auto diagram = builder_.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  systems::Context<double>* plant_context =
      &(diagram->mutable_plant_context(diagram_context.get()));
  auto in_collision_constraint = std::make_shared<InCollisionConstraint>(
      &(diagram->plant()), 0, 2, plant_context);

  InverseKinematics ik(diagram->plant(), plant_context);
  ik.get_mutable_prog()->AddConstraint(in_collision_constraint, ik.q());

  Eigen::VectorXd sphere_position_x = Eigen::VectorXd::LinSpaced(
      10, -wall_length_ / 2 - 0.2, wall_length_ / 2 + 0.2);
  Eigen::VectorXd q_guess = Eigen::VectorXd::Zero(ik.q().rows());
  q_guess.head<4>() << 1, 0, 0, 0;
  q_guess.segment<4>(7) << 1, 0, 0, 0;
  solvers::SolverOptions solver_options;
  solver_options.SetOption(solvers::IpoptSolver::id(), "max_iter", 1000);
  for (int i = 0; i < sphere_position_x.rows(); ++i) {
    q_guess(11) = sphere_position_x(i);
    const auto result = solvers::Solve(ik.prog(), q_guess, solver_options);
    EXPECT_TRUE(result.is_success());
    if (!result.is_success()) {
      if (result.get_solver_id() == solvers::IpoptSolver::id()) {
        drake::log()->info(
            "ipopt status: {}, {}",
            result.get_solver_details<solvers::IpoptSolver>().status,
            result.get_solver_details<solvers::IpoptSolver>()
                .ConvertStatusToString());
      }
      drake::log()->info(result.get_solution_result());
      const Eigen::VectorXd q_sol = result.GetSolution(ik.q());
      drake::log()->info("q_sol: {}\n", fmt_eigen(q_sol.transpose()));
      const auto q_sol_ad = math::InitializeAutoDiff(q_sol);
      AutoDiffVecXd y_sol_ad;
      in_collision_constraint->Eval(q_sol_ad, &y_sol_ad);
      drake::log()->info("y_sol value {}, derivatives {}\n",
                         y_sol_ad(0).value(),
                         fmt_eigen(y_sol_ad(0).derivatives().transpose()));
    }
  }
}

TEST_F(Planar2DofTest, InCollisionTest) {
  auto diagram = builder_.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  systems::Context<double>* plant_context =
      &(diagram->mutable_plant_context(diagram_context.get()));
  const double normalizer = 1;
  auto in_collision_constraint = std::make_shared<InCollisionConstraint>(
      &(diagram->plant()), 0, normalizer, plant_context);

  InverseKinematics ik(diagram->plant(), plant_context);
  ik.get_mutable_prog()->AddConstraint(in_collision_constraint, ik.q());

  const Eigen::Vector2d q_joint_lower_limit =
      diagram->plant().GetPositionLowerLimits();
  const Eigen::Vector2d q_joint_upper_limit =
      diagram->plant().GetPositionUpperLimits();

  Eigen::VectorXd q0_values =
      Eigen::VectorXd::LinSpaced(10, -M_PI / 2, M_PI / 2);
  Eigen::VectorXd q1_values = Eigen::VectorXd::LinSpaced(
      10, q_joint_lower_limit(1), q_joint_upper_limit(1));
  Eigen::VectorXd q_guess = Eigen::VectorXd::Zero(ik.q().rows());
  solvers::SolverOptions solver_options;
  solver_options.SetOption(solvers::IpoptSolver::id(), "max_iter", 1000);
  if (solvers::SnoptSolver::is_available() &&
      solvers::SnoptSolver::is_enabled()) {
    // TODO(hongkai.dai): For some reasons IPOPT often fails to solve the
    // problem and exceeds the maximal iteration. I will look into this later.
    for (int i = 0; i < q0_values.rows(); ++i) {
      q_guess(0) = q0_values(i);
      for (int j = 0; j < q1_values.rows(); ++j) {
        q_guess(1) = q1_values(j);
        const auto result = solvers::Solve(ik.prog(), q_guess, solver_options);
        EXPECT_TRUE(result.is_success());
        if (!result.is_success()) {
          if (result.get_solver_id() == solvers::IpoptSolver::id()) {
            drake::log()->info(
                "ipopt status: {}, {}",
                result.get_solver_details<solvers::IpoptSolver>().status,
                result.get_solver_details<solvers::IpoptSolver>()
                    .ConvertStatusToString());
          } else if (result.get_solver_id() == solvers::SnoptSolver::id()) {
            drake::log()->info(
                "snopt info: {}",
                result.get_solver_details<solvers::SnoptSolver>().info);
          }
          drake::log()->info(result.get_solution_result());
          const Eigen::VectorXd q_sol = result.GetSolution(ik.q());
          drake::log()->info("q_sol: {}", fmt_eigen(q_sol.transpose()));
          const auto q_sol_ad = math::InitializeAutoDiff(q_sol);
          AutoDiffVecXd y_sol_ad;
          in_collision_constraint->Eval(q_sol_ad, &y_sol_ad);
          drake::log()->info("y_sol value {}, derivatives {}\n",
                             y_sol_ad(0).value(),
                             fmt_eigen(y_sol_ad(0).derivatives().transpose()));
        }
      }
    }
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
