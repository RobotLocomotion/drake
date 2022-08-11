#include "drake/traj_opt/trajectory_optimizer.h"

#include <algorithm>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/traj_opt/inverse_dynamics_partials.h"
#include "drake/traj_opt/problem_definition.h"
#include "drake/traj_opt/trajectory_optimizer_state.h"
#include "drake/traj_opt/trajectory_optimizer_workspace.h"
#include "drake/traj_opt/velocity_partials.h"

namespace drake {
namespace traj_opt {

class TrajectoryOptimizerTester {
 public:
  TrajectoryOptimizerTester() = delete;

  static double CalcCost(const TrajectoryOptimizer<double>& optimizer,
                         const std::vector<VectorXd>& q,
                         const std::vector<VectorXd>& v,
                         const std::vector<VectorXd>& tau,
                         TrajectoryOptimizerWorkspace<double>* workspace) {
    return optimizer.CalcCost(q, v, tau, workspace);
  }

  static void CalcVelocities(const TrajectoryOptimizer<double>& optimizer,
                             const std::vector<VectorXd>& q,
                             std::vector<VectorXd>* v) {
    optimizer.CalcVelocities(q, v);
  }

  static void CalcAccelerations(const TrajectoryOptimizer<double>& optimizer,
                                const std::vector<VectorXd>& v,
                                std::vector<VectorXd>* a) {
    optimizer.CalcAccelerations(v, a);
  }

  static void CalcInverseDynamics(
      const TrajectoryOptimizer<double>& optimizer,
      const std::vector<VectorXd>& q, const std::vector<VectorXd>& v,
      const std::vector<VectorXd>& a,
      TrajectoryOptimizerWorkspace<double>* workspace,
      std::vector<VectorXd>* tau) {
    optimizer.CalcInverseDynamics(q, v, a, workspace, tau);
  }

  static void CalcInverseDynamicsPartials(
      const TrajectoryOptimizer<double>& optimizer,
      const std::vector<VectorXd>& q, const std::vector<VectorXd>& v,
      const std::vector<VectorXd>& a, const std::vector<VectorXd>& tau,
      TrajectoryOptimizerWorkspace<double>* workspace,
      InverseDynamicsPartials<double>* id_partials) {
    optimizer.CalcInverseDynamicsPartials(q, v, a, tau, workspace, id_partials);
  }

  static void CalcGradientFiniteDiff(
      const TrajectoryOptimizer<double>& optimizer,
      const TrajectoryOptimizerState<double>& state, EigenPtr<VectorXd> g) {
    optimizer.CalcGradientFiniteDiff(state, g);
  }

  static void CalcInverseDynamicsSingleTimeStep(
      const TrajectoryOptimizer<double>& optimizer, const VectorXd& q,
      const VectorXd& v, const VectorXd a,
      TrajectoryOptimizerWorkspace<double>* workspace, VectorXd* tau) {
    optimizer.CalcInverseDynamicsSingleTimeStep(q, v, a, workspace, tau);
  }
};

namespace internal {

using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorXd;
using multibody::DiscreteContactSolver;
using multibody::MultibodyPlant;
using multibody::Parser;
using test::LimitMalloc;

/**
 * Test our optimizer with a simple pendulum swingup task.
 */
GTEST_TEST(TrajectoryOptimizerTest, PendulumSwingup) {
  // Define the optimization problem
  const int num_steps = 20;
  const double dt = 5e-2;

  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = Vector1d(0.1);
  opt_prob.v_init = Vector1d(0.0);
  opt_prob.Qq = 1.0 * MatrixXd::Identity(1, 1);
  opt_prob.Qv = 0.1 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_q = 1000 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_v = 1 * MatrixXd::Identity(1, 1);
  opt_prob.R = 0.01 * MatrixXd::Identity(1, 1);
  opt_prob.q_nom = Vector1d(M_PI);
  opt_prob.v_nom = Vector1d(0);

  // Create a pendulum model
  MultibodyPlant<double> plant(dt);
  const std::string urdf_file =
      FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  Parser(&plant).AddAllModelsFromFile(urdf_file);
  plant.Finalize();

  // Create an optimizer
  SolverParameters solver_params;
  solver_params.max_iterations = 20;
  solver_params.verbose = false;

  TrajectoryOptimizer<double> optimizer(&plant, opt_prob, solver_params);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();

  // Set an initial guess
  std::vector<VectorXd> q_guess;
  for (int t = 0; t <= num_steps; ++t) {
    q_guess.push_back(opt_prob.q_init);
  }

  // Solve the optimization problem
  TrajectoryOptimizerSolution<double> solution;
  TrajectoryOptimizerStats<double> stats;

  SolverFlag status = optimizer.Solve(q_guess, &solution, &stats);
  EXPECT_EQ(status, SolverFlag::kSuccess);

  // With such a large penalty on the final position, and such a low control
  // penalty, we should be close to the target position at the last timestep.
  EXPECT_NEAR(solution.q[num_steps][0], opt_prob.q_nom[0], 1e-3);
}

/**
 * Test our computation of the Hessian on a system
 * with more than one DoF.
 */
GTEST_TEST(TrajectoryOptimizerTest, HessianAcrobot) {
  // Define an optimization problem.
  const int num_steps = 5;
  const double dt = 1e-2;

  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = Vector2d(0.1, 0.2);
  opt_prob.v_init = Vector2d(-0.01, 0.03);
  opt_prob.Qq = 0.1 * MatrixXd::Identity(2, 2);
  opt_prob.Qv = 0.2 * MatrixXd::Identity(2, 2);
  opt_prob.Qf_q = 0.3 * MatrixXd::Identity(2, 2);
  opt_prob.Qf_v = 0.4 * MatrixXd::Identity(2, 2);
  opt_prob.R = 0.01 * MatrixXd::Identity(2, 2);
  opt_prob.q_nom = Vector2d(1.5, -0.1);
  opt_prob.v_nom = Vector2d(0.2, 0.1);

  // Create a pendulum model
  MultibodyPlant<double> plant(dt);
  const std::string urdf_file =
      FindResourceOrThrow("drake/multibody/benchmarks/acrobot/acrobot.urdf");
  Parser(&plant).AddAllModelsFromFile(urdf_file);
  plant.Finalize();

  // Create an optimizer
  TrajectoryOptimizer<double> optimizer(&plant, opt_prob);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();

  // Make some fake data
  std::vector<VectorXd> q(num_steps + 1);
  q[0] = opt_prob.q_init;
  for (int t = 1; t <= num_steps; ++t) {
    q[t] = q[t - 1] + dt * opt_prob.v_init;
  }
  state.set_q(q);

  // Compute the Gauss-Newton Hessian approximation numerically
  const int nq = plant.num_positions();
  const int num_vars = nq * (num_steps + 1);
  PentaDiagonalMatrix<double> H_sparse(num_steps + 1, nq);
  optimizer.CalcHessian(state, &H_sparse);
  MatrixXd H = H_sparse.MakeDense();

  // Set up an autodiff copy of the optimizer and plant
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_ad =
      systems::System<double>::ToAutoDiffXd(plant);
  TrajectoryOptimizer<AutoDiffXd> optimizer_ad(plant_ad.get(), opt_prob);
  TrajectoryOptimizerState<AutoDiffXd> state_ad = optimizer_ad.CreateState();

  std::vector<VectorX<AutoDiffXd>> q_ad(num_steps + 1, VectorX<AutoDiffXd>(2));
  int ad_idx = 0;  // index for autodiff variables
  for (int t = 0; t <= num_steps; ++t) {
    for (int i = 0; i < nq; ++i) {
      q_ad[t].segment<1>(i) =
          math::InitializeAutoDiff(q[t].segment<1>(i), num_vars, ad_idx);
      ++ad_idx;
    }
  }
  state_ad.set_q(q_ad);
  AutoDiffXd L_ad = optimizer_ad.EvalCost(state_ad);  // forces cache update

  // Formulate an equivalent least-squares problem, where
  //
  //    L(q) = 1/2 r(q)'*r(q)
  //    J = dr(q)/dq
  //
  // and the gradient and Gauss-Newton Hessian approximation are given by
  //
  //    g = J'r,
  //    H = J'J.
  //
  Matrix2d Qq_sqrt = (dt * 2 * opt_prob.Qq).cwiseSqrt();  // diagonal matrices
  Matrix2d Qv_sqrt = (dt * 2 * opt_prob.Qv).cwiseSqrt();
  Matrix2d R_sqrt = (dt * 2 * opt_prob.R).cwiseSqrt();
  Matrix2d Qfq_sqrt = (2 * opt_prob.Qf_q).cwiseSqrt();
  Matrix2d Qfv_sqrt = (2 * opt_prob.Qf_v).cwiseSqrt();

  const std::vector<VectorX<AutoDiffXd>>& v_ad = optimizer_ad.EvalV(state_ad);
  const std::vector<VectorX<AutoDiffXd>>& u_ad = optimizer_ad.EvalTau(state_ad);

  // Here we construct the residual
  //        [        ...           ]
  //        [ sqrt(Qq)*(q_t-q_nom) ]
  // r(q) = [ sqrt(Qv)*(v_t-v_nom) ]
  //        [ sqrt(R) * tau_t      ]
  //        [        ...           ]
  //        [ sqrt(Qfq)*(q_T-q_nom)]
  //        [ sqrt(Qfv)*(v_T-v_nom)]
  VectorX<AutoDiffXd> r(num_steps * 6 + 4);
  r.setZero();
  for (int t = 0; t < num_steps; ++t) {
    r.segment(t * 6, 2) = Qq_sqrt * (q_ad[t] - opt_prob.q_nom);
    r.segment(t * 6 + 2, 2) = Qv_sqrt * (v_ad[t] - opt_prob.v_nom);
    r.segment(t * 6 + 4, 2) = R_sqrt * u_ad[t];
  }
  r.segment(num_steps * 6, 2) = Qfq_sqrt * (q_ad[num_steps] - opt_prob.q_nom);
  r.segment(num_steps * 6 + 2, 2) =
      Qfv_sqrt * (v_ad[num_steps] - opt_prob.v_nom);

  MatrixXd J = math::ExtractGradient(r);
  AutoDiffXd L_lsqr = 0.5 * r.transpose() * r;
  VectorXd g_lsqr = J.transpose() * math::ExtractValue(r);
  MatrixXd H_lsqr = J.transpose() * J;

  // Check that the cost from our least-squares formulation is correct
  const double kToleranceCost = 10 * std::numeric_limits<double>::epsilon();
  double L = optimizer.EvalCost(state);
  EXPECT_NEAR(L, L_lsqr.value(), kToleranceCost);

  // Check that the gradient from our least-squares formulation matches what we
  // compute analytically. Primary source of error is our use of finite
  // differences to compute dtau/dq. We ignore the top block of the gradient,
  // since this is overwritten with zero, because q0 is fixed.
  const double kToleranceGrad =
      sqrt(std::numeric_limits<double>::epsilon()) / dt;
  VectorXd g(num_vars);
  optimizer.CalcGradient(state, &g);
  EXPECT_TRUE(CompareMatrices(g.bottomRows(num_steps * nq),
                              g_lsqr.bottomRows(num_steps * nq), kToleranceGrad,
                              MatrixCompareType::relative));

  // Check that the Hessian approximation from least-squares (Gauss-Newton)
  // matches what we compute analytically. Finite differences is again the
  // primary source of error. We ignore the rows and columns, since these are
  // overwritten to fix q0.
  const double kToleranceHess = sqrt(std::numeric_limits<double>::epsilon());
  EXPECT_TRUE(
      CompareMatrices(H.bottomRightCorner(num_steps * nq, num_steps * nq),
                      H_lsqr.bottomRightCorner(num_steps * nq, num_steps * nq),
                      kToleranceHess, MatrixCompareType::relative));
}

/**
 * Test our computation of the Hessian by comparing
 * with autodiff.
 */
GTEST_TEST(TrajectoryOptimizerTest, HessianPendulum) {
  // Define an optimization problem.
  const int num_steps = 5;
  const double dt = 1e-2;

  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = Vector1d(0.1);
  opt_prob.v_init = Vector1d(0.0);
  opt_prob.Qq = 0.1 * MatrixXd::Identity(1, 1);
  opt_prob.Qv = 0.2 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_q = 0.3 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_v = 0.4 * MatrixXd::Identity(1, 1);
  opt_prob.R = 0.05 * MatrixXd::Identity(1, 1);
  opt_prob.q_nom = Vector1d(0.1);
  opt_prob.v_nom = Vector1d(-0.1);

  // Create a pendulum model
  MultibodyPlant<double> plant(dt);
  const std::string urdf_file =
      FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  Parser(&plant).AddAllModelsFromFile(urdf_file);
  plant.Finalize();

  // Create an optimizer
  TrajectoryOptimizer<double> optimizer(&plant, opt_prob);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();

  // Make some fake data
  std::vector<VectorXd> q(num_steps + 1);
  q[0] = opt_prob.q_init;
  for (int t = 1; t <= num_steps; ++t) {
    q[t] = q[t - 1] + 0.1 * dt * MatrixXd::Identity(1, 1);
  }
  state.set_q(q);

  // Compute the Hessian analytically
  const int nq = plant.num_positions();
  const int num_vars = nq * (num_steps + 1);
  PentaDiagonalMatrix<double> H_sparse(num_steps + 1, nq);
  optimizer.CalcHessian(state, &H_sparse);
  MatrixXd H = H_sparse.MakeDense();

  // Compute the Hessian using autodiff
  // Note that this is the true Hessian, and not the Gauss-Newton approximation
  // that we will use. But for this simple pendulum the two are very close
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_ad =
      systems::System<double>::ToAutoDiffXd(plant);
  TrajectoryOptimizer<AutoDiffXd> optimizer_ad(plant_ad.get(), opt_prob);
  TrajectoryOptimizerState<AutoDiffXd> state_ad = optimizer_ad.CreateState();

  std::vector<VectorX<AutoDiffXd>> q_ad(num_steps + 1);
  for (int t = 0; t <= num_steps; ++t) {
    q_ad[t] = math::InitializeAutoDiff(q[t], num_steps + 1, t);
  }
  state_ad.set_q(q_ad);

  VectorX<AutoDiffXd> g_ad(num_vars);
  optimizer_ad.CalcGradient(state_ad, &g_ad);
  MatrixXd H_ad = math::ExtractGradient(g_ad);

  // We overwrite the first row and column of the Hessian, so we won't compare
  // those
  const double kTolerance = sqrt(std::numeric_limits<double>::epsilon()) / dt;
  EXPECT_TRUE(
      CompareMatrices(H.bottomRightCorner(num_steps * nq, num_steps * nq),
                      H_ad.bottomRightCorner(num_steps * nq, num_steps * nq),
                      kTolerance, MatrixCompareType::relative));
}

GTEST_TEST(TrajectoryOptimizerTest, AutodiffGradient) {
  // Test our gradient computations using autodiff
  const int num_steps = 5;
  const double dt = 1e-2;

  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = Vector1d(0.1);
  opt_prob.v_init = Vector1d(0.0);
  opt_prob.Qq = 0.1 * MatrixXd::Identity(1, 1);
  opt_prob.Qv = 0.2 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_q = 0.3 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_v = 0.4 * MatrixXd::Identity(1, 1);
  opt_prob.R = 0.5 * MatrixXd::Identity(1, 1);
  opt_prob.q_nom = Vector1d(0.1);
  opt_prob.v_nom = Vector1d(-0.1);

  // Create a pendulum model
  MultibodyPlant<double> plant(dt);
  const std::string urdf_file =
      FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  Parser(&plant).AddAllModelsFromFile(urdf_file);
  plant.Finalize();

  // Create an optimizer
  TrajectoryOptimizer<double> optimizer(&plant, opt_prob);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();
  TrajectoryOptimizerWorkspace<double> workspace(num_steps, plant);

  // Make some fake data
  std::vector<VectorXd> q(num_steps + 1);
  q[0] = opt_prob.q_init;
  for (int t = 1; t <= num_steps; ++t) {
    q[t] = q[t - 1] + 0.1 * dt * MatrixXd::Identity(1, 1);
  }
  state.set_q(q);

  // Compute the gradient analytically
  VectorXd g(plant.num_positions() * (num_steps + 1));
  optimizer.CalcGradient(state, &g);

  // Compute the gradient using autodiff
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_ad =
      systems::System<double>::ToAutoDiffXd(plant);
  TrajectoryOptimizer<AutoDiffXd> optimizer_ad(plant_ad.get(), opt_prob);
  TrajectoryOptimizerState<AutoDiffXd> state_ad = optimizer_ad.CreateState();
  TrajectoryOptimizerWorkspace<AutoDiffXd> workspace_ad(num_steps, *plant_ad);

  std::vector<VectorX<AutoDiffXd>> q_ad(num_steps + 1);
  for (int t = 0; t <= num_steps; ++t) {
    q_ad[t] = math::InitializeAutoDiff(q[t], num_steps + 1, t);
  }
  state_ad.set_q(q_ad);

  AutoDiffXd cost_ad = optimizer_ad.EvalCost(state_ad);
  VectorXd g_ad = cost_ad.derivatives();

  // We neglect the top row of the gradient, since we are setting it to zero.
  const double kTolerance = sqrt(std::numeric_limits<double>::epsilon());
  EXPECT_TRUE(CompareMatrices(g.bottomRows(num_steps),
                              g_ad.bottomRows(num_steps), kTolerance,
                              MatrixCompareType::relative));
}

GTEST_TEST(TrajectoryOptimizerTest, CalcGradientKuka) {
  const int num_steps = 3;
  const double dt = 1e-2;

  // Create a robot model
  MultibodyPlant<double> plant(dt);
  const std::string urdf_file = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_no_collision.urdf");

  Parser(&plant).AddAllModelsFromFile(urdf_file);
  plant.set_discrete_contact_solver(DiscreteContactSolver::kSap);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"));
  plant.Finalize();

  // Set up an optimization problem
  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = VectorXd(7);
  opt_prob.q_init.setConstant(0.1);
  opt_prob.v_init = VectorXd(7);
  opt_prob.v_init.setConstant(0.2);
  opt_prob.Qq = 0.1 * MatrixXd::Identity(7, 7);
  opt_prob.Qv = 0.2 * MatrixXd::Identity(7, 7);
  opt_prob.Qf_q = 0.3 * MatrixXd::Identity(7, 7);
  opt_prob.Qf_v = 0.4 * MatrixXd::Identity(7, 7);
  opt_prob.R = 0.5 * MatrixXd::Identity(7, 7);
  opt_prob.q_nom = VectorXd(7);
  opt_prob.q_nom.setConstant(-0.2);
  opt_prob.v_nom = VectorXd(7);
  opt_prob.v_nom.setConstant(-0.1);

  // Create an optimizer
  TrajectoryOptimizer<double> optimizer(&plant, opt_prob);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();
  TrajectoryOptimizerWorkspace<double> workspace(num_steps, plant);

  // Make some fake data
  std::vector<VectorXd> q(num_steps + 1);
  q[0] = opt_prob.q_init;
  for (int t = 1; t <= num_steps; ++t) {
    q[t] = q[t - 1] + 0.1 * dt * MatrixXd::Identity(7, 7);
  }
  state.set_q(q);

  // Compute the ("ground truth") gradient with finite differences
  VectorXd g_gt(plant.num_positions() * (num_steps + 1));
  TrajectoryOptimizerTester::CalcGradientFiniteDiff(optimizer, state, &g_gt);

  // Compute the gradient with our method
  VectorXd g(plant.num_positions() * (num_steps + 1));
  optimizer.CalcGradient(state, &g);

  // Looks like we're losing a lot of precision here, but I think that's because
  // it comes from several sources:
  //    1) finite differences give us eps^(1/2)
  //    2) computing v(q) gives us a factor of 1/dt
  //    3) computing tau(v(q)) gives us an additional factor of 1/dt
  const double kTolerance =
      pow(std::numeric_limits<double>::epsilon(), 1. / 2.) / dt / dt;
  EXPECT_TRUE(
      CompareMatrices(g, g_gt, kTolerance, MatrixCompareType::relative));
}

GTEST_TEST(TrajectoryOptimizerTest, CalcGradientPendulumNoGravity) {
  const int num_steps = 10;
  const double dt = 5e-2;

  // Set up a system model: pendulum w/o gravity yields a linear system
  MultibodyPlant<double> plant(dt);
  const std::string urdf_file =
      FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  Parser(&plant).AddAllModelsFromFile(urdf_file);
  plant.mutable_gravity_field().set_gravity_vector(VectorXd::Zero(3));
  plant.Finalize();

  // Set up a toy optimization problem
  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = Vector1d(0.0);
  opt_prob.v_init = Vector1d(0.0);
  opt_prob.Qq = 0.1 * MatrixXd::Identity(1, 1);
  opt_prob.Qv = 0.2 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_q = 0.3 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_v = 0.4 * MatrixXd::Identity(1, 1);
  opt_prob.R = 0.5 * MatrixXd::Identity(1, 1);
  opt_prob.q_nom = Vector1d(M_PI);
  opt_prob.v_nom = Vector1d(-0.1);

  // Create an optimizer
  TrajectoryOptimizer<double> optimizer(&plant, opt_prob);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();
  TrajectoryOptimizerWorkspace<double> workspace(num_steps, plant);

  // Create some fake data
  std::vector<VectorXd> q;
  q.push_back(Vector1d(0.0000000000000000000000000));
  q.push_back(Vector1d(0.0950285641187840757204697));
  q.push_back(Vector1d(0.2659896360172592788551071));
  q.push_back(Vector1d(0.4941147113506765831125733));
  q.push_back(Vector1d(0.7608818755930255584019051));
  q.push_back(Vector1d(1.0479359055822168311777887));
  q.push_back(Vector1d(1.3370090901260500704239575));
  q.push_back(Vector1d(1.6098424281109515732168802));
  q.push_back(Vector1d(1.8481068641834854648919872));
  q.push_back(Vector1d(2.0333242222438583368671061));
  q.push_back(Vector1d(2.1467874956452459578315484));
  state.set_q(q);

  // Compute the ground truth gradient with autodiff
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_ad =
      systems::System<double>::ToAutoDiffXd(plant);
  TrajectoryOptimizer<AutoDiffXd> optimizer_ad(plant_ad.get(), opt_prob);
  TrajectoryOptimizerState<AutoDiffXd> state_ad = optimizer_ad.CreateState();
  TrajectoryOptimizerWorkspace<AutoDiffXd> workspace_ad(num_steps, *plant_ad);

  std::vector<VectorX<AutoDiffXd>> q_ad(num_steps + 1);
  for (int t = 0; t <= num_steps; ++t) {
    q_ad[t] = math::InitializeAutoDiff(q[t], num_steps + 1, t);
  }
  state_ad.set_q(q_ad);

  AutoDiffXd cost_ad = optimizer_ad.EvalCost(state_ad);
  VectorXd g_gt = cost_ad.derivatives();
  g_gt[0] = 0;  // q0 is fixed

  // Compute the gradient with our method
  VectorXd g(plant.num_positions() * (num_steps + 1));
  optimizer.CalcGradient(state, &g);

  // Even without gravity (a.k.a. linear system), finite differences is only
  // accurate to sqrt(epsilon)
  const double kTolerance = sqrt(std::numeric_limits<double>::epsilon());
  EXPECT_TRUE(
      CompareMatrices(g, g_gt, kTolerance, MatrixCompareType::relative));

  // Evaluate error in d(tau)/d(q), as compared to the ground truth pendulum
  // model
  //
  //     m*l^2*a + m*g*l*sin(q) + b*v = tau
  //
  // where q is the joint angle and a = dv/dt, v = dq/dt.
  const double m = 1.0;
  const double l = 0.5;
  const double b = 0.1;

  InverseDynamicsPartials<double> id_partials_gt(num_steps, 1, 1);
  for (int t = 0; t < num_steps; ++t) {
    // dtau[t]/dq[t+1]
    id_partials_gt.dtau_dqp[t](0, 0) = 1 / dt / dt * m * l * l + 1 / dt * b;

    // dtau[t]/dq[t]
    if (t == 0) {
      // v[0] is constant
      id_partials_gt.dtau_dqt[t](0, 0) = -1 / dt / dt * m * l * l - 1 / dt * b;
    } else {
      id_partials_gt.dtau_dqt[t](0, 0) = -2 / dt / dt * m * l * l - 1 / dt * b;
    }

    // dtau[t]/dq[t-1]
    id_partials_gt.dtau_dqm[t](0, 0) = 1 / dt / dt * m * l * l;

    // Derivatives w.r.t. q[t-1] do not exist
    if (t == 0) {
      id_partials_gt.dtau_dqm[t](0, 0) = NAN;
    }
  }

  const InverseDynamicsPartials<double>& id_partials =
      optimizer.EvalInverseDynamicsPartials(state);
  for (int t = 0; t < num_steps; ++t) {
    if (t > 0) {
      EXPECT_NEAR(id_partials.dtau_dqm[t](0), id_partials_gt.dtau_dqm[t](0),
                  10 * kTolerance);
    }
    EXPECT_NEAR(id_partials.dtau_dqt[t](0), id_partials_gt.dtau_dqt[t](0),
                10 * kTolerance);
    EXPECT_NEAR(id_partials.dtau_dqp[t](0), id_partials_gt.dtau_dqp[t](0),
                10 * kTolerance);
  }

  // Check that mass matrix of the plant is truely constant
  auto plant_context = plant.CreateDefaultContext();
  MatrixXd M(1, 1);
  for (int t = 0; t < num_steps; ++t) {
    plant.SetPositions(plant_context.get(), q[t]);
    plant.SetVelocities(plant_context.get(), optimizer.EvalV(state)[t]);
    plant.CalcMassMatrix(*plant_context, &M);

    EXPECT_NEAR(M(0, 0), m * l * l, std::numeric_limits<double>::epsilon());
  }

  // Check our computation of tau(q)
  double tau;
  double tau_gt;
  const std::vector<VectorXd>& v = optimizer.EvalV(state);
  const std::vector<VectorXd>& a = optimizer.EvalA(state);
  const std::vector<VectorXd>& tau_comp = optimizer.EvalTau(state);
  const double kToleranceTau = 10 * std::numeric_limits<double>::epsilon();
  for (int t = 0; t < num_steps; ++t) {
    tau = tau_comp[t](0);
    tau_gt = m * l * l * a[t](0) + b * v[t + 1](0);
    EXPECT_NEAR(tau_gt, tau, kToleranceTau);
  }
}

GTEST_TEST(TrajectoryOptimizerTest, CalcGradientPendulum) {
  // Set up an optimization problem
  const int num_steps = 5;
  const double dt = 1e-3;

  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = Vector1d(0.1);
  opt_prob.v_init = Vector1d(0.0);
  opt_prob.Qq = 0.1 * MatrixXd::Identity(1, 1);
  opt_prob.Qv = 0.2 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_q = 0.3 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_v = 0.4 * MatrixXd::Identity(1, 1);
  opt_prob.R = 0.5 * MatrixXd::Identity(1, 1);
  opt_prob.q_nom = Vector1d(0.1);
  opt_prob.v_nom = Vector1d(-0.1);

  // Create a pendulum model
  MultibodyPlant<double> plant(dt);
  const std::string urdf_file =
      FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  Parser(&plant).AddAllModelsFromFile(urdf_file);
  plant.set_discrete_contact_solver(DiscreteContactSolver::kSap);
  plant.Finalize();

  // Create an optimizer
  TrajectoryOptimizer<double> optimizer(&plant, opt_prob);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();
  TrajectoryOptimizerWorkspace<double> workspace(num_steps, plant);

  // Make some fake data
  std::vector<VectorXd> q(num_steps + 1);
  q[0] = opt_prob.q_init;
  for (int t = 1; t <= num_steps; ++t) {
    q[t] = q[t - 1] + 0.1 * dt * MatrixXd::Identity(1, 1);
  }
  state.set_q(q);

  // Compute the ("ground truth") gradient with finite differences
  VectorXd g_gt(plant.num_positions() * (num_steps + 1));
  TrajectoryOptimizerTester::CalcGradientFiniteDiff(optimizer, state, &g_gt);

  // Compute the gradient with our method
  VectorXd g(plant.num_positions() * (num_steps + 1));
  optimizer.CalcGradient(state, &g);

  // Compare the two
  const double kTolerance = pow(std::numeric_limits<double>::epsilon(), 0.5);
  EXPECT_TRUE(
      CompareMatrices(g, g_gt, kTolerance, MatrixCompareType::relative));
}

GTEST_TEST(TrajectoryOptimizerTest, PendulumDtauDq) {
  const int num_steps = 5;
  const double dt = 1e-2;

  // Set up a system model
  MultibodyPlant<double> plant(dt);
  const std::string urdf_file =
      FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  Parser(&plant).AddAllModelsFromFile(urdf_file);
  plant.set_discrete_contact_solver(DiscreteContactSolver::kSap);
  plant.Finalize();

  // Create a trajectory optimizer
  ProblemDefinition opt_prob;
  opt_prob.q_init = Vector1d(0.0);
  opt_prob.v_init = Vector1d(0.1);
  opt_prob.num_steps = num_steps;
  TrajectoryOptimizer<double> optimizer(&plant, opt_prob);
  TrajectoryOptimizerWorkspace<double> workspace(num_steps, plant);

  // Create some fake data
  std::vector<VectorXd> q;
  q.push_back(opt_prob.q_init);
  for (int t = 1; t <= num_steps; ++t) {
    q.push_back(Vector1d(0.0 + 0.6 * t));
  }

  // Compute inverse dynamics partials
  InverseDynamicsPartials<double> grad_data(num_steps, 1, 1);
  std::vector<VectorXd> v(num_steps + 1);
  std::vector<VectorXd> a(num_steps);
  std::vector<VectorXd> tau(num_steps);
  TrajectoryOptimizerTester::CalcVelocities(optimizer, q, &v);
  TrajectoryOptimizerTester::CalcAccelerations(optimizer, v, &a);
  TrajectoryOptimizerTester::CalcInverseDynamics(optimizer, q, v, a, &workspace,
                                                 &tau);
  TrajectoryOptimizerTester::CalcInverseDynamicsPartials(
      optimizer, q, v, a, tau, &workspace, &grad_data);

  // Compute ground truth partials from the pendulum model
  //
  //     m*l^2*a + m*g*l*sin(q) + b*v = tau
  //
  // where q is the joint angle and a = dv/dt, v = dq/dt.
  const double m = 1.0;
  const double l = 0.5;
  const double b = 0.1;
  const double g = 9.81;

  InverseDynamicsPartials<double> grad_data_gt(num_steps, 1, 1);
  for (int t = 0; t < num_steps; ++t) {
    // dtau[t]/dq[t+1]
    grad_data_gt.dtau_dqp[t](0, 0) =
        1 / dt / dt * m * l * l + 1 / dt * b + m * g * l * cos(q[t + 1](0));

    // dtau[t]/dq[t]
    grad_data_gt.dtau_dqt[t](0, 0) = -2 / dt / dt * m * l * l - 1 / dt * b;

    if (t == 0) {
      // v[0] is constant
      grad_data_gt.dtau_dqt[t](0, 0) = -1 / dt / dt * m * l * l - 1 / dt * b;
    }

    // dtau[t]/dq[t-1]
    grad_data_gt.dtau_dqm[t](0, 0) = 1 / dt / dt * m * l * l;

    // Derivatives w.r.t. q[t-1] do not exist
    if (t == 0) {
      grad_data_gt.dtau_dqm[t](0, 0) = NAN;
    }
  }

  // Compare the computed values and the analytical ground truth
  const double kTolerance = sqrt(std::numeric_limits<double>::epsilon());
  for (int t = 0; t < num_steps; ++t) {
    EXPECT_TRUE(CompareMatrices(grad_data.dtau_dqm[t], grad_data_gt.dtau_dqm[t],
                                kTolerance, MatrixCompareType::relative));
    EXPECT_TRUE(CompareMatrices(grad_data.dtau_dqt[t], grad_data_gt.dtau_dqt[t],
                                kTolerance, MatrixCompareType::relative));
    EXPECT_TRUE(CompareMatrices(grad_data.dtau_dqp[t], grad_data_gt.dtau_dqp[t],
                                kTolerance, MatrixCompareType::relative));
  }
}

/**
 * Check the precision of our computation of costs using the state abstraction.
 */
GTEST_TEST(TrajectoryOptimizerTest, CalcCostFromState) {
  const int num_steps = 10;
  const double dt = 5e-2;

  // Set up a system model: pendulum w/o gravity yields a linear system
  MultibodyPlant<double> plant(dt);
  const std::string urdf_file =
      FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  Parser(&plant).AddAllModelsFromFile(urdf_file);
  plant.mutable_gravity_field().set_gravity_vector(VectorXd::Zero(3));
  plant.Finalize();

  // Set up a toy optimization problem
  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = Vector1d(0.0);
  opt_prob.v_init = Vector1d(0.0);
  opt_prob.Qq = 0.0 * MatrixXd::Identity(1, 1);
  opt_prob.Qv = 0.1 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_q = 10.0 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_v = 1.0 * MatrixXd::Identity(1, 1);
  opt_prob.R = 1.0 * MatrixXd::Identity(1, 1);
  opt_prob.q_nom = Vector1d(M_PI);
  opt_prob.v_nom = Vector1d(0.0);

  // Create some fake data, which are very close to optimality.
  std::vector<VectorXd> q;
  q.push_back(Vector1d(0.0000000000000000000000000));
  q.push_back(Vector1d(0.0950285641187840757204697));
  q.push_back(Vector1d(0.2659896360172592788551071));
  q.push_back(Vector1d(0.4941147113506765831125733));
  q.push_back(Vector1d(0.7608818755930255584019051));
  q.push_back(Vector1d(1.0479359055822168311777887));
  q.push_back(Vector1d(1.3370090901260500704239575));
  q.push_back(Vector1d(1.6098424281109515732168802));
  q.push_back(Vector1d(1.8481068641834854648919872));
  q.push_back(Vector1d(2.0333242222438583368671061));
  q.push_back(Vector1d(2.1467874956452459578315484));

  // Compute the cost as a function of state
  TrajectoryOptimizer<double> optimizer(&plant, opt_prob);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();
  state.set_q(q);
  double L = optimizer.EvalCost(state);

  // Compute the ground truth cost using an analytical model of the (linear)
  // pendulum dynamics.
  //
  //     m*l^2*a + m*g*l*sin(q) + b*v = tau
  //
  // where q is the joint angle and a = dv/dt, v = dq/dt, and g=0.
  const double m = 1.0;
  const double l = 0.5;
  const double b = 0.1;

  double L_gt = 0;
  double qt;
  double vt = opt_prob.v_init[0];
  double vp;
  double ut;
  for (int t = 0; t < num_steps; ++t) {
    qt = q[t][0];
    if (t > 0) {
      vt = (q[t][0] - q[t - 1][0]) / dt;
    }
    vp = (q[t + 1][0] - q[t][0]) / dt;
    ut = m * l * l * (vp - vt) / dt + b * vp;

    L_gt += dt * (qt - opt_prob.q_nom(0)) * opt_prob.Qq(0) *
            (qt - opt_prob.q_nom(0));
    L_gt += dt * (vt - opt_prob.v_nom(0)) * opt_prob.Qv(0) *
            (vt - opt_prob.v_nom(0));
    L_gt += dt * ut * opt_prob.R(0) * ut;
  }

  qt = q[num_steps][0];
  vt = (q[num_steps][0] - q[num_steps - 1][0]) / dt;
  L_gt +=
      (qt - opt_prob.q_nom(0)) * opt_prob.Qf_q(0) * (qt - opt_prob.q_nom(0));
  L_gt +=
      (vt - opt_prob.v_nom(0)) * opt_prob.Qf_v(0) * (vt - opt_prob.v_nom(0));

  const double kTolerance = 100 * std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(L, L_gt, kTolerance);
}

/**
 * Test our computation of the total cost L(q)
 */
GTEST_TEST(TrajectoryOptimizerTest, CalcCost) {
  const int num_steps = 100;
  const double dt = 1e-2;

  // Set up an (empty) system model
  MultibodyPlant<double> plant(dt);
  plant.Finalize();

  // Set up the optimization problem
  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = Vector2d(0.2, 0.1);
  opt_prob.v_init = Vector2d(-0.1, 0.0);
  opt_prob.Qq = 0.1 * Matrix2d::Identity();
  opt_prob.Qv = 0.2 * Matrix2d::Identity();
  opt_prob.Qf_q = 0.3 * Matrix2d::Identity();
  opt_prob.Qf_v = 0.4 * Matrix2d::Identity();
  opt_prob.R = 0.5 * Matrix2d::Identity();
  opt_prob.q_nom = Vector2d(1.2, 1.1);
  opt_prob.v_nom = Vector2d(-1.1, 1.0);

  // Make some fake data
  std::vector<VectorXd> q;
  std::vector<VectorXd> v;
  std::vector<VectorXd> tau;
  for (int t = 0; t < opt_prob.num_steps; ++t) {
    q.push_back(Vector2d(0.2, 0.1));
    v.push_back(Vector2d(-0.1, 0.0));
    tau.push_back(Vector2d(-1.0, 1.0));
  }
  q.push_back(Vector2d(0.2, 0.1));
  v.push_back(Vector2d(-0.1, 0.0));

  // Compute the cost and compare with the true value
  TrajectoryOptimizer<double> optimizer(&plant, opt_prob);
  TrajectoryOptimizerWorkspace<double> workspace(num_steps, plant);
  double L =
      TrajectoryOptimizerTester::CalcCost(optimizer, q, v, tau, &workspace);
  double L_gt =
      num_steps * dt * (2 * 0.1 + 2 * 0.2 + 2 * 0.5) + 2 * 0.3 + 2 * 0.4;

  const double kTolerance = std::numeric_limits<double>::epsilon() / dt;
  EXPECT_NEAR(L, L_gt, kTolerance);
}

/**
 * Test our computation of generalized forces
 *
 *   tau_t = InverseDynamics(a_t, v_t, q_t)
 *
 * where a_t = (v_{t+1}-v_t)/dt.
 *
 */
GTEST_TEST(TrajectoryOptimizerTest, PendulumCalcInverseDynamics) {
  const int num_steps = 5;
  const double dt = 1e-2;

  // Set up the system model
  MultibodyPlant<double> plant(dt);
  const std::string urdf_file =
      FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  Parser(&plant).AddAllModelsFromFile(urdf_file);
  plant.set_discrete_contact_solver(DiscreteContactSolver::kSap);
  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();

  // Make some fake data
  std::vector<VectorXd> q;
  std::vector<VectorXd> v;
  for (int t = 0; t <= num_steps; ++t) {
    // Not physically valid, but should be fine for this test
    q.push_back(Vector1d(-0.2 + dt * 0.1 * t));
    v.push_back(Vector1d(0.1 + dt * 0.01 * t));
  }

  // Compute ground truth torque analytically using the
  // pendulum model
  //
  //     m*l^2*a + m*g*l*sin(q) + b*v = tau
  //
  // where q is the joint angle and a = dv/dt, v = dq/dt.
  const double m = 1.0;
  const double l = 0.5;
  const double b = 0.1;
  const double g = 9.81;

  std::vector<VectorXd> tau_gt;
  for (int t = 0; t < num_steps; ++t) {
    Vector1d a = (v[t + 1] - v[t]) / dt;
    Vector1d sin_q = sin(q[t + 1](0)) * MatrixXd::Identity(1, 1);
    Vector1d tau_t = m * l * l * a + m * g * l * sin_q + b * v[t + 1];
    tau_gt.push_back(tau_t);
  }

  // Create a trajectory optimizer object
  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  TrajectoryOptimizer<double> optimizer(&plant, opt_prob);
  TrajectoryOptimizerWorkspace<double> workspace(num_steps, plant);

  // Compute tau from q and v
  std::vector<VectorXd> tau(num_steps, VectorXd(1));
  std::vector<VectorXd> a(num_steps, VectorXd(1));
  {
    // It appears, via trial and error, that CalcInverseDynamics makes exactly
    // 15 allocations for this example.
    LimitMalloc guard({.max_num_allocations = 15});
    TrajectoryOptimizerTester::CalcAccelerations(optimizer, v, &a);
    TrajectoryOptimizerTester::CalcInverseDynamics(optimizer, q, v, a,
                                                   &workspace, &tau);
  }

  // Check that our computed values match the true (recorded) ones
  const double kToleranceTau = std::numeric_limits<double>::epsilon();
  for (int t = 0; t < num_steps; ++t) {
    EXPECT_TRUE(CompareMatrices(tau[t], tau_gt[t], kToleranceTau,
                                MatrixCompareType::relative));
  }
}

/**
 * Test our computation of generalized velocities
 *
 *   v_t = (q_t - q_{t-1})/dt
 *
 */
GTEST_TEST(TrajectoryOptimizerTest, CalcVelocities) {
  const int num_steps = 5;
  const double dt = 1e-2;

  // Create a TrajectoryOptimizer object
  MultibodyPlant<double> plant(dt);
  plant.Finalize();
  ProblemDefinition opt_prob;
  opt_prob.q_init = Vector2d(0.1, 0.2);
  opt_prob.v_init = Vector2d(0.5 / dt, 1.5 / dt);
  opt_prob.num_steps = num_steps;
  TrajectoryOptimizer<double> optimizer(&plant, opt_prob);

  // Construct a std::vector of generalized positions (q)
  // where q(t) = [0.1 + 0.5*t]
  //              [0.2 + 1.5*t]
  std::vector<VectorXd> q;
  for (int t = 0; t <= num_steps; ++t) {
    q.push_back(Vector2d(0.1 + 0.5 * t, 0.2 + 1.5 * t));
  }

  // Compute v from q
  std::vector<VectorXd> v(num_steps + 1);
  TrajectoryOptimizerTester::CalcVelocities(optimizer, q, &v);

  // Check that our computed v is correct
  const double kTolerance = std::numeric_limits<double>::epsilon() / dt;
  for (int t = 0; t <= num_steps; ++t) {
    EXPECT_TRUE(CompareMatrices(v[t], opt_prob.v_init, kTolerance,
                                MatrixCompareType::relative));
  }
}

}  // namespace internal
}  // namespace traj_opt
}  // namespace drake
