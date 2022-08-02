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

  AutoDiffXd cost_ad = optimizer_ad.CalcCost(state_ad);
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
 * Quick sanity check on our computation of costs using the state abstraction.
 */
GTEST_TEST(TrajectoryOptimizerTest, CalcCostFromState) {
  const int num_steps = 10;
  const double dt = 1e-3;

  // Set up a system model
  MultibodyPlant<double> plant(dt);
  const std::string urdf_file =
      FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  Parser(&plant).AddAllModelsFromFile(urdf_file);
  plant.Finalize();

  // Set up a toy optimization problem
  ProblemDefinition opt_prob;
  opt_prob.num_steps = num_steps;
  opt_prob.q_init = Vector1d(0.0);
  opt_prob.v_init = Vector1d(-0.1);
  opt_prob.Qq = 0.0 * MatrixXd::Identity(1, 1);
  opt_prob.Qv = 1.0 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_q = 0.0 * MatrixXd::Identity(1, 1);
  opt_prob.Qf_v = 1.0 * MatrixXd::Identity(1, 1);
  opt_prob.R = 0.0 * MatrixXd::Identity(1, 1);
  opt_prob.q_nom = Vector1d(0.1);
  opt_prob.v_nom = Vector1d(0.9);

  // Create some fake data
  std::vector<VectorXd> q;
  q.push_back(opt_prob.q_init);
  for (int t = 1; t <= num_steps; ++t) {
    q.push_back(Vector1d(0.0 - 0.1 * dt * t));
  }

  // Create an optimizer
  TrajectoryOptimizer<double> optimizer(&plant, opt_prob);
  TrajectoryOptimizerState<double> state = optimizer.CreateState();
  state.set_q(q);

  // Evaluate the cost
  double L = optimizer.CalcCost(state);
  double L_gt = num_steps * dt + 1;

  const double kTolerance = std::numeric_limits<double>::epsilon() / dt;
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
