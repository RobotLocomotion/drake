#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/framework/siso_vector_system.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/trajectory_optimization/direct_collocation.h"
#include "drake/systems/trajectory_optimization/direct_trajectory_optimization.h"

using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;

using drake::solvers::detail::VecIn;
using drake::solvers::detail::VecOut;

namespace drake {
namespace systems {
namespace {

typedef PiecewisePolynomial<double> PiecewisePolynomialType;

class FinalCost {
 public:
  static size_t numInputs() { return 3; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
    y(0) = x(2) * x(2);
  }
};

class MyDirectTrajOpt : public DirectTrajectoryOptimization {
 public:
  MyDirectTrajOpt(const int num_inputs, const int num_states,
                  const int num_time_samples,
                  const double traj_time_lower_bound,
                  const double traj_time_upper_bound)
      : DirectTrajectoryOptimization(num_inputs, num_states, num_time_samples,
                                     traj_time_lower_bound,
                                     traj_time_upper_bound) {}

 private:
  void DoAddRunningCost(const symbolic::Expression& g) override {}
  void DoAddRunningCost(std::shared_ptr<solvers::Cost> constraint) override {}
};

GTEST_TEST(TrajectoryOptimizationTest, DirectTrajectoryOptimizationTest) {
  const int kNumInputs(1);
  const int kNumStates(2);
  const int kNumTimeSamples(21);  // aka N.
  MyDirectTrajOpt direct_traj(kNumInputs, kNumStates, kNumTimeSamples, 0, 25);

  // Add bounds on time intervals.
  direct_traj.AddTimeIntervalBounds(
      Eigen::Matrix<double, kNumTimeSamples - 1, 1>::Constant(0.01),
      Eigen::Matrix<double, kNumTimeSamples - 1, 1>::Constant(10));
  std::vector<int> interval_indices{1, 2, 3};
  direct_traj.AddTimeIntervalBounds(Eigen::Vector3d(0.2, 0.3, 0.4),
                                    Eigen::Vector3d(5, 5, 5), interval_indices);
  Eigen::Matrix<double, kNumTimeSamples - 1, 1> h_lb =
      Eigen::Matrix<double, kNumTimeSamples - 1, 1>::Constant(0.01);
  Eigen::Matrix<double, kNumTimeSamples - 1, 1> h_ub =
      Eigen::Matrix<double, kNumTimeSamples - 1, 1>::Constant(10);
  h_lb(1) = 0.2;
  h_lb(2) = 0.3;
  h_lb(3) = 0.4;
  h_ub(1) = 5;
  h_ub(2) = 5;
  h_ub(3) = 5;

  const double t_init_in(7);
  const Polynomiald y = Polynomiald("y");
  const Polynomiald y1 = (7 * y);
  const Polynomiald y2 = (2 * y) + 1;

  const std::vector<Polynomiald> u_vec{y1};
  std::vector<PiecewisePolynomialType::PolynomialMatrix> y_vec(1);
  y_vec[0].resize(2, 1);
  y_vec[0](0) = y1;
  y_vec[0](1) = y2;
  const std::vector<double> times{0.0, t_init_in};

  const PiecewisePolynomialType inputs_u(u_vec, times);
  const PiecewisePolynomialType states_x(y_vec, times);

  const int kInputConstraintLo = 1;
  const int kInputConstraintHi = kNumTimeSamples - 2;
  const Vector1d constrained_input(30);
  auto input_constraint = std::make_shared<solvers::LinearEqualityConstraint>(
      Vector1d(1), constrained_input);
  direct_traj.AddInputConstraint(input_constraint,
                                 {kInputConstraintLo, kInputConstraintHi});

  const int kStateConstraintLo = 2;
  const int kStateConstraintHi = kNumTimeSamples - 3;
  const Eigen::Vector2d constrained_state(11, 22);
  auto state_constraint = std::make_shared<solvers::LinearEqualityConstraint>(
      Eigen::Matrix2d::Identity(), constrained_state);
  direct_traj.AddStateConstraint(state_constraint,
                                 {kStateConstraintLo, kStateConstraintHi});

  solvers::SolutionResult result = solvers::SolutionResult::kUnknownError;
  result =
      direct_traj.SolveTraj(t_init_in, PiecewisePolynomialType(), states_x);
  EXPECT_EQ(result, solvers::SolutionResult::kSolutionFound)
      << "Result is an Error";

  Eigen::MatrixXd inputs;
  Eigen::MatrixXd states;
  std::vector<double> times_out;

  direct_traj.GetResultSamples(&inputs, &states, &times_out);
  for (int i = 1; i < kNumTimeSamples; ++i) {
    EXPECT_GE(times_out[i] - times_out[i - 1], h_lb(i - 1) - 1E-10);
    EXPECT_LE(times_out[i] - times_out[i - 1], h_ub(i - 1) + 1E-10);
  }

  PiecewisePolynomialTrajectory input_traj =
      direct_traj.ReconstructInputTrajectory();

  EXPECT_TRUE(CompareMatrices(constrained_input, inputs.col(kInputConstraintLo),
                              1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(constrained_input,
                              input_traj.value(times_out[kInputConstraintLo]),
                              1e-6, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(constrained_input, inputs.col(kInputConstraintHi),
                              1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(constrained_input,
                              input_traj.value(times_out[kInputConstraintHi]),
                              1e-6, MatrixCompareType::absolute));

  PiecewisePolynomialTrajectory state_traj =
      direct_traj.ReconstructStateTrajectory();
  EXPECT_TRUE(CompareMatrices(constrained_state, states.col(kStateConstraintLo),
                              1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(constrained_state,
                              state_traj.value(times_out[kStateConstraintLo]),
                              1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(constrained_state, states.col(kStateConstraintHi),
                              1e-10, MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(constrained_state,
                              state_traj.value(times_out[kStateConstraintHi]),
                              1e-10, MatrixCompareType::absolute));

  // Add bounds on the inputs and make sure they're enforced.
  Vector1d input_min(1);
  direct_traj.AddInputBounds(input_min, constrained_input);
  result =
      direct_traj.SolveTraj(t_init_in, PiecewisePolynomialType(), states_x);
  EXPECT_EQ(result, solvers::SolutionResult::kSolutionFound)
      << "Result is an Error";
  direct_traj.GetResultSamples(&inputs, &states, &times_out);

  EXPECT_GE(inputs(0, 0), input_min(0));

  result = direct_traj.SolveTraj(t_init_in, inputs_u, states_x);
  EXPECT_EQ(result, solvers::SolutionResult::kSolutionFound)
      << "Result is an Error";

  // Add some cost functions and see that something gets minimized.
  // First check that we have values not particularly near zero where
  // we're going to try to minimize next time.
  direct_traj.GetResultSamples(&inputs, &states, &times_out);

  // Tests adding constrainst to the original mathematical program, via
  // the state variable accessor and symbolic formula.
  direct_traj.AddLinearConstraint(direct_traj.initial_state().array() == 0.0);

  // Adds a final cost
  direct_traj.AddFinalCostFunc(FinalCost());
  result = direct_traj.SolveTraj(t_init_in, inputs_u, states_x);
  EXPECT_EQ(result, solvers::SolutionResult::kSolutionFound)
      << "Result is an Error";

  direct_traj.GetResultSamples(&inputs, &states, &times_out);
  EXPECT_NEAR(states(1, 0), 0, 1e-10);
  EXPECT_NEAR(states(1, kNumTimeSamples - 1), 0, 1e-10);
}

GTEST_TEST(TrajectoryOptimizationTest, PlaceholderVariableTest) {
  const int kNumInputs(1);
  const int kNumStates(2);
  const int kNumTimeSamples(21);  // aka N.
  MyDirectTrajOpt prog(kNumInputs, kNumStates, kNumTimeSamples, 0, 25);

  // Adding a placeholder variable directly to the program should fail.
  // (but currently does NOT; see #5623)
  auto u = prog.input();
  prog.AddCost(u(0));  // TODO(russt): EXPECT_THROW
}

// qddot = u.
template <typename T>
class DoubleIntegrator : public SisoVectorSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DoubleIntegrator);

  DoubleIntegrator() : SisoVectorSystem<T>(1, 1) {
    this->DeclareContinuousState(1, 1, 0);
  }
  ~DoubleIntegrator() override{};

 protected:
  DoubleIntegrator<AutoDiffXd>* DoToAutoDiffXd() const override {
    return new DoubleIntegrator<AutoDiffXd>();
  }

  void DoCalcVectorOutput(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* output) const override {
    *output << state(0);
  }

  void DoCalcVectorTimeDerivatives(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* derivatives) const override {
    *derivatives << state(1), input(0);
  }
};

// Tests the double integrator minimum-time problem known solution.
GTEST_TEST(TrajectoryOptimizationTest, DoubleIntegratorTest) {
  DoubleIntegrator<double> double_integrator;
  auto context = double_integrator.CreateDefaultContext();
  const int timesteps{10};
  DircolTrajectoryOptimization prog(&double_integrator, *context, timesteps,
                                    0.5, 20.0);

  // u \in [-1,1].
  prog.AddInputBounds(Vector1d(-1.0), Vector1d(1.0));
  // xf = [0,0].
  prog.AddLinearConstraint(prog.final_state().array() == 0.0);

  // x0 = [-1,0].
  prog.AddLinearConstraint(prog.initial_state() == Eigen::Vector2d(-1.0, 0.0));

  // Cost is just total time.
  prog.AddFinalCost(prog.time().cast<symbolic::Expression>());

  EXPECT_EQ(prog.Solve(), solvers::SolutionResult::kSolutionFound);

  // Solution should be bang-band (u = +1 then -1).
  int i = 0;
  while (i < timesteps / 2.0)
    EXPECT_NEAR(prog.GetSolution(prog.input(i++))(0), 1.0, 1e-5);
  while (i < timesteps)
    EXPECT_NEAR(prog.GetSolution(prog.input(i++))(0), -1.0, 1e-5);
}

// Tests the double integrator without input limits results in minimal time.
GTEST_TEST(TrajectoryOptimizationTest, MinimumTimeTest) {
  DoubleIntegrator<double> double_integrator;
  auto context = double_integrator.CreateDefaultContext();
  const int timesteps{10};
  const double min_time{0.5};
  DircolTrajectoryOptimization prog(&double_integrator, *context, timesteps,
                                    min_time, 20.0);

  // Note: No input limits this time.

  // xf = [0,0].
  prog.AddLinearConstraint(prog.final_state().array() == 0.0);

  // x0 = [-1,0].
  prog.AddLinearConstraint(prog.initial_state() == Eigen::Vector2d(-1.0, 0.0));

  // Cost is just total time.
  prog.AddFinalCost(prog.time().cast<symbolic::Expression>());

  EXPECT_EQ(prog.Solve(), solvers::SolutionResult::kSolutionFound);

  // Solution should have total time equal to 0.5.
  double total_time = 0;
  for (int i = 0; i < timesteps - 1; i++)
    total_time += prog.GetSolution(prog.timestep(i))(0);
  EXPECT_NEAR(total_time, min_time, 1e-5);
}

/// A simple example where the plant has no inputs.
GTEST_TEST(TrajectoryOptimizationTest, NoInputs) {
  // xdot = -x.
  systems::LinearSystem<double> plant(
      Vector1d(-1.0),                        // A
      Eigen::Matrix<double, 1, 0>::Zero(),   // B
      Eigen::Matrix<double, 0, 1>::Zero(),   // C
      Eigen::Matrix<double, 0, 0>::Zero());  // D

  auto context = plant.CreateDefaultContext();
  const int timesteps{10};
  const double duration{1.0};
  DircolTrajectoryOptimization prog(&plant, *context, timesteps, duration,
                                    duration);

  prog.AddTimeIntervalBounds(duration / (timesteps - 1),
                             duration / (timesteps - 1));

  const double x0 = 2.0;
  prog.AddLinearConstraint(prog.initial_state() == Vector1d(x0));

  EXPECT_EQ(prog.Solve(), solvers::SolutionResult::kSolutionFound);

  EXPECT_NEAR(prog.GetSolution(prog.final_state())(0), x0 * std::exp(-duration),
              1e-6);
}

}  // anonymous namespace
}  // namespace systems
}  // namespace drake
