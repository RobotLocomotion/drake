#include "drake/systems/trajectory_optimization/direct_collocation.h"

#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/systems/framework/vector_system.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/trajectory_optimization/direct_trajectory_optimization.h"

namespace drake {
namespace systems {
namespace {

// Implements the double integrator: qddot = u.
template <typename T>
class DoubleIntegrator : public VectorSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DoubleIntegrator);

  DoubleIntegrator()
      : VectorSystem<T>(SystemTypeTag<systems::DoubleIntegrator>{}, 1, 1) {
    this->DeclareContinuousState(1, 1, 0);
  }

  template <typename U>
  explicit DoubleIntegrator(const DoubleIntegrator<U>&)
      : DoubleIntegrator<T>() {}

  ~DoubleIntegrator() override {}

 protected:
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

// Tests the double integrator minimum-time problem with the known solution.
GTEST_TEST(TrajectoryOptimizationTest, DoubleIntegratorTest) {
  DoubleIntegrator<double> double_integrator;
  auto context = double_integrator.CreateDefaultContext();
  const int timesteps{10};
  DircolTrajectoryOptimization prog(&double_integrator, *context, timesteps,
                                    0.5, 20.0);

  // u \in [-1,1].
  prog.AddConstraintToAllKnotPoints(Vector1d(-1.0) <= prog.input());
  prog.AddConstraintToAllKnotPoints(prog.input() <= Vector1d(1.0));

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

// A simple example where the plant has no inputs.
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
