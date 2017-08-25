#include "drake/systems/trajectory_optimization/direct_transcription.h"

#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"
#include "drake/math/autodiff.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
namespace {

using symbolic::Variable;
using symbolic::Expression;

GTEST_TEST(DirectTranscriptionTest, DiscreteTimeConstructorThrows) {
  // Construct a trivial continuous time system.
  const Eigen::Matrix2d A = Eigen::Matrix2d::Identity();
  const Eigen::Matrix<double, 2, 0> B;
  const Eigen::Matrix<double, 0, 2> C;
  const Eigen::Matrix<double, 0, 0> D;
  LinearSystem<double> system(A, B, C, D);

  const auto context = system.CreateDefaultContext();
  EXPECT_THROW(DirectTranscription(&system, *context, 3), std::runtime_error);
}

namespace {

template <typename T>
class CubicPolynomialSystem final : public systems::LeafSystem<T> {
 public:
  explicit CubicPolynomialSystem(double timestep)
      : systems::LeafSystem<T>(
            systems::SystemTypeTag<
                trajectory_optimization::CubicPolynomialSystem>{}),
        timestep_(timestep) {       // Zero inputs, zero outputs.
    this->DeclareDiscreteState(1);  // One state variable.
    this->DeclarePeriodicDiscreteUpdate(timestep);
  }

  // Scalar-converting copy constructor.
  template <typename U>
  explicit CubicPolynomialSystem(const CubicPolynomialSystem<U>& system)
      : CubicPolynomialSystem(system.timestep()) {}

  double timestep() const { return timestep_; }

 private:
  // x[n+1] = x³[n]
  void DoCalcDiscreteVariableUpdates(
      const Context<T>& context,
      const std::vector<const DiscreteUpdateEvent<T>*>&,
      DiscreteValues<T>* discrete_state) const final {
    using std::pow;
    discrete_state->get_mutable_vector(0)->SetAtIndex(
        0, pow(context.get_discrete_state(0)->get_value()[0], 3.0));
  }

  const double timestep_{0.0};
};

}  // namespace

// This example will NOT use the symbolic constraints.
GTEST_TEST(DirectTranscriptionTest, DiscreteTimeConstraintTest) {
  const double kTimeStep = 1.0;
  CubicPolynomialSystem<double> system(kTimeStep);

  const auto context = system.CreateDefaultContext();
  int kNumSampleTimes = 3;
  DirectTranscription prog(&system, *context, kNumSampleTimes);

  // TODO(russt):  Uncomment this upon resolution of #6878.
  // EXPECT_EQ(prog.fixed_timestep(),kTimeStep);

  // Sets all decision variables to trivial known values (1,2,3,...).
  prog.SetDecisionVariableValues(
      Eigen::VectorXd::LinSpaced(prog.num_vars(), 1, prog.num_vars()));

  // Constructor should add dynamic constraints, and these are the
  // only generic constraints that should be in the program so far.
  const std::vector<solvers::Binding<solvers::Constraint>>&
      dynamic_constraints = prog.generic_constraints();
  EXPECT_EQ(dynamic_constraints.size(), kNumSampleTimes - 1);

  using std::pow;
  for (int i = 0; i < (kNumSampleTimes - 1); i++) {
    EXPECT_EQ(prog.EvalBindingAtSolution(dynamic_constraints[i])[0],
              prog.GetSolution(prog.state(i + 1)[0]) -
                  pow(prog.GetSolution(prog.state(i)[0]), 3.0));
  }
}

// This example WILL use the symbolic constraints.
GTEST_TEST(DirectTranscriptionTest, DiscreteTimeSymbolicConstraintTest) {
  Eigen::Matrix2d A, B;
  // clang-format off
  A << 1, 2,
      3, 4;
  B << 5, 6,
      7, 8;
  // clang-format on
  const Eigen::MatrixXd C(0, 2), D(0, 2);
  const double kTimeStep = .1;
  LinearSystem<double> system(A, B, C, D, kTimeStep);

  const auto context = system.CreateDefaultContext();
  int kNumSampleTimes = 3;
  DirectTranscription prog(&system, *context, kNumSampleTimes);

  // TODO(russt):  Uncomment this upon resolution of #6878.
  // EXPECT_EQ(prog.fixed_timestep(),kTimeStep);

  // Sets all decision variables to trivial known values (1,2,3,...).
  prog.SetDecisionVariableValues(
      Eigen::VectorXd::LinSpaced(prog.num_vars(), 1, prog.num_vars()));

  // Constructor should add dynamic constraints, and these should have been
  // added as linear equality constraints. (There is also one linear
  // equality constraint for the control input).
  const std::vector<solvers::Binding<solvers::LinearEqualityConstraint>>&
      dynamic_constraints = prog.linear_equality_constraints();
  EXPECT_EQ(dynamic_constraints.size(), kNumSampleTimes);
  // Check that there are no nonlinear constraints in the program.
  EXPECT_EQ(prog.generic_constraints().size(), 0);

  for (int i = 0; i < (kNumSampleTimes - 1); i++) {
    EXPECT_TRUE(
        CompareMatrices(prog.EvalBindingAtSolution(dynamic_constraints[i]),
                        prog.GetSolution(prog.state(i + 1)) -
                            A * prog.GetSolution(prog.state(i)) -
                            B * prog.GetSolution(prog.input(i))));
  }
}

GTEST_TEST(DirectTranscriptionTest, AddRunningCostTest) {
  // x[n+1] = 1.
  const Eigen::Matrix<double, 1, 1> A(0.0);
  const Eigen::Matrix<double, 1, 0> B;
  const Vector1d f0(1.0);
  const Eigen::Matrix<double, 0, 1> C;
  const Eigen::Matrix<double, 0, 0> D;
  const Eigen::Matrix<double, 0, 1> y0;
  const double kTimeStep = 0.1;
  AffineSystem<double> system(A, B, f0, C, D, y0, kTimeStep);

  const auto context = system.CreateDefaultContext();
  const int kNumSamples{5};

  DirectTranscription prog(&system, *context, kNumSamples);

  // Check that there are no nonlinear constraints in the program.
  EXPECT_EQ(prog.generic_constraints().size(), 0);

  // x[0] = 0.
  prog.AddLinearConstraint(prog.initial_state() == Vector1d(1.0));

  prog.AddRunningCost(prog.state());
  prog.AddFinalCost(prog.state().cast<symbolic::Expression>());

  EXPECT_EQ(prog.Solve(), solvers::SolutionResult::kSolutionFound);

  // Cost is x[N] + \sum_{0...N-1} h*x[i]
  EXPECT_NEAR(prog.GetOptimalCost(), kTimeStep * (kNumSamples - 1) + 1, 1e-6);
}

// TODO(russt): Add tests for ReconstructTrajectory methods once their output is
// non-trivial.

}  // anonymous namespace
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
