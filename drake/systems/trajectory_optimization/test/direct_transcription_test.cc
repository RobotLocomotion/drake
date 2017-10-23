#include "drake/systems/trajectory_optimization/direct_transcription.h"

#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/piecewise_polynomial_linear_system.h"

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
        timestep_(timestep) {
    // Zero inputs, zero outputs.
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
    discrete_state->get_mutable_vector(0).SetAtIndex(
        0, pow(context.get_discrete_state(0).GetAtIndex(0), 3.0));
  }

  const double timestep_{0.0};
};

template <typename T>
class LinearSystemWParams final : public systems::LeafSystem<T> {
 public:
  LinearSystemWParams()
      : systems::LeafSystem<T>(
            systems::SystemTypeTag<
                trajectory_optimization::LinearSystemWParams>{}) {
    // Zero inputs, zero outputs.
    this->DeclareDiscreteState(1);                     // One state variable.
    this->DeclareNumericParameter(BasicVector<T>(1));  // One parameter.
    this->DeclarePeriodicDiscreteUpdate(1.0);
  }

  // Scalar-converting copy constructor.
  template <typename U>
  explicit LinearSystemWParams(const LinearSystemWParams<U>& system)
      : LinearSystemWParams() {}

 private:
  // x[n+1] = p0 * x[n]
  void DoCalcDiscreteVariableUpdates(
      const Context<T>& context,
      const std::vector<const DiscreteUpdateEvent<T>*>&,
      DiscreteValues<T>* discrete_state) const final {
    discrete_state->get_mutable_vector(0).SetAtIndex(
        0, context.get_numeric_parameter(0).GetAtIndex(0) *
               context.get_discrete_state(0).GetAtIndex(0));
  }
};

std::unique_ptr<AffineSystem<double>> MakeAffineSystem(const double time_step) {
  // x[n+1] = x[n] + 2 * u[n] + 3,
  // y[n] = 4 * x[n] + 5 * u[n] + 6.
  const Eigen::Matrix<double, 1, 1> A(1.0);
  const Eigen::Matrix<double, 1, 1> B(2.0);
  const Vector1d f0(3.0);
  const Eigen::Matrix<double, 1, 1> C(4.0);
  const Eigen::Matrix<double, 1, 1> D(5.0);
  const Eigen::Matrix<double, 1, 1> y0(6.0);
  return std::make_unique<AffineSystem<double>>(A, B, f0, C, D, y0, time_step);
}

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
  const double kTimeStep = 0.1;
  std::unique_ptr<AffineSystem<double>> system = MakeAffineSystem(kTimeStep);

  const auto context = system->CreateDefaultContext();
  int kNumSampleTimes = 3;
  DirectTranscription prog(system.get(), *context, kNumSampleTimes);

  // TODO(russt):  Uncomment this upon resolution of #6878.
  // EXPECT_EQ(prog.fixed_timestep(),kTimeStep);

  // Sets all decision variables to trivial known values (1,2,3,...).
  prog.SetDecisionVariableValues(
      Eigen::VectorXd::LinSpaced(prog.num_vars(), 1, prog.num_vars()));

  // Constructor should add dynamic constraints, and these should have been
  // added as linear equality constraints.
  const std::vector<solvers::Binding<solvers::LinearEqualityConstraint>>&
      dynamic_constraints = prog.linear_equality_constraints();
  EXPECT_EQ(dynamic_constraints.size(), kNumSampleTimes);
  // Check that there are no nonlinear constraints in the program.
  EXPECT_EQ(prog.generic_constraints().size(), 0);

  for (int i = 0; i < (kNumSampleTimes - 1); i++) {
    const Vector1d dynamic_constraint_val =
        prog.EvalBindingAtSolution(dynamic_constraints[i]) -
        dynamic_constraints[i].constraint()->lower_bound();
    const Vector1d dynamic_constraint_expected =
        prog.GetSolution(prog.state(i + 1)) -
        system->A() * prog.GetSolution(prog.state(i)) -
        system->B() * prog.GetSolution(prog.input(i)) -
        system->f0();

    // Check that the system's state equation still holds with equality,
    // regardless of the specific encoding MathematicalProgram chooses.
    EXPECT_TRUE(
        CompareMatrices(dynamic_constraint_val, dynamic_constraint_expected) ||
        CompareMatrices(dynamic_constraint_val, -dynamic_constraint_expected));
  }
}

// This example tests the LinearSystem specialization of the constructor.
GTEST_TEST(DirectTranscriptionTest, DiscreteTimeLinearSystemTest) {
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

  EXPECT_EQ(prog.fixed_timestep(), kTimeStep);

  // Sets all decision variables to trivial known values (1,2,3,...).
  prog.SetDecisionVariableValues(
      Eigen::VectorXd::LinSpaced(prog.num_vars(), 1, prog.num_vars()));

  // Constructor should add dynamic constraints, and these should have been
  // added as linear equality constraints.
  const std::vector<solvers::Binding<solvers::LinearEqualityConstraint>>&
      dynamic_constraints = prog.linear_equality_constraints();
  EXPECT_EQ(dynamic_constraints.size(), kNumSampleTimes);
  // Check that there are no nonlinear constraints in the program.
  EXPECT_EQ(prog.generic_constraints().size(), 0);

  const double kNumericalTolerance = 1e-10;
  for (int i = 0; i < (kNumSampleTimes - 1); i++) {
    EXPECT_TRUE(
        CompareMatrices(prog.EvalBindingAtSolution(dynamic_constraints[i]),
                        prog.GetSolution(prog.state(i + 1)) -
                          A * prog.GetSolution(prog.state(i)) -
                          B * prog.GetSolution(prog.input(i)),
                        kNumericalTolerance));
  }
}

// This example tests the TimeVaryingLinearSystem specialization of the
// constructor.
GTEST_TEST(DirectTranscriptionTest, TimeVaryingLinearSystemTest) {
  const std::vector<double> times {0., 1.};
  std::vector<Eigen::MatrixXd> Avec(times.size());
  std::vector<Eigen::MatrixXd> Bvec(times.size());
  std::vector<Eigen::MatrixXd> Cvec(times.size());
  std::vector<Eigen::MatrixXd> Dvec(times.size());
  Eigen::Matrix2d A0;
  A0 << 1, 2, 3, 4;
  Eigen::Matrix2d B0;
  B0 << 5, 6, 7, 8;
  Eigen::Matrix2d C0;
  C0 << 9, 10, 11, 12;
  Eigen::Matrix2d D0;
  D0 << 13, 14, 15, 16;
  for (int i{0}; i < static_cast<int>(times.size()); ++i) {
    Avec[i] = A0 + i * Eigen::Matrix2d::Ones();
    Bvec[i] = B0 + i * Eigen::Matrix2d::Ones();
    Cvec[i] = C0 + i * Eigen::Matrix2d::Ones();
    Dvec[i] = D0 + i * Eigen::Matrix2d::Ones();
  }
  const auto A = PiecewisePolynomial<double>::FirstOrderHold(times, Avec);
  const auto B = PiecewisePolynomial<double>::FirstOrderHold(times, Bvec);
  const auto C = PiecewisePolynomial<double>::FirstOrderHold(times, Cvec);
  const auto D = PiecewisePolynomial<double>::FirstOrderHold(times, Dvec);

  const double kTimeStep = .1;
  PiecewisePolynomialLinearSystem<double> system({A, B, C, D}, kTimeStep);

  const auto context = system.CreateDefaultContext();
  int kNumSampleTimes = 3;
  DirectTranscription prog(&system, *context, kNumSampleTimes);

  EXPECT_EQ(prog.fixed_timestep(), kTimeStep);

  // Sets all decision variables to trivial known values (1,2,3,...).
  prog.SetDecisionVariableValues(
      Eigen::VectorXd::LinSpaced(prog.num_vars(), 1, prog.num_vars()));

  // Constructor should add dynamic constraints, and these should have been
  // added as linear equality constraints.
  const std::vector<solvers::Binding<solvers::LinearEqualityConstraint>>&
      dynamic_constraints = prog.linear_equality_constraints();
  EXPECT_EQ(dynamic_constraints.size(), kNumSampleTimes);
  // Check that there are no nonlinear constraints in the program.
  EXPECT_EQ(prog.generic_constraints().size(), 0);

  const double kNumericalTolerance = 1e-10;
  for (int i = 0; i < (kNumSampleTimes - 1); i++) {
    const double t = system.time_period() * i;
    EXPECT_TRUE(
        CompareMatrices(prog.EvalBindingAtSolution(dynamic_constraints[i]),
                        prog.GetSolution(prog.state(i + 1)) -
                          A.value(t) * prog.GetSolution(prog.state(i)) -
                          B.value(t) * prog.GetSolution(prog.input(i)),
                        kNumericalTolerance));
  }
}

GTEST_TEST(DirectTranscriptionTest, AddRunningCostTest) {
  const double kTimeStep = 0.1;
  std::unique_ptr<AffineSystem<double>> system = MakeAffineSystem(kTimeStep);

  const auto context = system->CreateDefaultContext();
  const int kNumSamples{5};

  DirectTranscription prog(system.get(), *context, kNumSamples);

  // Check that there are no nonlinear constraints in the program.
  EXPECT_EQ(prog.generic_constraints().size(), 0);

  // x[0] = 1.0
  prog.AddLinearConstraint(prog.initial_state() == Vector1d(1.0));

  prog.AddRunningCost(prog.state() * prog.state());
  prog.AddFinalCost(prog.state() * prog.state());

  EXPECT_EQ(prog.Solve(), solvers::SolutionResult::kSolutionFound);

  // Compute the expected cost as c[N] + \Sum_{i = 0...N-1} h * c[i]
  //   where c[i] is the running cost and c[N] is the terminal cost.
  double expected_cost{0.};
  for (int i{0}; i < kNumSamples - 1; i++) {
    expected_cost +=
        kTimeStep * std::pow(prog.GetSolution(prog.state(i))[0], 2.);
  }

  EXPECT_NEAR(prog.GetOptimalCost(), expected_cost, 1e-6);
}

// Check symbolic dynamics with parameters.
GTEST_TEST(DirectTranscriptionTest, LinearSystemWParamsTest) {
  LinearSystemWParams<double> system;

  const auto context = system.CreateDefaultContext();
  const double kGain = -1.0;
  context->get_mutable_numeric_parameter(0).SetAtIndex(0, kGain);
  const int kNumSampleTimes = 3;
  DirectTranscription prog(&system, *context, kNumSampleTimes);

  // Sets all decision variables to trivial known values (1,2,3,...).
  prog.SetDecisionVariableValues(
      Eigen::VectorXd::LinSpaced(prog.num_vars(), 1, prog.num_vars()));

  // Constructor should add dynamic constraints, and these are the
  // only generic constraints that should be in the program so far.
  const std::vector<solvers::Binding<solvers::LinearEqualityConstraint>>&
      dynamic_constraints = prog.linear_equality_constraints();
  EXPECT_EQ(dynamic_constraints.size(), kNumSampleTimes - 1);

  for (int i = 0; i < (kNumSampleTimes - 1); i++) {
    // Checks that x[n+1] = kGain*x[n].
    EXPECT_EQ(prog.EvalBindingAtSolution(dynamic_constraints[i])[0],
              prog.GetSolution(prog.state(i + 1)[0]) -
                  kGain * prog.GetSolution(prog.state(i)[0]));
  }
}

// TODO(russt): Add tests for ReconstructTrajectory methods once their output is
// non-trivial.

}  // anonymous namespace
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
