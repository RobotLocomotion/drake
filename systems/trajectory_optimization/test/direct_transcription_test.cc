#include "drake/systems/trajectory_optimization/direct_transcription.h"

#include <cmath>
#include <cstddef>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/symbolic_vector_system.h"
#include "drake/systems/primitives/trajectory_linear_system.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
namespace {

using symbolic::Variable;
using symbolic::Expression;
using trajectories::PiecewisePolynomial;

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
    (*discrete_state)[0] =
        pow(context.get_discrete_state(0).GetAtIndex(0), 3.0);
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
    (*discrete_state)[0] = context.get_numeric_parameter(0).GetAtIndex(0) *
                           context.get_discrete_state(0).GetAtIndex(0);
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

// This example will NOT use the symbolic constraints.  It tests
// DirectTranscriptionConstraint with a discrete-time system.
GTEST_TEST(DirectTranscriptionTest, DiscreteTimeConstraintTest) {
  const double kTimeStep = 1.0;
  CubicPolynomialSystem<double> system(kTimeStep);

  const auto context = system.CreateDefaultContext();
  int kNumSampleTimes = 3;
  DirectTranscription prog(&system, *context, kNumSampleTimes);

  EXPECT_EQ(prog.fixed_timestep(), kTimeStep);

  // Sets all decision variables to trivial known values (1,2,3,...).
  prog.SetInitialGuessForAllVariables(
      Eigen::VectorXd::LinSpaced(prog.num_vars(), 1, prog.num_vars()));

  // Constructor should add dynamic constraints, and these are the
  // only generic constraints that should be in the program so far.
  const std::vector<solvers::Binding<solvers::Constraint>>&
      dynamic_constraints = prog.generic_constraints();
  EXPECT_EQ(dynamic_constraints.size(), kNumSampleTimes - 1);

  using std::pow;
  for (int i = 0; i < (kNumSampleTimes - 1); i++) {
    EXPECT_EQ(
        prog.EvalBindingAtInitialGuess(dynamic_constraints[i])[0],
        prog.GetInitialGuess(prog.state(i + 1)[0]) -
            pow(prog.GetInitialGuess(prog.state(i)[0]), 3.0));
  }
}

// This example will not use the symbolic constraints (because it has nonlinear
// dynamics).  It tests DirectTranscriptionConstraint with a continuous-time
// system.
GTEST_TEST(DirectTranscriptionTest, ContinuousTimeConstraintTest) {
  const double kTimeStep = 0.25;

  // xdot = sin(x)
  using std::sin;
  Variable x{"x"};
  SymbolicVectorSystem<double> system(
      {}, Vector1<Variable>{x}, Vector0<Variable>{},
      Vector1<Expression>{sin(x)}, Vector0<Expression>{});

  const auto context = system.CreateDefaultContext();
  int kNumSampleTimes = 3;
  DirectTranscription prog(&system, *context, kNumSampleTimes,
                           TimeStep{kTimeStep});

  EXPECT_EQ(prog.fixed_timestep(), kTimeStep);

  // Sets all decision variables to trivial known values (1,2,3,...).
  prog.SetInitialGuessForAllVariables(
      Eigen::VectorXd::LinSpaced(prog.num_vars(), 1, prog.num_vars()));

  // Constructor should add dynamic constraints, and these are the
  // only generic constraints that should be in the program so far.
  const std::vector<solvers::Binding<solvers::Constraint>>&
      dynamic_constraints = prog.generic_constraints();
  EXPECT_EQ(dynamic_constraints.size(), kNumSampleTimes - 1);
  // In fact, there are no other constraints (at least not yet):
  EXPECT_EQ(dynamic_constraints.size(), prog.GetAllConstraints().size());

  using std::pow;
  for (int i = 0; i < (kNumSampleTimes - 1); i++) {
    double state_i = prog.GetInitialGuess(prog.state(i))[0];
    EXPECT_NEAR(
        prog.EvalBindingAtInitialGuess(dynamic_constraints[i])[0],
        prog.GetInitialGuess(prog.state(i + 1)[0]) - state_i -
        kTimeStep*sin(state_i), 1e-14);
  }
}

// This example WILL use the symbolic constraints; it tests an affine
// discrete-time system.
GTEST_TEST(DirectTranscriptionTest, DiscreteTimeSymbolicConstraintTest) {
  const double kTimeStep = 0.1;
  std::unique_ptr<AffineSystem<double>> system = MakeAffineSystem(kTimeStep);

  const auto context = system->CreateDefaultContext();
  int kNumSampleTimes = 3;
  DirectTranscription prog(system.get(), *context, kNumSampleTimes);

  EXPECT_EQ(prog.fixed_timestep(), kTimeStep);

  // Sets all decision variables to trivial known values (1,2,3,...).
  prog.SetInitialGuessForAllVariables(
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
        prog.EvalBindingAtInitialGuess(dynamic_constraints[i]) -
        dynamic_constraints[i].evaluator()->lower_bound();
    const Vector1d dynamic_constraint_expected =
        prog.GetInitialGuess(prog.state(i + 1)) -
        system->A() * prog.GetInitialGuess(prog.state(i)) -
        system->B() * prog.GetInitialGuess(prog.input(i)) - system->f0();

    // Check that the system's state equation still holds with equality,
    // regardless of the specific encoding MathematicalProgram chooses.
    EXPECT_TRUE(
        CompareMatrices(dynamic_constraint_val, dynamic_constraint_expected) ||
        CompareMatrices(dynamic_constraint_val, -dynamic_constraint_expected));
  }
}

// This example WILL use the symbolic constraints; it tests a continuous-time
// system.
GTEST_TEST(DirectTranscriptionTest, ContinuousTimeSymbolicConstraintTest) {
  std::unique_ptr<AffineSystem<double>> system = MakeAffineSystem(0.0);

  const auto context = system->CreateDefaultContext();
  int kNumSampleTimes = 3;
  const double kTimeStep = 0.1;
  DirectTranscription prog(system.get(), *context, kNumSampleTimes,
                           TimeStep{kTimeStep});

  EXPECT_EQ(prog.fixed_timestep(), kTimeStep);

  // Sets all decision variables to trivial known values (1,2,3,...).
  prog.SetInitialGuessForAllVariables(
      Eigen::VectorXd::LinSpaced(prog.num_vars(), 1, prog.num_vars()));

  // Constructor should add dynamic constraints, and these should have been
  // added as linear equality constraints.  Since system has an input, there
  // is also one additional constraint from
  // ConstrainEqualInputAtFinalTwoTimesteps.
  const std::vector<solvers::Binding<solvers::LinearEqualityConstraint>>&
      dynamic_constraints = prog.linear_equality_constraints();
  EXPECT_EQ(dynamic_constraints.size(), kNumSampleTimes);
  // Check that there are no nonlinear constraints in the program.
  EXPECT_EQ(prog.generic_constraints().size(), 0);

  // All but the last constraint are DirectTranscriptionConstraints.
  for (int i = 0; i < (kNumSampleTimes - 1); i++) {
    const Vector1d dynamic_constraint_val =
        prog.EvalBindingAtInitialGuess(dynamic_constraints[i]) -
        dynamic_constraints[i].evaluator()->lower_bound();
    const Vector1d dynamic_constraint_expected =
        prog.GetInitialGuess(prog.state(i + 1)) -
        prog.GetInitialGuess(prog.state(i)) -
        kTimeStep * system->A() * prog.GetInitialGuess(prog.state(i)) -
        kTimeStep * system->B() * prog.GetInitialGuess(prog.input(i)) -
        kTimeStep * system->f0();

    // Check that the system's state equation still holds with equality,
    // regardless of the specific encoding MathematicalProgram chooses.
    EXPECT_TRUE(CompareMatrices(dynamic_constraint_val,
        dynamic_constraint_expected, 1e-14));
  }
}

void SolvePendulumTrajectory(bool continuous_time) {
  // Only solve under SNOPT (IPOPT is unreliable here).
  solvers::SnoptSolver snopt_solver;
  if (!snopt_solver.is_available()) {
    drake::log()->warn("SNOPT is unavaible; this test shall be skipped.");
    return;
  }

  SCOPED_TRACE(fmt::format("continuous_time = {}", continuous_time));

  const char* const urdf_path =
      "drake/examples/pendulum/Pendulum.urdf";

  // The time step here is somewhat arbitrary. The value chosen here
  // provides a reasonably fast solve.
  const double kFixedTimeStep = 0.1;

  const double kTimeStep = continuous_time ? 0 : kFixedTimeStep;

  auto pendulum =
      std::make_unique<multibody::MultibodyPlant<double>>(kTimeStep);
  multibody::Parser parser(pendulum.get());
  parser.AddModelFromFile(FindResourceOrThrow(urdf_path));
  pendulum->Finalize();

  // Create the DirectTranscription object, and specify which input port
  // on the MultibodyPlant corresponds to the control input.
  auto context = pendulum->CreateDefaultContext();
  const InputPortIndex actuation_port_index =
      pendulum->get_actuation_input_port().get_index();
  const int kNumTimeSamples = 50;
  std::unique_ptr<DirectTranscription> dirtran{};
  if (continuous_time) {
    dirtran = std::make_unique<DirectTranscription>(
        pendulum.get(), *context, kNumTimeSamples, TimeStep(kFixedTimeStep),
        actuation_port_index);
  } else {
    dirtran = std::make_unique<DirectTranscription>(
        pendulum.get(), *context, kNumTimeSamples,
        actuation_port_index);
  }

  // Adds a torque actuation limit.
  const double kTorqueLimit = 3.0;  // N*m.
  const solvers::VectorXDecisionVariable& u = dirtran->input();
  dirtran->AddConstraintToAllKnotPoints(-kTorqueLimit <= u(0));
  dirtran->AddConstraintToAllKnotPoints(u(0) <= kTorqueLimit);

  BasicVector<double> initial_state(Eigen::VectorXd::Zero(2));
  BasicVector<double> final_state(Eigen::VectorXd::Zero(2));

  // Set the initial and final state constraints.
  const int kTheta_index = 0, kThetadot_index = 1;
  initial_state.SetAtIndex(kTheta_index, 0.0);
  initial_state.SetAtIndex(kThetadot_index, 0.0);
  final_state.SetAtIndex(kTheta_index, M_PI);
  final_state.SetAtIndex(kThetadot_index, 0.0);
  dirtran->AddLinearConstraint(dirtran->initial_state() ==
                               initial_state.get_value());
  dirtran->AddLinearConstraint(dirtran->final_state() ==
                               final_state.get_value());

  const double R = 10;  // Cost on input "effort".
  dirtran->AddRunningCost((R * u) * u);

  // Create an initial guess for the state trajectory.
  const double timespan_init = 4;
  auto traj_init_x = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init},
      {initial_state.get_value(), final_state.get_value()});
  dirtran->SetInitialTrajectory(PiecewisePolynomial<double>(), traj_init_x);

  const auto result = snopt_solver.Solve(*dirtran, {}, {});
  EXPECT_TRUE(result.is_success());
}

// Tests that MultibodyPlant can be optimized using both the discrete and
// continuous time entry points into the plant.
GTEST_TEST(DirectTranscriptionTest, MultibodyDiscreteTest) {
  SolvePendulumTrajectory(false);
}

GTEST_TEST(DirectTranscriptionTest, MultibodyContinuousTest) {
  SolvePendulumTrajectory(true);
}

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
  prog.SetInitialGuessForAllVariables(
      Eigen::VectorXd::LinSpaced(prog.num_vars(), 1, prog.num_vars()));

  // Constructor should add dynamic constraints, and these should have been
  // added as linear equality constraints.  There is also one additional
  // constraints from ConstrainEqualInputAtFinalTwoTimesteps.
  const std::vector<solvers::Binding<solvers::LinearEqualityConstraint>>&
      dynamic_constraints = prog.linear_equality_constraints();
  EXPECT_EQ(dynamic_constraints.size(), kNumSampleTimes);
  // Check that there are no nonlinear constraints in the program.
  EXPECT_EQ(prog.generic_constraints().size(), 0);

  const double kNumericalTolerance = 1e-10;
  for (int i = 0; i < (kNumSampleTimes - 1); i++) {
    EXPECT_TRUE(CompareMatrices(
        prog.EvalBindingAtInitialGuess(dynamic_constraints[i]),
        prog.GetInitialGuess(prog.state(i + 1)) -
            A * prog.GetInitialGuess(prog.state(i)) -
            B * prog.GetInitialGuess(prog.input(i)),
        kNumericalTolerance));
  }
}

// This example tests the TimeVaryingLinearSystem overload of the
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
  TrajectoryLinearSystem<double> system(A, B, C, D, kTimeStep);

  const auto context = system.CreateDefaultContext();
  int kNumSampleTimes = 3;
  DirectTranscription prog(&system, *context, kNumSampleTimes);

  EXPECT_EQ(prog.fixed_timestep(), kTimeStep);

  // Sets all decision variables to trivial known values (1,2,3,...).
  prog.SetInitialGuessForAllVariables(
      Eigen::VectorXd::LinSpaced(prog.num_vars(), 1, prog.num_vars()));

  // Constructor should add dynamic constraints, and these should have been
  // added as linear equality constraints.  There is also one additional
  // constraint from ConstrainEqualInputAtFinalTwoTimesteps.
  const std::vector<solvers::Binding<solvers::LinearEqualityConstraint>>&
      dynamic_constraints = prog.linear_equality_constraints();
  EXPECT_EQ(dynamic_constraints.size(), kNumSampleTimes);
  // Check that there are no nonlinear constraints in the program.
  EXPECT_EQ(prog.generic_constraints().size(), 0);

  const double kNumericalTolerance = 1e-10;
  for (int i = 0; i < (kNumSampleTimes - 1); i++) {
    const double t = system.time_period() * i;
    EXPECT_TRUE(CompareMatrices(
        prog.EvalBindingAtInitialGuess(dynamic_constraints[i]),
        prog.GetInitialGuess(prog.state(i + 1)) -
            A.value(t) * prog.GetInitialGuess(prog.state(i)) -
            B.value(t) * prog.GetInitialGuess(prog.input(i)),
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

  const solvers::MathematicalProgramResult result = Solve(prog);
  EXPECT_TRUE(result.is_success());

  // Compute the expected cost as c[N] + \sum_{i = 0...N-1} h * c[i]
  //   where c[i] is the running cost and c[N] is the terminal cost.
  double expected_cost{0.};
  for (int i{0}; i < kNumSamples - 1; i++) {
    expected_cost +=
        kTimeStep * std::pow(result.GetSolution(prog.state(i))[0], 2.);
  }

  EXPECT_NEAR(result.get_optimal_cost(), expected_cost, 1e-6);
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
  prog.SetInitialGuessForAllVariables(
      Eigen::VectorXd::LinSpaced(prog.num_vars(), 1, prog.num_vars()));

  // Constructor should add dynamic constraints, and these are the
  // only generic constraints that should be in the program so far.
  const std::vector<solvers::Binding<solvers::LinearEqualityConstraint>>&
      dynamic_constraints = prog.linear_equality_constraints();
  EXPECT_EQ(dynamic_constraints.size(), kNumSampleTimes - 1);

  for (int i = 0; i < (kNumSampleTimes - 1); i++) {
    // Checks that x[n+1] = kGain*x[n].
    EXPECT_EQ(
        prog.EvalBindingAtInitialGuess(dynamic_constraints[i])[0],
        prog.GetInitialGuess(prog.state(i + 1)[0]) -
            kGain * prog.GetInitialGuess(prog.state(i)[0]));
  }
}

// TODO(russt): Add tests for ReconstructTrajectory methods once their output is
// non-trivial.

}  // anonymous namespace
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
