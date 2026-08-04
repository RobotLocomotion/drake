#include "drake/multibody/fem/fem_solver.h"

#include <limits>
#include <memory>
#include <unordered_set>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/fem/acceleration_newmark_scheme.h"
#include "drake/multibody/fem/test/dummy_model.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

using Eigen::MatrixXd;
constexpr double kTolerance = 1e-12;
/* Parameters for the Newmark-beta integration scheme. */
constexpr double kDt = 0.01;
constexpr double kGamma = 0.5;
constexpr double kBeta = 0.25;

/* Boolean wrapper to help feed true/false into the typed test FemSolverTest. */
template <bool b>
struct BoolWrapper {
  static constexpr bool value = b;
};

/* Unit test for FemSolver. We template the test on whether the model is linear
 or not to cover code path where we leverage the explicit linearity knowledge as
 well as the generic Newton Raphson solve.
 @tparam T BoolWrapper<true> or BoolWrapper<false>. */
template <typename T>
class FemSolverTest : public ::testing::Test {
 protected:
  void set_max_newton_iterations(int max_iterations) {
    solver_.max_iterations_ = max_iterations;
  }
  AccelerationNewmarkScheme<double> integrator_{kDt, kGamma, kBeta};
  DummyModel<T::value> model_{integrator_.GetWeights()};
  FemSolver<double> solver_{&model_, &integrator_};
};

namespace {

TYPED_TEST_SUITE_P(FemSolverTest);
TYPED_TEST_P(FemSolverTest, Tolerance) {
  /* Default values. */
  EXPECT_EQ(this->solver_.relative_tolerance(), 1e-2);
  EXPECT_EQ(this->solver_.absolute_tolerance(), 1e-6);
  EXPECT_EQ(this->solver_.max_linear_solver_tolerance(), 0.1);
  /* Test Setters. */
  constexpr double kTol = 1e-8;
  this->solver_.set_relative_tolerance(kTol);
  this->solver_.set_absolute_tolerance(kTol);
  this->solver_.set_max_linear_solver_tolerance(kTol);
  EXPECT_EQ(this->solver_.relative_tolerance(), kTol);
  EXPECT_EQ(this->solver_.absolute_tolerance(), kTol);
  EXPECT_EQ(this->solver_.max_linear_solver_tolerance(), kTol);
  /* Linear solver tolerance. */
  const double max_tol = 0.2;
  double residual = 1e-4;
  double prev_residual = 1e-3;
  double prev_tol = 0.1;
  this->solver_.set_max_linear_solver_tolerance(max_tol);
  /* Negative previous tolerance results in default. */
  EXPECT_EQ(
      this->solver_.ComputeLinearSolverTolerance(residual, prev_residual, -1),
      max_tol);
  /* The case with tolerance = γ*ratio^α where ratio = ‖Fₖ‖/‖Fₖ₋₁‖. */
  const double ratio = residual / prev_residual;
  EXPECT_EQ(this->solver_.ComputeLinearSolverTolerance(residual, prev_residual,
                                                       prev_tol),
            ratio * ratio);
  /* The case where residual is shrinking fast. */
  residual = 1e-6;
  EXPECT_EQ(this->solver_.ComputeLinearSolverTolerance(residual, prev_residual,
                                                       prev_tol),
            prev_tol * prev_tol);
  /* The case where residual unexpectedly grows, and the safe-guard kicks in. */
  residual = 2.0 * prev_residual;
  EXPECT_EQ(this->solver_.ComputeLinearSolverTolerance(residual, prev_residual,
                                                       prev_tol),
            max_tol);
}

/* Tests that the behavior of FemSolver::AdvanceOneTimeStep agrees with analytic
 results. The DummyModel contains DummyElements that give non-zero residual at
 zero state and zero residual everywhere else. The dummy element also has
 constant stiffness, damping, and mass matrix (aka they are state-independent).
 As a result, we expect AdvanceOneTimeStep to converge after exactly one Newton
 iteration, and the unknown variable z (acceleration in this case) should
 satisfy A*z = -b, where A is the constant tangent matrix and b is the nonzero
 residual evaluated at the zero state. */
TYPED_TEST_P(FemSolverTest, AdvanceOneTimeStep) {
  constexpr bool is_linear = TypeParam::value;
  typename DummyModel<is_linear>::DummyBuilder builder(&this->model_);
  builder.AddTwoElementsWithSharedNodes();
  builder.Build();
  std::unique_ptr<FemState<double>> state0 = this->model_.MakeFemState();
  const std::unordered_set<int> nonparticipating_vertices = {0, 1};
  const systems::LeafContext<double> dummy_context;
  const FemPlantData<double> dummy_data{dummy_context, {}};
  /* Set a tight linear solve tolerance to show that we can converge to tight
   tolerances if needed. */
  this->solver_.set_max_linear_solver_tolerance(
      std::numeric_limits<double>::epsilon());
  const int num_iterations = this->solver_.AdvanceOneTimeStep(
      *state0, dummy_data, nonparticipating_vertices);
  EXPECT_EQ(num_iterations, 1);
  /* Compute the expected result from AdvanceOneTimeStep(). */
  std::unique_ptr<FemState<double>> expected_state =
      this->model_.MakeFemState();
  auto tangent_matrix0 = this->model_.MakeTangentMatrix();
  this->model_.CalcTangentMatrix(*state0, tangent_matrix0.get());
  const MatrixXd A0 = tangent_matrix0->MakeDenseMatrix();
  VectorX<double> b0(this->model_.num_dofs());
  this->model_.CalcResidual(*state0, dummy_data, &b0);
  /* The solver is not considered as "converged" with the initial state. */
  EXPECT_FALSE(this->solver_.solver_converged(b0.norm(), b0.norm()));
  Eigen::LLT<MatrixXd> llt;
  llt.compute(A0);
  const VectorX<double> dz = llt.solve(-b0);
  this->integrator_.UpdateStateFromChangeInUnknowns(dz, expected_state.get());
  const FemState<double>& computed_state = this->solver_.next_fem_state();
  EXPECT_TRUE(CompareMatrices(expected_state->GetPositions(),
                              computed_state.GetPositions(), kTolerance));
  EXPECT_TRUE(CompareMatrices(expected_state->GetAccelerations(),
                              computed_state.GetAccelerations(), kTolerance));
  EXPECT_TRUE(CompareMatrices(expected_state->GetVelocities(),
                              computed_state.GetVelocities(), kTolerance));

  /* Check the Schur complement is as expected. */
  auto tangent_matrix = this->model_.MakeTangentMatrix();
  this->model_.CalcTangentMatrix(computed_state, tangent_matrix.get());
  contact_solvers::internal::SchurComplement
      force_balance_tangent_matrix_schur_complement(*tangent_matrix,
                                                    nonparticipating_vertices);
  const MatrixXd expected_schur_complement =
      force_balance_tangent_matrix_schur_complement.get_D_complement();
  const contact_solvers::internal::SchurComplement& computed_schur_complement =
      this->solver_.next_schur_complement();
  EXPECT_TRUE(CompareMatrices(expected_schur_complement,
                              computed_schur_complement.get_D_complement(),
                              kTolerance, MatrixCompareType::relative));
}

/* Tests that AdvanceOneTimeStep for nonlinear models throws an error message if
 the Newton solver doesn't converge within the max number of iterations. */
TYPED_TEST_P(FemSolverTest, Nonconvergence) {
  constexpr bool is_linear = TypeParam::value;
  if (!is_linear) {
    typename DummyModel<is_linear>::DummyBuilder builder(&this->model_);
    builder.AddTwoElementsWithSharedNodes();
    builder.Build();
    std::unique_ptr<FemState<double>> state0 = this->model_.MakeFemState();
    const std::unordered_set<int> nonparticipating_vertices = {0, 1};
    const systems::LeafContext<double> dummy_context;
    const FemPlantData<double> dummy_data{dummy_context, {}};
    /* We set up the model so that nonlinear solves takes exactly one iteration
     to converge. */
    this->set_max_newton_iterations(0);
    DRAKE_EXPECT_THROWS_MESSAGE(
        this->solver_.AdvanceOneTimeStep(*state0, dummy_data,
                                         nonparticipating_vertices),
        ".*failed.*");
  }
}

TYPED_TEST_P(FemSolverTest, DefaultStateAndSchurComplement) {
  std::unique_ptr<FemState<double>> default_state = this->model_.MakeFemState();
  const FemState<double>& next_state = this->solver_.next_fem_state();

  EXPECT_TRUE(CompareMatrices(default_state->GetPositions(),
                              next_state.GetPositions()));
  EXPECT_TRUE(CompareMatrices(default_state->GetAccelerations(),
                              next_state.GetAccelerations()));
  EXPECT_TRUE(CompareMatrices(default_state->GetVelocities(),
                              next_state.GetVelocities()));
  const contact_solvers::internal::SchurComplement next_schur_complement =
      this->solver_.next_schur_complement();
  EXPECT_EQ(next_schur_complement.get_D_complement().rows(), 0);
  EXPECT_EQ(next_schur_complement.get_D_complement().cols(), 0);
}

/* Tests that SetNextFemState properly copies the state variables. */
TYPED_TEST_P(FemSolverTest, SetNextFemState) {
  constexpr bool is_linear = TypeParam::value;
  typename DummyModel<is_linear>::DummyBuilder builder(&this->model_);
  builder.AddTwoElementsWithSharedNodes();
  builder.Build();

  /* Construct an arbitrary target state. */
  std::unique_ptr<FemState<double>> state0 = this->model_.MakeFemState();

  /* Compute the next FEM state. */
  const systems::LeafContext<double> dummy_context;
  const FemPlantData<double> dummy_data{dummy_context, {}};
  this->solver_.AdvanceOneTimeStep(*state0, dummy_data, {});

  /* Make some arbitrary state value that's different from the next FEM state.
   */
  const int num_dofs = state0->num_dofs();
  auto q = Eigen::VectorXd::LinSpaced(num_dofs, 1.0, 2.0);
  auto q0 = Eigen::VectorXd::LinSpaced(num_dofs, 2.0, 3.0);
  auto v = Eigen::VectorXd::LinSpaced(num_dofs, 3.0, 4.0);
  auto a = Eigen::VectorXd::LinSpaced(num_dofs, 4.0, 5.0);
  {
    const FemState<double>& next_state = this->solver_.next_fem_state();
    EXPECT_NE(next_state.GetPositions(), q);
    EXPECT_NE(next_state.GetPreviousStepPositions(), q0);
    EXPECT_NE(next_state.GetVelocities(), v);
    EXPECT_NE(next_state.GetAccelerations(), a);
  }

  /* Now force the next state to be the arbitrary values from above. */
  std::unique_ptr<FemState<double>> state1 = this->model_.MakeFemState();
  state1->SetPositions(q);
  state1->SetTimeStepPositions(q0);
  state1->SetVelocities(v);
  state1->SetAccelerations(a);
  this->solver_.SetNextFemState(*state1);

  /* Verify that the next state is set to the expected values. */
  const FemState<double>& next_state = this->solver_.next_fem_state();
  EXPECT_EQ(next_state.num_dofs(), state1->num_dofs());
  EXPECT_EQ(next_state.num_nodes(), state1->num_nodes());
  EXPECT_EQ(next_state.GetPositions(), state1->GetPositions());
  EXPECT_EQ(next_state.GetPreviousStepPositions(),
            state1->GetPreviousStepPositions());
  EXPECT_EQ(next_state.GetVelocities(), state1->GetVelocities());
  EXPECT_EQ(next_state.GetAccelerations(), state1->GetAccelerations());

  /* Verify that the schur complement at the next time step is emptied. */
  const contact_solvers::internal::SchurComplement next_schur_complement =
      this->solver_.next_schur_complement();
  EXPECT_EQ(next_schur_complement.get_D_complement().rows(), 0);
  EXPECT_EQ(next_schur_complement.get_D_complement().cols(), 0);
}

using AllTypes = ::testing::Types<BoolWrapper<true>, BoolWrapper<false>>;
REGISTER_TYPED_TEST_SUITE_P(FemSolverTest, Tolerance, AdvanceOneTimeStep,
                            Nonconvergence, DefaultStateAndSchurComplement,
                            SetNextFemState);
INSTANTIATE_TYPED_TEST_SUITE_P(LinearAndNonLinear, FemSolverTest, AllTypes);

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
