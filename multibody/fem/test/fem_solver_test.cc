#include "drake/multibody/fem/fem_solver.h"

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
constexpr double kTolerance = 16 * std::numeric_limits<double>::epsilon();
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
  DummyModel<T::value> model_{};
  AccelerationNewmarkScheme<double> integrator_{kDt, kGamma, kBeta};
  FemSolver<double> solver_{&model_, &integrator_};
};

namespace {

TYPED_TEST_SUITE_P(FemSolverTest);
TYPED_TEST_P(FemSolverTest, Tolerance) {
  /* Default values. */
  EXPECT_EQ(this->solver_.relative_tolerance(), 1e-4);
  EXPECT_EQ(this->solver_.absolute_tolerance(), 1e-6);
  /* Test Setters. */
  constexpr double kTol = 1e-8;
  this->solver_.set_relative_tolerance(kTol);
  this->solver_.set_absolute_tolerance(kTol);
  EXPECT_EQ(this->solver_.relative_tolerance(), kTol);
  EXPECT_EQ(this->solver_.absolute_tolerance(), kTol);
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
  const int num_iterations = this->solver_.AdvanceOneTimeStep(
      *state0, dummy_data, nonparticipating_vertices);
  EXPECT_EQ(num_iterations, 1);

  /* Compute the expected result from AdvanceOneTimeStep(). */
  std::unique_ptr<FemState<double>> expected_state =
      this->model_.MakeFemState();
  auto tangent_matrix0 = this->model_.MakeTangentMatrix();
  this->model_.CalcTangentMatrix(*state0, this->integrator_.GetWeights(),
                                 tangent_matrix0.get());
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
  this->model_.CalcTangentMatrix(computed_state, this->integrator_.GetWeights(),
                                 tangent_matrix.get());
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
 * the Newton solver doesn't converge within the max number of iterations. */
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

TYPED_TEST_P(FemSolverTest, SetNextFemState) {
  std::unique_ptr<FemState<double>> default_state = this->model_.MakeFemState();
  const FemState<double>& next_state = this->solver_.next_fem_state();

  this->solver_.SetNextFemState(next_state);

  /* Verify that the next state is set. */
  const FemState<double>& next_state2 = this->solver_.next_fem_state();
  EXPECT_EQ(next_state.num_dofs(), next_state2.num_dofs());
  EXPECT_EQ(next_state.num_nodes(), next_state2.num_nodes());
  EXPECT_EQ(next_state.GetPositions(), next_state2.GetPositions());
  EXPECT_EQ(next_state.GetVelocities(), next_state2.GetVelocities());
  EXPECT_EQ(next_state.GetAccelerations(), next_state2.GetAccelerations());

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
