#include "drake/systems/analysis/hermitian_dense_output.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace systems {
namespace analysis {
namespace {

template <typename T>
class HermitianDenseOutputTest : public ::testing::Test {
 protected:
  const double kInvalidTime{-1.0};
  const double kInitialTime{0.0};
  const double kMidTime{0.5};
  const double kFinalTime{1.0};
  const double kTimeStep{0.1};
  const MatrixX<double> kInitialState{
    (MatrixX<double>(3, 1) << 0., 0., 0.).finished()};
  const MatrixX<double> kMidState{
    (MatrixX<double>(3, 1) << 0.5, 5., 50.).finished()};
  const MatrixX<double> kFinalState{
    (MatrixX<double>(3, 1) << 1., 10., 100.).finished()};
  const MatrixX<double> kFinalStateWithFewerDimensions{
    (MatrixX<double>(2, 1) << 1., 10.).finished()};
  const MatrixX<double> kFinalStateWithMoreDimensions{
    (MatrixX<double>(4, 1) << 1., 10., 100., 1000.).finished()};
  const MatrixX<double> kFinalStateNotAVector{
    (MatrixX<double>(2, 2) << 1., 10., 100., 1000.).finished()};
  const MatrixX<double> kInitialStateDerivative{
    (MatrixX<double>(3, 1) << 0., 1., 0.).finished()};
  const MatrixX<double> kMidStateDerivative{
    (MatrixX<double>(3, 1) << 0.5, 0.5, 0.5).finished()};
  const MatrixX<double> kFinalStateDerivative{
    (MatrixX<double>(3, 1) << 1., 0., 1.).finished()};
  const MatrixX<double> kFinalStateDerivativeWithFewerDimensions{
    (MatrixX<double>(2, 1) << 1., 0.).finished()};
  const MatrixX<double> kFinalStateDerivativeWithMoreDimensions{
    (MatrixX<double>(4, 1) << 1., 0., 1., 0.).finished()};
  const MatrixX<double> kFinalStateDerivativeNotAVector{
    (MatrixX<double>(2, 2) << 0., 1., 0., 1.).finished()};
  const int kValidElementIndex{0};
  const int kInvalidElementIndex{10};

  const std::string kEmptyOutputErrorMessage{
    ".*[Dd]ense output.*empty.*"};
  const std::string kZeroLengthStepErrorMessage{
    ".*step has zero length.*"};
  const std::string kCannotRollbackErrorMessage{
    "No updates to rollback."};
  const std::string kCannotConsolidateErrorMessage{
    "No updates to consolidate."};
  const std::string kInvalidElementIndexErrorMessage{
    ".*out of.*dense output.*range.*"};
  const std::string kInvalidTimeErrorMessage{
    ".*[Tt]ime.*out of.*dense output.*domain.*"};
  const std::string kTimeMismatchErrorMessage{
    ".*start time.*end time.*differ.*"};
  const std::string kStateMismatchErrorMessage{
    ".*start state.*end state.*differ.*"};
  const std::string kStateDerivativeMismatchErrorMessage{
    ".*start state derivative.*end state derivative.*differ.*"};
  const std::string kStepBackwardsErrorMessage{
    ".*cannot be extended backwards.*"};
  const std::string kDimensionMismatchErrorMessage{
    ".*dimensions do not match.*"};
  const std::string kNotAVectorErrorMessage{".*not a column matrix.*"};
};


// HermitianDenseOutput types to test.
typedef ::testing::Types<double, AutoDiffXd> OutputTypes;

TYPED_TEST_SUITE(HermitianDenseOutputTest, OutputTypes);

// Checks that HermitianDenseOutput consistency is ensured.
TYPED_TEST(HermitianDenseOutputTest, OutputConsistency) {
  // Instantiates dense output.
  HermitianDenseOutput<TypeParam> dense_output;
  // Verifies that the dense output is empty and API behavior
  // is consistent with that fact.
  ASSERT_TRUE(dense_output.is_empty());
  DRAKE_EXPECT_THROWS_MESSAGE(
      dense_output.Evaluate(this->kInitialTime),
      std::logic_error, this->kEmptyOutputErrorMessage);
  DRAKE_EXPECT_THROWS_MESSAGE(dense_output.EvaluateNth(
      this->kInitialTime, this->kValidElementIndex),
      std::logic_error, this->kEmptyOutputErrorMessage);
  DRAKE_EXPECT_THROWS_MESSAGE(dense_output.start_time(), std::logic_error,
                             this->kEmptyOutputErrorMessage);
  DRAKE_EXPECT_THROWS_MESSAGE(dense_output.end_time(), std::logic_error,
                             this->kEmptyOutputErrorMessage);
  DRAKE_EXPECT_THROWS_MESSAGE(dense_output.size(), std::logic_error,
                             this->kEmptyOutputErrorMessage);
  DRAKE_EXPECT_THROWS_MESSAGE(dense_output.Rollback(), std::logic_error,
                             this->kCannotRollbackErrorMessage);
  DRAKE_EXPECT_THROWS_MESSAGE(dense_output.Consolidate(), std::logic_error,
                             this->kCannotConsolidateErrorMessage);

  // Verifies that trying to update the dense output with
  // a zero length step fails.
  typename HermitianDenseOutput<TypeParam>::IntegrationStep first_step(
      this->kInitialTime, this->kInitialState, this->kInitialStateDerivative);
  DRAKE_EXPECT_THROWS_MESSAGE(dense_output.Update(first_step),
                             std::runtime_error,
                             this->kZeroLengthStepErrorMessage);

  // Verifies that trying to update the dense output with
  // a valid step succeeds.
  first_step.Extend(this->kMidTime, this->kMidState,
                    this->kMidStateDerivative);
  dense_output.Update(first_step);

  // Verifies that an update does not imply a consolidation and thus
  // the dense output remains empty.
  ASSERT_TRUE(dense_output.is_empty());
  DRAKE_EXPECT_THROWS_MESSAGE(dense_output.Evaluate(this->kMidTime),
                             std::logic_error, this->kEmptyOutputErrorMessage);
  DRAKE_EXPECT_THROWS_MESSAGE(
      dense_output.EvaluateNth(this->kMidTime, this->kValidElementIndex),
      std::logic_error, this->kEmptyOutputErrorMessage);
  DRAKE_EXPECT_THROWS_MESSAGE(dense_output.start_time(), std::logic_error,
                             this->kEmptyOutputErrorMessage);
  DRAKE_EXPECT_THROWS_MESSAGE(dense_output.end_time(), std::logic_error,
                             this->kEmptyOutputErrorMessage);
  DRAKE_EXPECT_THROWS_MESSAGE(dense_output.size(), std::logic_error,
                             this->kEmptyOutputErrorMessage);

  // Consolidates all previous updates.
  dense_output.Consolidate();

  // Verifies that it is not possible to roll back updates after consolidation.
  DRAKE_EXPECT_THROWS_MESSAGE(dense_output.Rollback(), std::logic_error,
                             this->kCannotRollbackErrorMessage);

  // Verifies that it is not possible to consolidate again.
  DRAKE_EXPECT_THROWS_MESSAGE(dense_output.Consolidate(), std::logic_error,
                             this->kCannotConsolidateErrorMessage);

  // Verifies that the dense output is not empty and that it
  // reflects the data provided on updates.
  ASSERT_FALSE(dense_output.is_empty());
  EXPECT_EQ(dense_output.start_time(), first_step.start_time());
  EXPECT_EQ(dense_output.end_time(), first_step.end_time());
  EXPECT_EQ(dense_output.size(), first_step.size());
  DRAKE_EXPECT_NO_THROW(dense_output.Evaluate(this->kMidTime));
  DRAKE_EXPECT_NO_THROW(dense_output.EvaluateNth(
      this->kMidTime, this->kValidElementIndex));

  // Verifies that invalid evaluation arguments generate errors.
  DRAKE_EXPECT_THROWS_MESSAGE(
      dense_output.EvaluateNth(this->kMidTime, this->kInvalidElementIndex),
      std::runtime_error, this->kInvalidElementIndexErrorMessage);
  DRAKE_EXPECT_THROWS_MESSAGE(
      dense_output.EvaluateNth(this->kInvalidTime, this->kValidElementIndex),
      std::runtime_error, this->kInvalidTimeErrorMessage);
  DRAKE_EXPECT_THROWS_MESSAGE(
      dense_output.Evaluate(this->kInvalidTime),
      std::runtime_error, this->kInvalidTimeErrorMessage);

  // Verifies that step updates that would disrupt the output continuity
  // fail.
  typename HermitianDenseOutput<TypeParam>::IntegrationStep second_step(
      (this->kFinalTime + this->kMidTime) / 2.,
      this->kMidState, this->kMidStateDerivative);
  second_step.Extend(this->kFinalTime, this->kFinalState,
                     this->kFinalStateDerivative);

  DRAKE_EXPECT_THROWS_MESSAGE(
      dense_output.Update(second_step), std::runtime_error,
      this->kTimeMismatchErrorMessage);

  typename HermitianDenseOutput<TypeParam>::IntegrationStep third_step(
      this->kMidTime, this->kMidState * 2., this->kMidStateDerivative);
  third_step.Extend(this->kFinalTime, this->kFinalState,
                     this->kFinalStateDerivative);
  DRAKE_EXPECT_THROWS_MESSAGE(
      dense_output.Update(third_step), std::runtime_error,
      this->kStateMismatchErrorMessage);

  typename HermitianDenseOutput<TypeParam>::IntegrationStep fourth_step(
      this->kMidTime, this->kMidState, this->kMidStateDerivative * 2.);
  fourth_step.Extend(this->kFinalTime, this->kFinalState,
                     this->kFinalStateDerivative);
  DRAKE_EXPECT_THROWS_MESSAGE(
      dense_output.Update(fourth_step), std::runtime_error,
      this->kStateDerivativeMismatchErrorMessage);
}

// Checks that HermitianDenseOutput::Step consistency is ensured.
TYPED_TEST(HermitianDenseOutputTest, StepsConsistency) {
  // Verifies that zero length steps are properly constructed.
  typename HermitianDenseOutput<TypeParam>::IntegrationStep step(
      this->kInitialTime, this->kInitialState, this->kInitialStateDerivative);
  ASSERT_EQ(step.get_times().size(), 1);
  EXPECT_EQ(step.start_time(), this->kInitialTime);
  EXPECT_EQ(step.end_time(), this->kInitialTime);
  EXPECT_EQ(step.size(), this->kInitialState.rows());
  ASSERT_EQ(step.get_states().size(), 1);
  EXPECT_TRUE(CompareMatrices(step.get_states().front(), this->kInitialState));
  ASSERT_EQ(step.get_state_derivatives().size(), 1);
  EXPECT_TRUE(CompareMatrices(step.get_state_derivatives().front(),
                              this->kInitialStateDerivative));

  // Verifies that any attempt to break step consistency fails.
  DRAKE_EXPECT_THROWS_MESSAGE(
      step.Extend(this->kInvalidTime, this->kFinalState,
                  this->kFinalStateDerivative),
      std::runtime_error, this->kStepBackwardsErrorMessage);

  DRAKE_EXPECT_THROWS_MESSAGE(
      step.Extend(this->kFinalTime, this->kFinalStateWithFewerDimensions,
                  this->kFinalStateDerivative), std::runtime_error,
      this->kDimensionMismatchErrorMessage);

  DRAKE_EXPECT_THROWS_MESSAGE(
      step.Extend(this->kFinalTime, this->kFinalStateWithMoreDimensions,
                  this->kFinalStateDerivative), std::runtime_error,
      this->kDimensionMismatchErrorMessage);

  DRAKE_EXPECT_THROWS_MESSAGE(
      step.Extend(this->kFinalTime, this->kFinalStateNotAVector,
                  this->kFinalStateDerivative), std::runtime_error,
      this->kNotAVectorErrorMessage);

  DRAKE_EXPECT_THROWS_MESSAGE(
      step.Extend(this->kFinalTime, this->kFinalState,
                  this->kFinalStateDerivativeWithFewerDimensions),
      std::runtime_error, this->kDimensionMismatchErrorMessage);

  DRAKE_EXPECT_THROWS_MESSAGE(
      step.Extend(this->kFinalTime, this->kFinalState,
                  this->kFinalStateDerivativeWithMoreDimensions),
      std::runtime_error, this->kDimensionMismatchErrorMessage);

  DRAKE_EXPECT_THROWS_MESSAGE(
      step.Extend(this->kFinalTime, this->kFinalState,
                  this->kFinalStateDerivativeNotAVector),
      std::runtime_error, this->kNotAVectorErrorMessage);

  // Extends the step with appropriate values.
  step.Extend(this->kFinalTime, this->kFinalState, this->kFinalStateDerivative);

  // Verifies that the step was properly extended.
  EXPECT_EQ(step.get_times().size(), 2);
  EXPECT_EQ(step.start_time(), this->kInitialTime);
  EXPECT_EQ(step.end_time(), this->kFinalTime);
  EXPECT_EQ(step.size(), this->kInitialState.rows());
  EXPECT_EQ(step.get_states().size(), 2);
  EXPECT_TRUE(CompareMatrices(step.get_states().back(), this->kFinalState));
  EXPECT_EQ(step.get_state_derivatives().size(), 2);
  EXPECT_TRUE(CompareMatrices(step.get_state_derivatives().back(),
                              this->kFinalStateDerivative));
}

// Checks that HermitianDenseOutput properly supports stepwise
// construction.
TYPED_TEST(HermitianDenseOutputTest, CorrectConstruction) {
  // Instantiates dense output.
  HermitianDenseOutput<TypeParam> dense_output;
  // Updates output for the first time.
  typename HermitianDenseOutput<TypeParam>::IntegrationStep first_step(
      this->kInitialTime, this->kInitialState, this->kInitialStateDerivative);
  first_step.Extend(this->kMidTime, this->kMidState, this->kMidStateDerivative);
  dense_output.Update(first_step);
  // Updates output a second time.
  typename HermitianDenseOutput<TypeParam>::IntegrationStep second_step(
      this->kMidTime, this->kMidState, this->kMidStateDerivative);
  second_step.Extend(this->kFinalTime, this->kFinalState,
                     this->kFinalStateDerivative);
  dense_output.Update(second_step);
  // Rolls back the last update.
  dense_output.Rollback();  // `second_step`
  // Consolidates existing updates.
  dense_output.Consolidate();  // only `first_step`

  // Verifies that the dense output only reflects the first step.
  EXPECT_FALSE(dense_output.is_empty());
  EXPECT_EQ(dense_output.start_time(), first_step.start_time());
  EXPECT_EQ(dense_output.end_time(), first_step.end_time());
  EXPECT_EQ(dense_output.size(), first_step.size());
  EXPECT_TRUE(CompareMatrices(dense_output.Evaluate(this->kInitialTime),
                              first_step.get_states().front()));
  EXPECT_TRUE(CompareMatrices(dense_output.Evaluate(this->kMidTime),
                              first_step.get_states().back()));
}

// Checks that HermitianDenseOutput properly implements and evaluates
// an Hermite interpolator.
TYPED_TEST(HermitianDenseOutputTest, CorrectEvaluation) {
  // Creates an Hermite cubic spline with times, states and state
  // derivatives.
  const std::vector<double> spline_times{
    this->kInitialTime, this->kMidTime, this->kFinalTime};
  const std::vector<MatrixX<double>> spline_states{
          this->kInitialState, this->kMidState, this->kFinalState};
  const std::vector<MatrixX<double>> spline_state_derivatives{
          this->kInitialStateDerivative, this->kMidStateDerivative,
          this->kFinalStateDerivative};
  const trajectories::PiecewisePolynomial<double> hermite_spline =
      trajectories::PiecewisePolynomial<double>::CubicHermite(
          spline_times, spline_states, spline_state_derivatives);
  // Instantiates dense output.
  HermitianDenseOutput<TypeParam> dense_output;
  // Updates output for the first time.
  typename HermitianDenseOutput<TypeParam>::IntegrationStep first_step(
      this->kInitialTime, this->kInitialState, this->kInitialStateDerivative);
  first_step.Extend(this->kMidTime, this->kMidState, this->kMidStateDerivative);
  dense_output.Update(first_step);
  // Updates output a second time.
  typename HermitianDenseOutput<TypeParam>::IntegrationStep second_step(
      this->kMidTime, this->kMidState, this->kMidStateDerivative);
  second_step.Extend(this->kFinalTime, this->kFinalState,
                     this->kFinalStateDerivative);
  dense_output.Update(second_step);
  // Consolidates all previous updates.
  dense_output.Consolidate();
  // Verifies that dense output and Hermite spline match.
  const double kAccuracy{1e-12};
  EXPECT_FALSE(dense_output.is_empty());
  for (TypeParam t = this->kInitialTime;
       t <= this->kFinalTime; t += this->kTimeStep) {
    const MatrixX<double> matrix_value =
        hermite_spline.value(ExtractDoubleOrThrow(t));
    const VectorX<TypeParam> vector_value =
        matrix_value.col(0).template cast<TypeParam>();
    EXPECT_TRUE(CompareMatrices(dense_output.Evaluate(t),
                                vector_value, kAccuracy));
  }
}

// Construct a HermitianDenseOutput<T> from PiecewisePolynomial<U>.
template <typename T>
void TestScalarType() {
  const Vector3<T> breaks(0.0, 1.0, 2.0);
  const RowVector3<T> samples(6.0, 5.0, 4.0);

  const auto foh =
      trajectories::PiecewisePolynomial<T>::FirstOrderHold(breaks, samples);

  const HermitianDenseOutput<T> hdo(foh);

  EXPECT_EQ(ExtractDoubleOrThrow(hdo.start_time()),
            ExtractDoubleOrThrow(breaks(0)));
  EXPECT_EQ(ExtractDoubleOrThrow(hdo.end_time()),
            ExtractDoubleOrThrow(breaks(2)));

  for (const T& time : {0.1, 0.4, 1.6}) {
    EXPECT_NEAR(ExtractDoubleOrThrow(hdo.Evaluate(time)(0)),
                ExtractDoubleOrThrow(foh.value(time)(0)), 1e-14);
  }
}

GTEST_TEST(HermitianDenseOutputTest, ConstructFromPiecewisePolynomialTest) {
  TestScalarType<double>();
  TestScalarType<AutoDiffXd>();
  TestScalarType<symbolic::Expression>();
}

}  // namespace
}  // namespace analysis
}  // namespace systems
}  // namespace drake
