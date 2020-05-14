#include "drake/systems/analysis/scalar_view_dense_output.h"

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/analysis/hermitian_dense_output.h"

namespace drake {
namespace systems {
namespace analysis {
namespace {

template <typename T>
class ScalarViewDenseOutputTest : public ::testing::Test {
 protected:
  std::unique_ptr<DenseOutput<T>> CreateDummyDenseOutput() {
    auto dense_output =
        std::make_unique<HermitianDenseOutput<T>>();
    typename HermitianDenseOutput<T>::IntegrationStep step(
        this->kInitialTime, this->kInitialState, this->kInitialStateDerivative);
    step.Extend(this->kFinalTime, this->kFinalState,
                this->kFinalStateDerivative);
    dense_output->Update(std::move(step));
    dense_output->Consolidate();
    return dense_output;
  }

  const double kInvalidTime{-1.0};
  const double kInitialTime{0.0};
  const double kMidTime{0.5};
  const double kFinalTime{1.0};
  const MatrixX<double> kInitialState{
    (MatrixX<double>(3, 1) << 0., 0., 0.).finished()};
  const MatrixX<T> kFinalState{
    (MatrixX<double>(3, 1) << 1., 10., 100.).finished()};
  const MatrixX<double> kInitialStateDerivative{
    (MatrixX<double>(3, 1) << 0., 1., 0.).finished()};
  const MatrixX<double> kFinalStateDerivative{
    (MatrixX<double>(3, 1) << 1., 0., 1.).finished()};
  const int kValidElementIndex{0};
  const int kInvalidElementIndex{10};
};

typedef ::testing::Types<double, AutoDiffXd> ExtensionTypes;

TYPED_TEST_SUITE(ScalarViewDenseOutputTest, ExtensionTypes);

// Checks that ScalarViewDenseOutput properly wraps a
// DenseOutput instance.
TYPED_TEST(ScalarViewDenseOutputTest, ExtensionConsistency) {
  // Verifies that passing a null base dense output results in an error.
  DRAKE_EXPECT_THROWS_MESSAGE(
      ScalarViewDenseOutput<TypeParam> dense_output(
          std::unique_ptr<HermitianDenseOutput<TypeParam>>(),
          this->kValidElementIndex),
      std::runtime_error, ".*dense output.*is null.*");

  // Verifies that views to invalid elements result in an error.
  DRAKE_EXPECT_THROWS_MESSAGE(
      ScalarViewDenseOutput<TypeParam> dense_output(
          this->CreateDummyDenseOutput(), this->kInvalidElementIndex),
      std::runtime_error, ".*out of.*dense output.*range.*");

  // Instantiates scalar continuous extension properly.
  ScalarViewDenseOutput<TypeParam> dense_output(
      this->CreateDummyDenseOutput(), this->kValidElementIndex);

  // Retrieves dummy base continuous extension.
  const DenseOutput<TypeParam>* base_output =
      dense_output.get_base_output();

  // Checks basic getters for consistency.
  EXPECT_EQ(dense_output.is_empty(),
            base_output->is_empty());
  EXPECT_EQ(dense_output.start_time(),
            base_output->start_time());
  EXPECT_EQ(dense_output.end_time(),
            base_output->end_time());
  EXPECT_EQ(dense_output.size(), 1);

  // Checks evaluation preconditions.
  DRAKE_EXPECT_THROWS_MESSAGE(
      dense_output.Evaluate(this->kInvalidTime),
      std::runtime_error,
      ".*[Tt]ime.*out of.*dense output.*domain.*");

  // Compares evaluations for consistency.
  EXPECT_EQ(
      dense_output.EvaluateScalar(this->kInitialTime),
      base_output->EvaluateNth(this->kInitialTime, this->kValidElementIndex));
  EXPECT_EQ(
      dense_output.EvaluateScalar(this->kMidTime),
      base_output->EvaluateNth(this->kMidTime, this->kValidElementIndex));
  EXPECT_EQ(
      dense_output.EvaluateScalar(this->kFinalTime),
      base_output->EvaluateNth(this->kFinalTime, this->kValidElementIndex));
}

}  // namespace
}  // namespace analysis
}  // namespace systems
}  // namespace drake
