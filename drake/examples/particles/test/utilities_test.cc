#include "drake/examples/particles/utilities.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_port_descriptor.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace examples {
namespace particles {
namespace {

///
/// A test fixture class for MakeDegenerateEulerJoint tests.
///
/// @tparam T must be a valid Eigen ScalarType.
///
template <typename T>
class SingleDOFEulerJointTest : public ::testing::Test {
 protected:
  /// Arrange a one degree of freedom DegenerateEulerJoint as
  /// the Device Under Test.
  void SetUp() override {
    // Only the first generalized coordinate gets through.
    MatrixX<T> translating_matrix(6, 1);
    translating_matrix.setZero();
    translating_matrix(0, 0) = 1.0;
    this->dut_ = MakeDegenerateEulerJoint(translating_matrix);
    this->context_ = this->dut_->CreateDefaultContext();
    this->output_ = this->dut_->AllocateOutput(*this->context_);
  }

  /// System (aka Device Under Test) being tested.
  std::unique_ptr<systems::System<T>> dut_;
  /// Context for the given @p dut_.
  std::unique_ptr<systems::Context<T>> context_;
  /// Outputs of the given @p dut_.
  std::unique_ptr<systems::SystemOutput<T>> output_;
};

TYPED_TEST_CASE_P(SingleDOFEulerJointTest);

/// Makes sure that MakeDegenerateEulerJoint joint
/// output is the right mapping of its inputs.
TYPED_TEST_P(SingleDOFEulerJointTest, OutputTest) {
  // Set input.
  const systems::InputPortDescriptor<TypeParam>& input_descriptor =
      this->dut_->get_input_port(0);
  auto input = std::make_unique<systems::BasicVector<TypeParam>>(
      input_descriptor.size());
  input->SetZero();
  input->SetAtIndex(0, static_cast<TypeParam>(1.0));  // q0 = 1.0
  input->SetAtIndex(input->size()/2, static_cast<TypeParam>(5.0));  // v0 = 5.0
  this->context_->FixInputPort(0, std::move(input));
  // Compute outputs.
  this->dut_->CalcOutput(*this->context_, this->output_.get());
  const systems::BasicVector<TypeParam>* output =
      this->output_->get_vector_data(0);
  // Check results.
  EXPECT_EQ(output->GetAtIndex(0),
            static_cast<TypeParam>(1.0));  // q0 == 1.0
  EXPECT_EQ(output->GetAtIndex(2),
            static_cast<TypeParam>(0.0));  // q3 == 0.0
  EXPECT_EQ(output->GetAtIndex(6),
            static_cast<TypeParam>(5.0));  // v0 == 5.0
  EXPECT_EQ(output->GetAtIndex(8),
            static_cast<TypeParam>(0.0));  // v3 == 0.0
}

REGISTER_TYPED_TEST_CASE_P(SingleDOFEulerJointTest, OutputTest);

INSTANTIATE_TYPED_TEST_CASE_P(WithDoubles, SingleDOFEulerJointTest, double);

/// Makes sure that MakeDegenerateEulerJoint throws when the given
/// translating matrix doesn't imply a 6 degrees of freedom output
/// (rows != 6).
GTEST_TEST(DegenerateEulerJointDimensionalityChecks, WrongOutputDOFTest) {
  // Scalar type is fixed as it makes no difference.
  ASSERT_THROW({
      auto bad_joint = MakeDegenerateEulerJoint(MatrixX<double>(4, 4));
    }, std::runtime_error);
}

/// Makes sure that MakeDegenerateEulerJoint throws when the given
/// translating matrix implies a 6 or more degrees of freedom input
/// (cols >= 6).
GTEST_TEST(DegenerateEulerJointDimensionalityChecks, TooManyInputDOFTest) {
  // Scalar type is fixed as it makes no difference.
  ASSERT_THROW({
      auto bad_joint = MakeDegenerateEulerJoint(MatrixX<double>(6, 8));
    }, std::runtime_error);
}

/// Makes sure that MakeDegenerateEulerJoint throws when the given
/// translating matrix implies a less than 1 degree of freedom
/// input (cols < 1).
GTEST_TEST(DegenerateEulerJointDimensionalityChecks, TooFewInputDOFTest) {
  // Scalar type is fixed as it makes no difference.
  ASSERT_THROW({
      auto bad_joint = MakeDegenerateEulerJoint(MatrixX<double>(6, 0));
    }, std::runtime_error);
}

}  // namespace
}  // namespace particles
}  // namespace examples
}  // namespace drake
