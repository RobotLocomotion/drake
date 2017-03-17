#include "drake/systems/primitives/saturation.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/input_port_value.h"

namespace drake {
namespace systems {
namespace {

template <typename T>
void TestInputAndOutput(const Saturation<T>& saturation_system,
                        std::unique_ptr<Context<T>> context,
                        const VectorX<T>& input_vector,
                        const VectorX<T>& expected_output) {
  const int port_size = saturation_system.get_size();

  // Verifies that Saturation allocates no state variables in the context.
  EXPECT_EQ(context->get_continuous_state()->size(), 0);
  auto output = saturation_system.AllocateOutput(*context);
  auto input = std::make_unique<BasicVector<T>>(port_size);

  input->get_mutable_value() << input_vector;

  // Hook input of the expected size.
  context->FixInputPort(saturation_system.get_input_port().get_index(),
                        std::move(input));

  saturation_system.CalcOutput(*context, output.get());

  // Checks that the number of output ports in the Saturation system and the
  // SystemOutput are consistent.
  ASSERT_EQ(output->get_num_ports(), 1);
  ASSERT_EQ(saturation_system.get_num_output_ports(), 1);
  const BasicVector<T>* output_vector = output->get_vector_data(0);
  ASSERT_NE(output_vector, nullptr);
  EXPECT_EQ(output_vector->get_value(), expected_output);
}

template <typename T>
void TestConstantSaturation(const Saturation<T>& saturation_system,
                            const VectorX<T>& input_vector,
                            const VectorX<T>& expected_output) {
  auto context = saturation_system.CreateDefaultContext();

  // Checks that the number of input ports in the Saturation system and the
  // Context are consistent.
  ASSERT_EQ(saturation_system.get_num_input_ports(), 1);
  ASSERT_EQ(context->get_num_input_ports(), 1);

  TestInputAndOutput<T>(saturation_system, std::move(context), input_vector,
                        expected_output);
}

template <typename T>
void TestVariableSaturation(const Saturation<T>& saturation_system,
                            const VectorX<T>& min_value_vector,
                            const VectorX<T>& max_value_vector,
                            const VectorX<T>& input_vector,
                            const VectorX<T>& expected_output) {
  auto context = saturation_system.CreateDefaultContext();

  const int port_size = saturation_system.get_size();

  // Checks that the number of input ports in the Saturation system and the
  // Context are consistent.
  ASSERT_EQ(saturation_system.get_num_input_ports(), 3);
  ASSERT_EQ(context->get_num_input_ports(), 3);

  // Applies the min and max values as inputs to the context.
  if (min_value_vector.size() > 0) {
    auto min_value = std::make_unique<BasicVector<T>>(port_size);
    min_value->get_mutable_value() << min_value_vector;
    // Hook min value of the expected size.
    context->FixInputPort(saturation_system.get_min_value_port().get_index(),
                          std::move(min_value));
  }

  if (max_value_vector.size()) {
    auto max_value = std::make_unique<BasicVector<T>>(port_size);
    max_value->get_mutable_value() << max_value_vector;
    // Hook max value of the expected size.
    context->FixInputPort(saturation_system.get_max_value_port().get_index(),
                          std::move(max_value));
  }

  TestInputAndOutput<T>(saturation_system, std::move(context), input_vector,
                        expected_output);
}

// Sets up a test over a range of inputs.
template <typename T>
void SaturationTest(bool run_constant_saturation_test) {
  // Tests for error thrown due to incorrectly initialized Saturation. (Both
  // max and min value ports are disabled).
  EXPECT_ANY_THROW(std::make_unique<Saturation<T>>(0 /* input_size */));

  // Tests for error thrown due to incorrectly initialized Saturation. (u_min
  // and u_max have unequal lengths).
  EXPECT_ANY_THROW(
      std::make_unique<Saturation<T>>(Vector3<T>(1.0, -4.5, -2.5) /* u_min */,
                                      Vector2<T>(3.0, 5.0) /* u_max */));

  // Tests for error thrown due to incorrectly initialized Saturation. (u_min
  // >= u_max along some or all dimensions).
  EXPECT_ANY_THROW(
      std::make_unique<Saturation<T>>(Vector3<T>(1.0, -4.5, -2.5) /* u_min */,
                                      Vector3<T>(0.75, 5.0, 2.0) /* u_max */));

  // Arbitrary choice of limits for the test.
  Vector4<T> kUMax, kUMin;
  kUMin << -0.3, 0.0, 1.3, -4.0;
  kUMax << 1.0, 2.5, 3.3, 2.5;

  std::unique_ptr<Saturation<T>> saturation_system;

  if (run_constant_saturation_test) {
    saturation_system = std::make_unique<Saturation<T>>(kUMin, kUMax);
  } else {
    saturation_system = std::make_unique<Saturation<T>>(4 /* input_size */);
  }

  const int kNumPoints = 10;

  // Since kUMin and kUMax are of dimension 4 for this test, the inputs to be
  // tested must be of dimension 4. Similar to the SaturationScalarTest, the
  // following logic creates a uniformly spaced range of input values
  // replicated across each of the four dimensions. This leads to a total of
  // kNumPoints test cases.
  Eigen::MatrixXd input_vector_range =
      Eigen::VectorXd::LinSpaced(kNumPoints, -5.0 /* lower limit */,
                                 5.0 /* upper limit */)
          .replicate(1, 4 /* input_size */)
          .transpose();

  for (int i = 0; i < kNumPoints; ++i) {
    Vector4<T> expected = input_vector_range.col(i);

    for (int j = 0; j < 4; ++j) {
      if (expected[j] < kUMin[j]) {
        expected[j] = kUMin[j];
      } else if (expected[j] > kUMax[j]) {
        expected[j] = kUMax[j];
      }
    }

    if (run_constant_saturation_test) {
      TestConstantSaturation<T>(
          *saturation_system, input_vector_range.col(i), expected);
    } else {
      // Tests Saturation with variable max and min values.
      TestVariableSaturation<T>(*saturation_system, kUMin, kUMax,
                                    input_vector_range.col(i), expected);

      VectorX<T> dummy_vector;
      // Tests Saturation with variable max and min values and neither supplied.
      // This results in the system acting as a pass through system.
      EXPECT_ANY_THROW(TestVariableSaturation<T>(
          *saturation_system, dummy_vector, dummy_vector,
          input_vector_range.col(i), input_vector_range.col(i)));
    }
  }
}

// Tests the ability to use double vectors for inputs, and limits and constant
// lower and upper limits.
GTEST_TEST(VariableSaturationTest, ConstantDoubleTest) {
  EXPECT_NO_FATAL_FAILURE(SaturationTest<double>(true));
}

// Tests the ability to use double vectors for inputs, and limits and variable
// lower and upper limits.
GTEST_TEST(VariableSaturationTest, VariableDoubleTest) {
  EXPECT_NO_FATAL_FAILURE(SaturationTest<double>(false));
}

// Tests the ability to use AutoDiff vectors for inputs, and limits and
// constant lower and upper limits.
GTEST_TEST(VariableSaturationTest, ConstantAutoDiffTest) {
  EXPECT_NO_FATAL_FAILURE(SaturationTest<AutoDiffXd>(true));
}

// Tests the ability to use AutoDiff vectors for inputs, and limits and
// variable lower and upper limits.
GTEST_TEST(VariableSaturationTest, VariableAutoDiffTest) {
  EXPECT_NO_FATAL_FAILURE(SaturationTest<AutoDiffXd>(false));
}

}  // namespace
}  // namespace systems
}  // namespace drake
