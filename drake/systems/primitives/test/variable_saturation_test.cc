#include "drake/systems/primitives/variable_saturation.h"

#include <memory>

#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"

namespace drake {
namespace systems {
namespace {

template <typename T>
void TestInputAndOutput(const VariableSaturation<T>& variable_saturation_system,
                        std::unique_ptr<Context<T>> context,
                        const VectorX<T>& input_vector,
                        const VectorX<T>& expected_output) {
  const int kPortSize = variable_saturation_system.get_size();

  // Verifies that Saturation allocates no state variables in the context.
  EXPECT_EQ(context->get_continuous_state()->size(), 0);

  // Verifies that Saturation allocates no state variables in the context.
  EXPECT_EQ(context->get_continuous_state()->size(), 0);
  auto output = variable_saturation_system.AllocateOutput(*context);
  auto input = std::make_unique<BasicVector<T>>(kPortSize);

  input->get_mutable_value() << input_vector;

  // Hook input of the expected size.
  context->FixInputPort(variable_saturation_system.get_input_port_index(),
                        std::move(input));

  variable_saturation_system.CalcOutput(*context, output.get());

  // Checks that the number of output ports in the Saturation system and the
  // SystemOutput are consistent.
  ASSERT_EQ(output->get_num_ports(), 1);
  ASSERT_EQ(variable_saturation_system.get_num_output_ports(), 1);
  const BasicVector<T>* output_vector = output->get_vector_data(0);
  ASSERT_NE(output_vector, nullptr);
  EXPECT_EQ(output_vector->get_value(), expected_output);
}

template <typename T>
void TestVariableSaturationSystemWithMinValue(
    const VariableSaturation<T>& variable_saturation_system,
    const VectorX<T>& min_value_vector, const VectorX<T>& input_vector,
    const VectorX<T>& expected_output) {
  auto context = variable_saturation_system.CreateDefaultContext();

  const int kPortSize = variable_saturation_system.get_size();

  auto min_value = std::make_unique<BasicVector<T>>(kPortSize);

  // Checks that the number of input ports in the Saturation system and the
  // Context are consistent.
  ASSERT_EQ(variable_saturation_system.get_num_input_ports(), 2);
  ASSERT_EQ(context->get_num_input_ports(), 2);

  min_value->get_mutable_value() << min_value_vector;

  // Hook input of the expected size.
  context->FixInputPort(variable_saturation_system.get_min_value_port_index(),
                        std::move(min_value));

  TestInputAndOutput(variable_saturation_system, std::move(context),
                     input_vector, expected_output);
}

template <typename T>
void TestVariableSaturationSystemWithMaxValue(
    const VariableSaturation<T>& variable_saturation_system,
    const VectorX<T>& max_value_vector, const VectorX<T>& input_vector,
    const VectorX<T>& expected_output) {
  auto context = variable_saturation_system.CreateDefaultContext();

  const int kPortSize = variable_saturation_system.get_size();

  auto max_value = std::make_unique<BasicVector<T>>(kPortSize);

  // Checks that the number of input ports in the Saturation system and the
  // Context are consistent.
  ASSERT_EQ(variable_saturation_system.get_num_input_ports(), 2);
  ASSERT_EQ(context->get_num_input_ports(), 2);

  max_value->get_mutable_value() << max_value_vector;

  // Hook input of the expected size.
  context->FixInputPort(variable_saturation_system.get_max_value_port_index(),
                        std::move(max_value));

  TestInputAndOutput(variable_saturation_system, std::move(context),
                     input_vector, expected_output);
}

template <typename T>
void TestVariableSaturationSystemWithMinAndMaxValue(
    const VariableSaturation<T>& variable_saturation_system,
    const VectorX<T>& max_value_vector, const VectorX<T>& min_value_vector,
    const VectorX<T>& input_vector, const VectorX<T>& expected_output) {
  auto context = variable_saturation_system.CreateDefaultContext();

  const int kPortSize = variable_saturation_system.get_size();

  auto min_value = std::make_unique<BasicVector<T>>(kPortSize);
  auto max_value = std::make_unique<BasicVector<T>>(kPortSize);

  // Checks that the number of input ports in the VariableSaturation system and
  // the Context are consistent.
  ASSERT_EQ(variable_saturation_system.get_num_input_ports(), 3);
  ASSERT_EQ(context->get_num_input_ports(), 3);

  min_value->get_mutable_value() << min_value_vector;
  max_value->get_mutable_value() << max_value_vector;

  // Hook input of the expected size.
  context->FixInputPort(variable_saturation_system.get_min_value_port_index(),
                        std::move(min_value));
  context->FixInputPort(variable_saturation_system.get_max_value_port_index(),
                        std::move(max_value));

  TestInputAndOutput(variable_saturation_system, std::move(context),
                     input_vector, expected_output);
}

// Sets up a test over a range of inputs.
template <typename T>
void VariableSaturationTest(bool enable_max_value_test,
                            bool enable_min_value_test) {
  // Tests for error thrown due to incorrectly initialized Saturation. (Both
  // max and min value ports are disabled).
  EXPECT_ANY_THROW(std::make_unique<VariableSaturation<T>>(
      false /* max_value_port_active */, false /* min_value_port_active */,
      3 /* input_size */));

  // Tests for error thrown due to incorrectly initialized Saturation. (Input
  // size is 0).
  EXPECT_ANY_THROW(std::make_unique<VariableSaturation<T>>(
      false /* max_value_port_enabled */, true /* min_value_port_enabled */,
      0 /* input_size */));

  // Arbitrary choice of limits for the test.
  Vector4<T> kUMax, kUMin;
  if (enable_min_value_test) {
    kUMin << -0.3, 0.0, 1.3, -4.0;
  } else {
    kUMin << Vector4<T>::Constant(4 /* input_size */,
                                  -std::numeric_limits<double>::infinity());
  }
  if (enable_max_value_test) {
    kUMax << 1.0, 2.5, 3.3, 2.5;
  } else {
    kUMax << Vector4<T>::Constant(4 /* input_size */,
                                  std::numeric_limits<double>::infinity());
  }

  const auto variable_saturation_system =
      std::make_unique<VariableSaturation<T>>(
          enable_max_value_test, enable_min_value_test, 4 /* input_size */);

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

    if (enable_max_value_test && enable_min_value_test) {
      EXPECT_NO_THROW(TestVariableSaturationSystemWithMinAndMaxValue<T>(
          *variable_saturation_system, kUMax, kUMin, input_vector_range.col(i),
          expected));
    } else if (enable_max_value_test) {
      EXPECT_NO_THROW(TestVariableSaturationSystemWithMaxValue<T>(
          *variable_saturation_system, kUMax, input_vector_range.col(i),
          expected));
    } else {
      EXPECT_NO_THROW(TestVariableSaturationSystemWithMinValue<T>(
          *variable_saturation_system, kUMin, input_vector_range.col(i),
          expected));
    }
  }
}

// Tests the ability to use double vectors for the lower saturation limits.
GTEST_TEST(VariableSaturationTest, MinDoubleTest) {
  EXPECT_NO_FATAL_FAILURE(VariableSaturationTest<double>(true, false));
}

// Tests the ability to use double vectors for the upper saturation limits.
GTEST_TEST(VariableSaturationTest, MaxDoubleTest) {
  EXPECT_NO_FATAL_FAILURE(VariableSaturationTest<double>(false, true));
}

// Tests the ability to use double vectors for the lower and upper
// saturation limits.
GTEST_TEST(VariableSaturationTest, MinMaxDoubleTest) {
  EXPECT_NO_FATAL_FAILURE(VariableSaturationTest<double>(true, true));
}

// Tests the ability to use AutoDiffXd vectors for the lower saturation limits.
GTEST_TEST(VariableSaturationTest, MinAutodiffTest) {
  EXPECT_NO_FATAL_FAILURE(VariableSaturationTest<AutoDiffXd>(true, false));
}
// Tests the ability to use AutoDiffXd vectors for the lower
// saturation limits.
GTEST_TEST(VariableSaturationTest, MaxAutodiffTest) {
  EXPECT_NO_FATAL_FAILURE(VariableSaturationTest<AutoDiffXd>(false, true));
}
// Tests the ability to use AutoDiffXd vectors for the lower and upper
// saturation limits.
GTEST_TEST(VariableSaturationTest, MinMaxAutodiffTest) {
  EXPECT_NO_FATAL_FAILURE(VariableSaturationTest<AutoDiffXd>(true, true));
}

}  // namespace
}  // namespace systems
}  // namespace drake
