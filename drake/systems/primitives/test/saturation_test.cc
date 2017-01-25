#include "drake/systems/primitives/saturation.h"

#include <memory>

#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"


namespace drake {
namespace systems {
namespace {

template <typename T>
void TestSaturationSystem(const Saturation<T>& saturation_system,
                          const VectorX<T>& input_vector,
                          const VectorX<T>& expected_output) {
  auto context = saturation_system.CreateDefaultContext();

  // Verifies that Saturation allocates no state variables in the context.
  EXPECT_EQ(context->get_continuous_state()->size(), 0);
  auto output = saturation_system.AllocateOutput(*context);
  auto input = std::make_unique<BasicVector<T>>(
      saturation_system.u_min().size());

  // Checks that the number of input ports in the Saturation system and the
  // Context are consistent.
  ASSERT_EQ(saturation_system.get_num_input_ports(), 1);
  ASSERT_EQ(context->get_num_input_ports(), 1);

  input->get_mutable_value() << input_vector;

  // Hook input of the expected size.
  context->FixInputPort(0, std::move(input));

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
void TestSaturationSystem(const Saturation<T>& saturation_system,
                          const T& input_scalar,
                          const T& expected_scalar) {
  TestSaturationSystem<T>(saturation_system,
                       VectorX<T>::Constant(1, input_scalar),
                          VectorX<T>::Constant(1, expected_scalar));
}

template <typename T>
void SaturationScalarTest() {
  // Tests for error thrown due to incorrectly initialized Saturation
  // (u_min > u_max).
  EXPECT_ANY_THROW(
      std::make_unique<Saturation<T>>(1.0 /* u_min */, -0.9 /* u_max */));

  const double kUMin = -0.4;
  const double kUMax = 1.8;
  const auto saturation_system =
      std::make_unique<Saturation<T>>(kUMin, kUMax);

  // Checks the getters.
  EXPECT_EQ(kUMin, saturation_system->get_u_min_scalar());
  EXPECT_EQ(kUMax, saturation_system->get_u_max_scalar());

  const int kNumPoints = 20;

  // Obtains a range of linear spaced input values. This results in a total of
  // kNumPoints test cases.
  Eigen::VectorXd input_eigen_vector_range = Eigen::VectorXd::LinSpaced(
      kNumPoints /* size */, -2.5 /* lower limit */, 2.5 /* upper limit */);

  // Converts range of inputs into a vector of doubles.
  std::vector<T> input_vector_range(input_eigen_vector_range.size());
  Eigen::Map<VectorX<T>>(input_vector_range.data(),
                              input_eigen_vector_range.rows()) =
      input_eigen_vector_range;

  for (int i = 0; i < kNumPoints; ++i) {
    T expected = input_vector_range.at(i);

    if (expected < kUMin) {
      expected = kUMin;
    } else if (expected > kUMax) {
      expected = kUMax;
    }

    EXPECT_NO_THROW(TestSaturationSystem<T>(
        *saturation_system, input_vector_range.at(i), expected));
  }
}

// Tests the ability to use Scalar doubles as the saturation limits.
GTEST_TEST(SaturationTest, SaturationScalarDoubleTest) {
  EXPECT_NO_FATAL_FAILURE(SaturationScalarTest<double>());
}

// Tests the ability to use Scalar AutoDiffXd as the saturation limits.
GTEST_TEST(SaturationTest, SaturationScalarAutoDiffTest) {
  EXPECT_NO_FATAL_FAILURE(SaturationScalarTest<AutoDiffXd>());
}

template <typename T>
void SaturationVectorTest() {
  // Tests for error thrown due to incorrectly initialized Saturation. (u_min
  // and u_max have unequal lengths).
  EXPECT_ANY_THROW(std::make_unique<Saturation<T>>(
      Vector3<T>(1.0, -4.5, -2.5) /* u_min */,
      Vector2<T>(3.0, 5.0) /* u_max */));

  // Tests for error thrown due to incorrectly initialized Saturation. (u_min
  // >= u_max along some or all dimensions).
  EXPECT_ANY_THROW(std::make_unique<Saturation<T>>(
      Vector3<T>(1.0, -4.5, -2.5) /* u_min */,
      Vector3<T>(0.75, 5.0, 2.0) /* u_max */));

  // Arbitrary choice of limits for the test.
  const Vector4<T> kUMin(1.0, 2.5, 3.3, 2.5);
  const Vector4<T> kUMax(-0.3, 0.0, 1.3, -4.0);

  const auto saturation_system =
      std::make_unique<Saturation<T>>(kUMax, kUMin);

  // Tests for error thrown due to calling the scalar getters.
  EXPECT_ANY_THROW(saturation_system->get_u_max_scalar());
  EXPECT_ANY_THROW(saturation_system->get_u_min_scalar());

  const int kNumPoints = 20;

  // Since kUMin and kUMax are of dimension 4 for this test, the inputs to be
  // tested must be of dimension 4. Similar to the SaturationScalarTest, the
  // following logic creates a uniformly spaced range of input values
  // replicated across each of the four dimensions. This leads to a total of
  // kNumPoints test cases.
  Eigen::MatrixXd input_vector_range =
      Eigen::VectorXd::LinSpaced(kNumPoints, -5.0 /* lower limit */,
                                 5.0 /* upper limit */)
          .replicate(1, 4)
          .transpose();

  for (int i = 0; i < kNumPoints; ++i) {
    Vector4<T> expected = input_vector_range.col(i);

    for (int j = 0; j < 4; ++j) {
      if (expected[j] < kUMax[j]) {
        expected[j] = kUMax[j];
      } else if (expected[j] > kUMin[j]) {
        expected[j] = kUMin[j];
      }
    }

    EXPECT_NO_THROW(TestSaturationSystem<T>(
        *saturation_system, input_vector_range.col(i), expected));
  }
}

// Tests the ability to use double vectors for the lower and upper
// saturation limits.
GTEST_TEST(SaturationTest, SaturationVectorDoubleTest) {
  EXPECT_NO_FATAL_FAILURE(SaturationVectorTest<double>());
}

// Tests the ability to use AutoDiffXd vectors for the lower and upper
// saturation limits.
GTEST_TEST(SaturationTest, SaturationVectorAutoDiffTest) {
  EXPECT_NO_FATAL_FAILURE(SaturationVectorTest<AutoDiffXd>());
}

}  // namespace
}  // namespace systems
}  // namespace drake
