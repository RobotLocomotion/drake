#include "gtest/gtest.h"

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"
#include "drake/systems/primitives/saturation.h"

namespace drake {
namespace systems {
namespace {

template <typename T>
void TestSaturationSystem(const Saturation<T>& saturation_system,
                          const VectorX<double>& input_vector,
                          const VectorX<double>& expected_output) {
  auto context = saturation_system.CreateDefaultContext();

  // Verifies that Saturation allocates no state variables in the context.
  EXPECT_EQ(context->get_continuous_state()->size(), 0);
  auto output = saturation_system.AllocateOutput(*context);
  auto input = std::make_unique<BasicVector<double>>(
      saturation_system.get_u_min_vector().size());

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
  const BasicVector<double>* output_vector = output->get_vector_data(0);
  ASSERT_NE(output_vector, nullptr);
  EXPECT_EQ(output_vector->get_value(), expected_output);
}

template <typename T>
void TestSaturationSystem(const Saturation<T>& saturation_system,
                          const double& input_scalar,
                          const double& expected_scalar) {
  TestSaturationSystem(saturation_system,
                       Eigen::VectorXd::Constant(1, input_scalar),
                       Eigen::VectorXd::Constant(1, expected_scalar));
}

// Tests the ability to use doubles as the saturation limits.
GTEST_TEST(SaturationTest, SaturationScalarTest) {
  // Test for death of an incorrectly initialised Saturation (u_min > u_max).
  EXPECT_DEATH(
      std::make_unique<Saturation<double>>(1.0 /* u_min */, -0.9 /* u_max */),
      "");

  const double kUMin = -0.4;
  const double kUMax = 1.8;
  const auto saturation_system =
      std::make_unique<Saturation<double>>(kUMin, kUMax);

  // Checks the getters.
  EXPECT_EQ(kUMin, saturation_system->get_u_min());
  EXPECT_EQ(kUMax, saturation_system->get_u_max());

  const int kNumPoints = 20;

  // Obtains a range of linear spaced input values. This results in a total of
  // kNumPoints test cases.
  Eigen::VectorXd input_eigen_vector_range = Eigen::VectorXd::LinSpaced(
      kNumPoints /* size */, -2.5 /* lower limit */, 2.5 /* upper limit */);

  // Converts range of inputs into a vector of doubles.
  std::vector<double> input_vector_range(input_eigen_vector_range.size());
  Eigen::Map<Eigen::VectorXd>(input_vector_range.data(),
                              input_eigen_vector_range.rows()) =
      input_eigen_vector_range;

  for (int i = 0; i < kNumPoints; ++i) {
    double expected = input_vector_range.at(i);

    if (expected < kUMin) {
      expected = kUMin;
    } else if (expected > kUMax) {
      expected = kUMax;
    }

    EXPECT_NO_THROW(TestSaturationSystem<double>(
        *saturation_system, input_vector_range.at(i), expected));
  }
}

// Tests the ability to use vectors for the lower and upper saturation limits.
GTEST_TEST(SaturationTest, SaturationVectorTest) {
  // Test for death of an incorrectly initialised Saturation. (u_min and
  // u_max are of incorrect length).
  EXPECT_DEATH(std::make_unique<Saturation<double>>(
                   Vector3<double>(1.0, -4.5, -2.5) /* u_min */,
                   Vector2<double>(3.0, 5.0) /* u_max */),
               "");

  // Arbitrary choice of limits for the test.
  const Vector4<double> kUMin(1.0, 2.5, 3.3, 2.5);
  const Vector4<double> kUMax(-0.3, 0.0, 1.3, -4.0);

  const auto saturation_system =
      std::make_unique<Saturation<double>>(kUMax, kUMin);

  // Test for the death due to calling the scalar getters.
  EXPECT_DEATH(saturation_system->get_u_max(), "");
  EXPECT_DEATH(saturation_system->get_u_min(), "");

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
    Vector4<double> expected = input_vector_range.col(i);

    for (int j = 0; j < 4; ++j) {
      if (expected[j] < kUMax[j]) {
        expected[j] = kUMax[j];
      } else if (expected[j] > kUMin[j]) {
        expected[j] = kUMin[j];
      }
    }

    EXPECT_NO_THROW(TestSaturationSystem<double>(
        *saturation_system, input_vector_range.col(i), expected));
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
