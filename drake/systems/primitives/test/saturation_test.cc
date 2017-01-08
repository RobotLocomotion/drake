#include "drake/systems/primitives/saturation.h"

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"

#include "gtest/gtest.h"

namespace drake {
namespace systems {
namespace {

template <typename T>
void TestSaturationSystem(const Saturation<T>& saturation_system,
                          const VectorX<double>& input_vector,
                          const VectorX<double>& expected_output) {
  auto context = saturation_system.CreateDefaultContext();

  // Verifies that Saturation allocates no state variables in the context.
  EXPECT_EQ(0, context->get_continuous_state()->size());
  auto output = saturation_system.AllocateOutput(*context);
  auto input = std::make_unique<BasicVector<double>>(
      saturation_system.get_sigma_lower_vector().size());

  // Checks that the number of input ports in the Saturation system and the
  // Context are consistent.
  ASSERT_EQ(1, saturation_system.get_num_input_ports());
  ASSERT_EQ(1, context->get_num_input_ports());

  input->get_mutable_value() << input_vector;

  // Hook input of the expected size.
  context->FixInputPort(0, std::move(input));

  saturation_system.CalcOutput(*context, output.get());

  // Checks that the number of output ports in the Saturation system and the
  // SystemOutput are consistent.
  ASSERT_EQ(1, output->get_num_ports());
  ASSERT_EQ(1, saturation_system.get_num_output_ports());
  const BasicVector<double>* output_vector = output->get_vector_data(0);
  ASSERT_NE(nullptr, output_vector);
  EXPECT_EQ(expected_output, output_vector->get_value());
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
  const double kSigma_l = -0.4;
  const double kSigma_u = 1.8;
  const auto saturation_system =
      std::make_unique<Saturation<double>>(kSigma_l, kSigma_u);

  // Checks the getters.
  EXPECT_EQ(kSigma_l, saturation_system->get_sigma_lower());
  EXPECT_EQ(kSigma_u, saturation_system->get_sigma_upper());

  const int kNumPoints = 20;

  // Obtains a range of linear spaced input values.
  Eigen::VectorXd input_eigen_vector_range = Eigen::VectorXd::LinSpaced(
      kNumPoints /* size */, -2.5 /* lower limit */, +2.5 /* upper limit */);

  // Converts range of inputs into a vector of doubles.
  std::vector<double> input_vector_range(input_eigen_vector_range.size());
  Eigen::Map<Eigen::VectorXd>(input_vector_range.data(),
                              input_eigen_vector_range.rows()) =
      input_eigen_vector_range;

  for (int i = 0; i < kNumPoints; ++i) {
    double expected = input_vector_range.at(i);

    if (expected < kSigma_l) {
      expected = kSigma_l;
    } else if (expected > kSigma_u) {
      expected = kSigma_u;
    }

    EXPECT_NO_THROW(TestSaturationSystem<double>(
        *saturation_system, input_vector_range.at(i), expected));
  }
}

// Tests the ability to use vectors for the lower and upper saturation limits.
GTEST_TEST(SaturationTest, SaturationVectorTest) {
  // Arbitrary choice of limits for the test.
  const Vector4<double> sigma_upper(1.0, 2.5, 3.3, 2.5);
  const Vector4<double> sigma_lower(-0.3, 0.0, 1.3, -4.0);

  const auto saturation_system =
      std::make_unique<Saturation<double>>(sigma_lower, sigma_upper);

  const int kNumPoints = 20;

  // Creates a range of inputs that is replicated across all four dimensions.
  Eigen::MatrixXd input_vector_range =
      Eigen::VectorXd::LinSpaced(kNumPoints, -5.0 /* lower limit */,
                                 5.0 /* upper limit */)
          .replicate(1, 4)
          .transpose();

  for (int i = 0; i < kNumPoints; ++i) {
    Vector4<double> expected = input_vector_range.col(i);

    for (int j = 0; j < 4; ++j) {
      if (expected[j] < sigma_lower[j]) {
        expected[j] = sigma_lower[j];
      } else if (expected[j] > sigma_upper[j]) {
        expected[j] = sigma_upper[j];
      }
    }

    EXPECT_NO_THROW(TestSaturationSystem<double>(
        *saturation_system, input_vector_range.col(i), expected));
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
