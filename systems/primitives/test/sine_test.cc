#include "drake/systems/primitives/sine.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

using Eigen::Vector3d;
using std::make_unique;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace {

template <typename T>
void TestSineSystem(const Sine<T>& sine_system,
                    const Eigen::MatrixXd& input_vectors,
                    const Eigen::MatrixXd& expected_outputs,
                    const Eigen::MatrixXd& expected_first_derivs,
                    const Eigen::MatrixXd& expected_second_derivs) {
  auto context = sine_system.CreateDefaultContext();

  // The tolerance used for testing the system outputs. System inputs are
  // specified to four significant digits, and the tolerance here is chosen to
  // match this precision.
  T ktest_tolerance = 1e-4;

  // Verifies that Sine allocates no state variables in the context.
  EXPECT_EQ(0, context->num_continuous_states());

  if (sine_system.is_time_based()) {
    // If the system is time based, the input_vectors Matrix should only contain
    // a row vector of time instances to test against. In this case, the system
    // has zero inputs ports.
    ASSERT_EQ(input_vectors.rows(), 1);
    ASSERT_EQ(0, sine_system.num_input_ports());
    ASSERT_EQ(0, context->num_input_ports());

    for (int i = 0; i < input_vectors.cols(); i++) {
      // Initialize the time in seconds to be used by the Sine system.
      context->SetTime(input_vectors(0, i));

      // Check the Sine output.
      ASSERT_EQ(3, sine_system.num_output_ports());
      EXPECT_TRUE(CompareMatrices(
          sine_system.get_output_port(0).Eval(*context),
          expected_outputs.col(i), ktest_tolerance));
      EXPECT_TRUE(CompareMatrices(
          sine_system.get_output_port(1).Eval(*context),
          expected_first_derivs.col(i), ktest_tolerance));
      EXPECT_TRUE(CompareMatrices(
          sine_system.get_output_port(2).Eval(*context),
          expected_second_derivs.col(i), ktest_tolerance));
    }
  } else {
    // Loop over the input vectors and check that the Sine system outputs match
    // the expected outputs.
    for (int i = 0; i < input_vectors.cols(); i++) {
      ASSERT_EQ(1, sine_system.num_input_ports());
      ASSERT_EQ(1, context->num_input_ports());

      // Confirm the size of the input port matches the size of the sample
      // inputs (i.e., each column in the input_vectors matrix represents a
      // sampling instant. The number of rows in input_vectors should match the
      // size of the input port).
      ASSERT_EQ(sine_system.amplitude_vector().size(), input_vectors.rows());

      // Initialize the input and associate it with the context.
      sine_system.get_input_port(0).FixValue(context.get(),
                                             input_vectors.col(i));

      // Check the Sine output.
      ASSERT_EQ(3, sine_system.num_output_ports());
      EXPECT_TRUE(CompareMatrices(
          sine_system.get_output_port(0).Eval(*context),
          expected_outputs.col(i), ktest_tolerance));
      EXPECT_TRUE(CompareMatrices(
          sine_system.get_output_port(1).Eval(*context),
          expected_first_derivs.col(i), ktest_tolerance));
      EXPECT_TRUE(CompareMatrices(
          sine_system.get_output_port(2).Eval(*context),
          expected_second_derivs.col(i), ktest_tolerance));
    }
  }
}

// Test with scalar input instances and scalar Sine parameters.
GTEST_TEST(SineTest, SineScalarTest) {
  const double kAmp = 1;
  const double kPhase = 2;
  const double kFreq = 3;
  const auto sine_system =
      make_unique<Sine<double>>(kAmp, kFreq, kPhase, 1, false);

  const Eigen::Vector4d input_vector(M_PI / 6.0, M_PI / 4.0, M_PI / 2.0, M_PI);
  const Eigen::Vector4d expected_output(-0.4161, -0.9372, 0.4161, -0.9093);
  const Eigen::Vector4d expected_first_deriv(-2.7279, -1.0461, 2.7279, 1.2484);
  const Eigen::Vector4d expected_second_deriv(3.7453, 8.4351, -3.7453, 8.1837);

  TestSineSystem(
      *sine_system, input_vector.transpose(), expected_output.transpose(),
      expected_first_deriv.transpose(), expected_second_deriv.transpose());
}

// Test with time based instances and scalar Sine parameters.
GTEST_TEST(SineTest, SineScalarTimeTest) {
  const double kAmp = 1;
  const double kPhase = 2;
  const double kFreq = 3;
  const auto sine_system =
      make_unique<Sine<double>>(kAmp, kFreq, kPhase, 1, true);

  const Eigen::Vector4d input_vector(M_PI / 6.0, M_PI / 4.0, M_PI / 2.0, M_PI);
  const Eigen::Vector4d expected_output(-0.4161, -0.9372, 0.4161, -0.9093);
  const Eigen::Vector4d expected_first_deriv(-2.7279, -1.0461, 2.7279, 1.2484);
  const Eigen::Vector4d expected_second_deriv(3.7453, 8.4351, -3.7453, 8.1837);

  TestSineSystem(
      *sine_system, input_vector.transpose(), expected_output.transpose(),
      expected_first_deriv.transpose(), expected_second_deriv.transpose());
}

// Test with vector input instances and scalar Sine parameters.
GTEST_TEST(SineTest, SineVectorTest) {
  const double kAmp = 1;
  const double kPhase = 2;
  const double kFreq = 3;
  const auto sine_system =
      make_unique<Sine<double>>(kAmp, kFreq, kPhase, 4, false);

  Eigen::Matrix4d input_vectors;
  input_vectors << 0.1, 0.2, 0.3, 0.4,
                   0.5, 0.6, 0.7, 0.8,
                   0.9, 1.0, 1.1, 1.2,
                   1.3, 1.4, 1.5, 1.6;

  Eigen::Matrix4d expected_output;
  expected_output << 0.7457, 0.5155, 0.2392, -0.0584,
                     -0.3508, -0.6119, -0.8183, -0.9516,
                     -0.9999, -0.9589, -0.8323, -0.6313,
                     -0.3739, -0.0831, 0.2151, 0.4941;

  Eigen::Matrix4d expected_first_deriv;
  expected_first_deriv << -1.9988, -2.5707, -2.9129, -2.9949,
                          -2.8094, -2.3729, -1.7245, -0.9220,
                          -0.0372, 0.8510, 1.6631, 2.3267,
                          2.7824, 2.9896, 2.9298, 2.6082;

  Eigen::Matrix4d expected_second_deriv;
  expected_second_deriv << -6.7113, -4.6395, -2.1532, 0.5254,
                           3.1570, 5.5067, 7.3645, 8.5644,
                           8.9993, 8.6303, 7.4904, 5.6814,
                           3.3649, 0.7478, -1.9361, -4.4470;

  TestSineSystem(*sine_system, input_vectors, expected_output,
                 expected_first_deriv, expected_second_deriv);
}

// Test with vector input instances and vector Sine parameters.
GTEST_TEST(SineTest, SineParameterTest) {
  Eigen::Vector4d kAmp(1.1, 1.2, 1.3, 1.4);
  Eigen::Vector4d kFreq(1.5, 1.6, 1.7, 1.8);
  Eigen::Vector4d kPhase(1.9, 2.0, 2.1, 2.2);
  const auto sine_system =
      make_unique<Sine<double>>(kAmp, kFreq, kPhase, false);

  Eigen::Matrix4d input_vectors;
  input_vectors << 0.1, 0.2, 0.3, 0.4,
                   0.5, 0.6, 0.7, 0.8,
                   0.9, 1.0, 1.1, 1.2,
                   1.3, 1.4, 1.5, 1.6;
  Eigen::Matrix4d expected_output;
  expected_output << 0.9761, 0.8893, 0.7826, 0.6583,
                     0.4020, 0.2167, 0.0259, -0.1656,
                     -0.6100, -0.7954, -0.9579, -1.0928,
                     -1.3792, -1.4000, -1.3754, -1.3065;

  Eigen::Matrix4d expected_first_deriv;
  expected_first_deriv << -0.7608, -0.9710, -1.1595, -1.3219,
                          -1.8091, -1.8884, -1.9196, -1.9016,
                          -1.9516, -1.7480, -1.4941, -1.1970,
                          -0.4323, 0.0192, 0.4700, 0.9057;

  Eigen::Matrix4d expected_second_deriv;
  expected_second_deriv << -2.1962, -2.0010, -1.7609, -1.4812,
                           -1.0291, -0.5548, -0.0663, 0.4238,
                           1.7629, 2.2988, 2.7684, 3.1582,
                           4.4688, 4.5359, 4.4564, 4.2329;

  TestSineSystem(*sine_system, input_vectors, expected_output,
                 expected_first_deriv, expected_second_deriv);
}

// Test with time instances, and vector Sine parameters.
GTEST_TEST(SineTest, SineParameterTimeTest) {
  Eigen::Vector4d kAmp(1.1, 1.2, 1.3, 1.4);
  Eigen::Vector4d kFreq(1.5, 1.6, 1.7, 1.8);
  Eigen::Vector4d kPhase(1.9, 2.0, 2.1, 2.2);
  const auto sine_system =
      make_unique<Sine<double>>(kAmp, kFreq, kPhase, true);

  Eigen::Vector4d input_vectors(0.1, 0.6, 1.1, 1.5);
  Eigen::Matrix4d expected_output;
  expected_output << 0.9761, 0.3685, -0.4369, -0.9306,
                     0.9977, 0.2167, -0.6957, -1.1419,
                     0.9950, 0.0281, -0.9579, -1.2975,
                     0.9661, -0.1932, -1.2062, -1.3754;

  Eigen::Matrix4d expected_first_deriv;
  expected_first_deriv << -0.7608, -1.5547, -1.5143, -0.8798,
                          -1.0669, -1.8884, -1.5644, -0.5901,
                          -1.4224, -2.2095, -1.4941, -0.1378,
                          -1.8238, -2.4959, -1.2791, 0.4700;

  Eigen::Matrix4d expected_second_deriv;
  expected_second_deriv << -2.1962, -0.8291, 0.9829, 2.0938,
                           -2.5540, -0.5548, 1.7810, 2.9233,
                           -2.8754, -0.0811, 2.7684, 3.7497,
                           -3.1302, 0.6258, 3.9082, 4.4564;

  TestSineSystem(*sine_system, input_vectors.transpose(), expected_output,
                 expected_first_deriv, expected_second_deriv);
}

GTEST_TEST(SineTest, BadSizeTest) {
  Eigen::Vector4d kAmp(1.1, 1.2, 1.3, 1.4);
  Eigen::Vector4d kFreq(1.5, 1.6, 1.7, 1.8);
  Eigen::Vector3d kPhase(1.9, 2.0, 2.1);
  DRAKE_EXPECT_THROWS_MESSAGE(
      Sine<double>(kAmp, kFreq, kPhase, true),
      ".*amplitudes.*==.*phases.*");
}

GTEST_TEST(SineTest, SineAccessorTest) {
  Eigen::Vector4d kAmp(1.1, 1.2, 1.3, 1.4);
  Eigen::Vector4d kFreq(1.5, 1.6, 1.7, 1.8);
  Eigen::Vector4d kPhase(1.9, 2.0, 2.1, 2.2);
  const auto sine_system = make_unique<Sine<double>>(kAmp, kFreq, kPhase, true);
  // Verifies the Sine accessors are OK.
  EXPECT_THROW(sine_system->amplitude(), std::logic_error);
  EXPECT_THROW(sine_system->frequency(), std::logic_error);
  EXPECT_THROW(sine_system->phase(), std::logic_error);
}

GTEST_TEST(SineTest, ToAutoDiff) {
  Eigen::Vector4d kAmp(1.1, 1.2, 1.3, 1.4);
  Eigen::Vector4d kFreq(1.5, 1.6, 1.7, 1.8);
  Eigen::Vector4d kPhase(1.9, 2.0, 2.1, 2.2);
  const Sine<double> sine_system(kAmp, kFreq, kPhase, true);
  EXPECT_TRUE(
      is_autodiffxd_convertible(sine_system, [&](const auto& converted) {
    EXPECT_EQ(0, converted.num_input_ports());
    EXPECT_EQ(3, converted.num_output_ports());
    EXPECT_EQ(kAmp, converted.amplitude_vector());
  }));
}

GTEST_TEST(SineTest, ToSymbolic) {
  const Sine<double> sine(1.0, 2.0, 3.0, true);
  EXPECT_TRUE(is_symbolic_convertible(sine));
}

}  // namespace
}  // namespace systems
}  // namespace drake
