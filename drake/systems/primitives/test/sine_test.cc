#include "drake/systems/primitives/sine.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/input_port_value.h"
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
                    const Eigen::MatrixXd& expected_outputs) {
  auto context = sine_system.CreateDefaultContext();

  // Verifies that Sine allocates no state variables in the context.
  EXPECT_EQ(0, context->get_continuous_state().size());
  auto output = sine_system.AllocateOutput(*context);

  if (sine_system.is_time_based()) {
    // If the system is time based, the input_vectors Matrix should only contain
    // a row vector of time instances to test against. In this case, the system
    // has zero inputs ports.
    ASSERT_EQ(input_vectors.rows(), 1);
    ASSERT_EQ(0, sine_system.get_num_input_ports());
    ASSERT_EQ(0, context->get_num_input_ports());

    for (int i = 0; i < input_vectors.cols(); i++) {
      // Initialize the time in seconds to be used by the Sine system.
      context->set_time(input_vectors(0, i));

      // Calculate the Sine system output.
      sine_system.CalcOutput(*context, output.get());

      // Checks that the number of output ports in the Sine system and the
      // system output are consistent.
      ASSERT_EQ(3, output->get_num_ports());
      ASSERT_EQ(3, sine_system.get_num_output_ports());
      const BasicVector<double> *output_vector = output->get_vector_data(0);
      ASSERT_NE(nullptr, output_vector);

      Eigen::VectorXd expected_vector = expected_outputs.col(i);
      EXPECT_TRUE(expected_vector.isApprox(output_vector->get_value(), 1e-3));
    }
  } else {
    // Loop over the input vectors and check that the Sine system outputs match
    // the expected outputs.
    for (int i = 0; i < input_vectors.cols(); i++) {
      auto input =
          make_unique<BasicVector<double>>(
              sine_system.get_amplitude_vector().size());
      ASSERT_EQ(1, sine_system.get_num_input_ports());
      ASSERT_EQ(1, context->get_num_input_ports());

      // Confirm the the size of the input port matches the size of the sample
      // inputs (i.e., each column in the input_vectors matrix represents a
      // sampling instant. The number of rows in input_vectors should match the
      // size of the input port).
      ASSERT_EQ(input->get_mutable_value().size(), input_vectors.rows());

      // Initialize the input and associate it with the context.
      input->get_mutable_value() << input_vectors.col(i);
      context->FixInputPort(0, std::move(input));

      // Calculate the Sine system output.
      sine_system.CalcOutput(*context, output.get());

      // Checks that the number of output ports in the Sine system and the
      // system output are consistent.
      ASSERT_EQ(3, output->get_num_ports());
      ASSERT_EQ(3, sine_system.get_num_output_ports());
      const BasicVector<double>* output_vector = output->get_vector_data(0);
      ASSERT_NE(nullptr, output_vector);

      Eigen::VectorXd expected_vector = expected_outputs.col(i);
      EXPECT_TRUE(expected_vector.isApprox(output_vector->get_value(), 1e-3));
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

  TestSineSystem(
      *sine_system, input_vector.transpose(), expected_output.transpose());
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

  TestSineSystem(
      *sine_system, input_vector.transpose(), expected_output.transpose());
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

  TestSineSystem(*sine_system, input_vectors, expected_output);
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

  TestSineSystem(*sine_system, input_vectors, expected_output);
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

  TestSineSystem(*sine_system, input_vectors.transpose(), expected_output);
}

GTEST_TEST(SineTest, SineAccessorTest) {
  Eigen::Vector4d kAmp(1.1, 1.2, 1.3, 1.4);
  Eigen::Vector4d kFreq(1.5, 1.6, 1.7, 1.8);
  Eigen::Vector4d kPhase(1.9, 2.0, 2.1, 2.2);
  const auto sine_system = make_unique<Sine<double>>(kAmp, kFreq, kPhase, true);
  // Verifies the Sine accessors are OK.
  EXPECT_THROW(sine_system->get_amplitude(), std::runtime_error);
  EXPECT_THROW(sine_system->get_frequency(), std::runtime_error);
  EXPECT_THROW(sine_system->get_phase(), std::runtime_error);
}

GTEST_TEST(SineTest, ToAutoDiff) {
  Eigen::Vector4d kAmp(1.1, 1.2, 1.3, 1.4);
  Eigen::Vector4d kFreq(1.5, 1.6, 1.7, 1.8);
  Eigen::Vector4d kPhase(1.9, 2.0, 2.1, 2.2);
  const Sine<double> sine_system(kAmp, kFreq, kPhase, true);
  EXPECT_TRUE(
      is_autodiffxd_convertible(sine_system, [&](const auto& converted) {
    EXPECT_EQ(0, converted.get_num_input_ports());
    EXPECT_EQ(3, converted.get_num_output_ports());
    EXPECT_EQ(kAmp, converted.get_amplitude_vector());
  }));
}

GTEST_TEST(SineTest, ToSymbolic) {
  const Sine<double> sine(1.0, 2.0, 3.0, true);
  EXPECT_TRUE(is_symbolic_convertible(sine));
}

}  // namespace
}  // namespace systems
}  // namespace drake
