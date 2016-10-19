#include "drake/systems/framework/primitives/gain.h"

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"

#include "gtest/gtest.h"

using Eigen::Vector3d;
using std::make_unique;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace {

// TODO(amcastro-tri): Create a diagram with a ConstantVectorSource feeding
// the input of the Gain system.
template <class T>
unique_ptr<FreestandingInputPort> MakeInput(
    unique_ptr<BasicVector<T>> data) {
  return make_unique<FreestandingInputPort>(std::move(data));
}

template <typename T>
void TestGainSystem(const Gain<T>& gain_system,
                    const Eigen::VectorXd& input_vector,
                    const Eigen::VectorXd& expected_output) {
  auto context = gain_system.CreateDefaultContext();

  // Verifies that Gain allocates no state variables in the context.
  EXPECT_EQ(nullptr, context->get_continuous_state());
  auto output = gain_system.AllocateOutput(*context);
  auto input = make_unique<BasicVector<double>>(gain_system.get_gain().size());

  // Checks that the number of input ports in the Gain system and the Context
  // are consistent.
  ASSERT_EQ(1, gain_system.get_num_input_ports());
  ASSERT_EQ(1, context->get_num_input_ports());

  input->get_mutable_value() << input_vector;

  // Hook input of the expected size.
  context->SetInputPort(0, MakeInput(std::move(input)));

  gain_system.EvalOutput(*context, output.get());

  // Checks that the number of output ports in the Gain system and the
  // SystemOutput are consistent.
  ASSERT_EQ(1, output->get_num_ports());
  ASSERT_EQ(1, gain_system.get_num_output_ports());
  const BasicVector<double>* output_vector = output->get_vector_data(0);
  ASSERT_NE(nullptr, output_vector);
  EXPECT_EQ(expected_output, output_vector->get_value());
}

GTEST_TEST(GainTest, VectorThroughGainSystem) {
  const double kGain{2.0};
  const int kSize = 3;
  const auto gain_system = make_unique<Gain<double>>(kGain, kSize);
  const Eigen::Vector3d input_vector(1.0, 3.14, 2.18);
  const Eigen::Vector3d expected_output(1.0 * kGain, 3.14 * kGain, 2.18 * kGain);
  TestGainSystem(*gain_system, input_vector, expected_output);
}

// Tests the ability to use a gain vector where the values vary.
GTEST_TEST(GainTest, GainVectorTest) {
  const Vector4<double> gain_values(1.0, 2.0, 3.0, 4.0);
  const auto gain_system = make_unique<Gain<double>>(gain_values);
  const Eigen::Vector4d input_vector(9.81, 5.46, 16.24, 98.12);
  const Eigen::Vector4d expected_output(
      1.0 * 9.81, 2.0 * 5.46, 3.0 * 16.24, 4.0 * 98.12);
  TestGainSystem(*gain_system, input_vector, expected_output);
}

}  // namespace
}  // namespace systems
}  // namespace drake
