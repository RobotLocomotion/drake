#include "drake/systems/framework/primitives/pid_controller2.h"

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_input.h"

#include "gtest/gtest.h"

using std::make_unique;

namespace drake {
namespace systems {
namespace {

// TODO(amcastro-tri): Create a diagram with a ConstantVectorSource feeding
// the input of the Gain system.
template <class T>
std::unique_ptr<FreestandingInputPort> MakeInput(
    std::unique_ptr<BasicVector<T>> data) {
  return make_unique<FreestandingInputPort>(std::move(data));
}

// Tests the EvalOutput() method for the PidController.
GTEST_TEST(PidController, EvalMethods) {
  const double kp = 2.0;
  const double ki = 3.0;
  const double kd = 1.0;
  const int port_length = 3;
  PidController<double> controller(kp, ki, kd, port_length);

  auto context = controller.CreateDefaultContext();
  auto output = controller.AllocateOutput(*context);
  auto derivatives = controller.AllocateTimeDerivatives();
  ASSERT_NE(nullptr, context);
  ASSERT_NE(nullptr, output);

  // Creates a "fake" free-standing input port.
  Vector3<double> error_signal(1.0, 2.0, 3.0);
  Vector3<double> error_signal_rate(1.3, 0.9, 3.14);

  // Error signal input port.
  auto vec0 = std::make_unique<BasicVector<double>>(port_length);
  vec0->get_mutable_value() << error_signal;
  context->SetInputPort(0, MakeInput(std::move(vec0)));

  // Error signal rate input port.
  auto vec1 = std::make_unique<BasicVector<double>>(port_length);
  vec1->get_mutable_value() << error_signal_rate;
  context->SetInputPort(1, MakeInput(std::move(vec1)));

  // Initializes the controllers' context to the default value in which the
  // integral is zero and evaluates the output.
  controller.SetDefaultState(context.get());
  controller.EvalOutput(*context, output.get());

  ASSERT_EQ(1, output->get_num_ports());
  const VectorBase<double>* output_vector = output->get_vector_data(0);
  EXPECT_EQ(3, output_vector->get_value().rows());
  EXPECT_EQ(kp * error_signal + kd * error_signal_rate,
            output_vector->get_value());

  // Initializes the integral to a non-zero value. A more interesting example.
  VectorX<double> integral_value(port_length);
  integral_value << 3.0, 2.0, 1.0;
  controller.set_integral_value(context.get(), integral_value);
  controller.EvalOutput(*context, output.get());
  EXPECT_EQ(kp * error_signal + ki * integral_value + kd * error_signal_rate,
            output_vector->get_value());

  // Evaluates derivatives and asserts correctness.
  controller.EvalTimeDerivatives(*context, derivatives.get());
  ASSERT_EQ(3, derivatives->get_state().size());
  ASSERT_EQ(0, derivatives->get_generalized_position().size());
  ASSERT_EQ(0, derivatives->get_generalized_velocity().size());
  ASSERT_EQ(3, derivatives->get_misc_continuous_state().size());

  // The only state in the PID controller is the integral of the input signal.
  // Therefore the time derivative of the state equals the input error signal.
  EXPECT_EQ(error_signal, derivatives->get_state().CopyToVector());
}

}  // namespace
}  // namespace systems
}  // namespace drake
