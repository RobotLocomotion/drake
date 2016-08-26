#include "drake/systems/framework/primitives/pid_controller.h"

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
GTEST_TEST(PidController, EvalOutput) {
  const double kp = 2.0;
  const double ki = 3.0;
  const int port_length = 3;
  PidController<double> controller(kp, ki, port_length);

  auto context = controller.CreateDefaultContext();
  auto output = controller.AllocateOutput(*context);
  ASSERT_NE(nullptr, context);
  ASSERT_NE(nullptr, output);

  // Creates a "fake" free-standing input port.
  Vector3<double> input_vector(1.0, 2.0, 3.0);

  auto vec = std::make_unique<BasicVector<double>>(port_length);
  vec->get_mutable_value() << input_vector;
  context->SetInputPort(0, MakeInput(std::move(vec)));

  // Initializes the controllers' context to the default value in which the
  // integral is zero and evaluates the output.
  controller.SetDefaultState(context.get());
  controller.EvalOutput(*context, output.get());

  ASSERT_EQ(1, output->get_num_ports());
  const VectorBase<double>* output_vector = output->get_vector_data(0);
  EXPECT_EQ(3, output_vector->get_value().rows());
  EXPECT_EQ(kp * input_vector, output_vector->get_value());

  // Initializes the integral to a non-zero value. A more interesting example.
  VectorX<double> integral_value(port_length);
  integral_value << 3.0, 2.0, 1.0;
  controller.set_integral_value(context.get(), integral_value);
  controller.EvalOutput(*context, output.get());
  EXPECT_EQ(kp * input_vector + ki * integral_value,
            output_vector->get_value());
}


}  // namespace
}  // namespace systems
}  // namespace drake
