#include "drake/automotive/dev/traffic_light_aware_controller.h"

#include <stdlib.h>

#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include "drake/automotive/gen/simple_car_state.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
using std::abs;
using std::unique_ptr;

using Eigen::VectorXd;

using systems::BasicVector;
using systems::Context;
using systems::InputPortDescriptor;
using systems::SystemOutput;

namespace automotive {
namespace {
using systems::LeafSystem;

VectorXd ReadOutput(const TrafficLightAwareController<double>& system,
                    const SystemOutput<double>& output_port);
void FeedInput(const VectorXd& input_value,
               const InputPortDescriptor<double>& input_port,
               Context<double>* context);

GTEST_TEST(TrafficLightAwareControllerTest, BasicTest) {
  const Eigen::Vector2d go(0, 100);
  const Eigen::Vector2d stop(0, -100);

  const Eigen::Vector4d signal_open(0, 0, 1, 0);
  const Eigen::Vector4d signal_closed(0, 0, 1, 1);

  // These vectors are (x,y) coordinates of the car's posiiton.
  const Eigen::Vector2d car_far(0, 10);
  const Eigen::Vector2d car_near(0, 0);

  // Set up the controller for testing.
  const TrafficLightAwareController<double> controller;
  unique_ptr<Context<double>> context = controller.CreateDefaultContext();
  unique_ptr<SystemOutput<double>> output_port =
      controller.AllocateOutput(*context);
  VectorXd controller_output;

  // Feed it a signal closed, but car is too far to worry.
  FeedInput(signal_closed, controller.traffic_light_input(), context.get());
  FeedInput(car_far, controller.car_state(), context.get());
  FeedInput(go, controller.other_controller_acceleration(), context.get());
  controller.CalcOutput(*context, output_port.get());
  controller_output = ReadOutput(controller, *output_port);
  EXPECT_EQ(controller_output, go);

  // Feed it a signal closed, and car the car is near so it should slam on the
  // brakes.
  FeedInput(signal_closed, controller.traffic_light_input(), context.get());
  FeedInput(car_near, controller.car_state(), context.get());
  FeedInput(go, controller.other_controller_acceleration(), context.get());
  controller.CalcOutput(*context, output_port.get());
  controller_output = ReadOutput(controller, *output_port);
  EXPECT_EQ(controller_output, stop);

  // Feed it a signal open
  FeedInput(signal_open, controller.traffic_light_input(), context.get());
  FeedInput(car_near, controller.car_state(), context.get());
  FeedInput(go, controller.other_controller_acceleration(), context.get());
  controller.CalcOutput(*context, output_port.get());
  controller_output = ReadOutput(controller, *output_port);
  EXPECT_EQ(controller_output, go);
}

VectorXd ReadOutput(const TrafficLightAwareController<double>& my_system,
                    const SystemOutput<double>& output_port) {
  VectorXd out_data =
      output_port.get_vector_data(my_system.acceleration_output().get_index())
          ->get_value();
  return out_data;
}

void FeedInput(const VectorXd& input_value,
               const InputPortDescriptor<double>& input_port,
               Context<double> * context) {
  context->FixInputPort(input_port.get_index(), input_value);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
