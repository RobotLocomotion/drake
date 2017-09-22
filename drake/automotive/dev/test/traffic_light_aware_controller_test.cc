#include "drake/automotive/dev/traffic_light_aware_controller.h"
#include <stdlib.h>
#include <iostream>
#include <memory>
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include "drake/systems/framework/basic_vector.h"
#include "drake/automotive/gen/simple_car_state.h"

namespace drake {
using std::abs;
using std::unique_ptr;
using systems::SystemOutput;
using systems::InputPortDescriptor;
using systems::Context;
using systems::BasicVector;

using Eigen::MatrixXd;
using Eigen::VectorXd;

using std::cout;
using std::endl;
namespace automotive {
namespace {
using systems::LeafSystem;

VectorXd ReadOutput(const LeafSystem<double>& system,
                    const SystemOutput<double>& output_port);
void FeedInput( const VectorXd& input_value, const InputPortDescriptor<double>& input_port, unique_ptr<Context<double>>& context);

GTEST_TEST(TrafficLightAwareControllerTest, BasicTest) {
  //DrivingCommand<double> go;
  //go.set_steering_angle(0);
  //go.set_acceleration(100);
  //DrivingCommand<double> stop;
  //stop.set_steering_angle(0);
  //stop.set_acceleration(-100);
  VectorXd go; go(0) = 0; go(1) = 100;
  VectorXd stop; stop(0) = 0; stop(1) = -100;

  VectorXd signal_open(4);
  signal_open(0) = 0; signal_open(1) = 0;
  signal_open(2) = 1; signal_open(3) = 0;

  VectorXd signal_closed(4);
  signal_closed(0) = 0; signal_closed(1) = 0;
  signal_closed(2) = 1; signal_closed(3) = 1;

	//SimpleCarState<double> car_far;
	//car_far.set_x(0);
	//car_far.set_y(10);
	//SimpleCarState<double> car_near;
	//car_near.set_x(0);
	//car_near.set_y(1);
	VectorXd car_far; car_far(0) = 0; car_far(1) = 10;
	VectorXd car_near; car_near(0) = 0; car_near(1) = 0;

	// Set up the controller for testing
  TrafficLightAwareController<double> controller;
  unique_ptr<Context<double>> context = controller.CreateDefaultContext();
  unique_ptr<SystemOutput<double>> output_port = controller.AllocateOutput(*context);
  VectorXd controller_output;

	// Feed it a signal closed, but car is too far to worry
	FeedInput( signal_closed, controller.traffic_light_input(), context );
	FeedInput( car_far, controller.car_state(), context );
	FeedInput( go, controller.other_controller_acceleration(), context );
	controller_output = ReadOutput( controller, *output_port );
	EXPECT_EQ( controller_output, go );
	
	// Feed it a signal closed, and car the car is near so it should slam on the brakes
	FeedInput( signal_closed, controller.traffic_light_input(), context );
	FeedInput( car_near, controller.car_state(), context );
	FeedInput( go, controller.other_controller_acceleration(), context );
	controller_output = ReadOutput( controller, *output_port );
	EXPECT_EQ( controller_output, stop );
	
	// Feed it a signal open
	FeedInput( signal_open, controller.traffic_light_input(), context );
	FeedInput( car_near, controller.car_state(), context );
	FeedInput( go, controller.other_controller_acceleration(), context );
	controller_output = ReadOutput( controller, *output_port );
	EXPECT_EQ( controller_output, go );
}

VectorXd ReadOutput(const LeafSystem<double>& system,
                    const SystemOutput<double>& output_port);
	VectorXd out_data = output_port.get_vector_data(system.output().get_index())->get_value();
	return out_data;
}

void FeedInput( const VectorXd& input_value, const InputPortDescriptor<double>& input_port, const unique_ptr<Context<double>>& context) {
  context->FixInputPort(input_port.get_index(), input_value);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
