#include "drake/automotive/dev/traffic_light.h"

#include <cstdlib>
#include <iostream>
#include <memory>

#include <Eigen/Dense>
#include <gtest/gtest.h>

namespace drake {
using Eigen::VectorXd;

using std::unique_ptr;

using systems::Context;
using systems::SystemOutput;

namespace automotive {
namespace {

GTEST_TEST(TrafficLightTest, BasicTest) {
  const TrafficLight<double> traffic_light(0, 0, 1, 6);
  unique_ptr<Context<double>> context = traffic_light.CreateDefaultContext();
  unique_ptr<SystemOutput<double>> output =
      traffic_light.AllocateOutput(*context);

  const Eigen::Vector4d signal_open(0, 0, 1, 0);
  const Eigen::Vector4d signal_closed(0, 0, 1, 1);

  // This sequence increases time and watches the signal change.
  traffic_light.CalcOutput(*context, output.get());
  VectorXd computed_output =
      output->get_vector_data(traffic_light.output().get_index())->get_value();
  EXPECT_EQ(computed_output, signal_open);

  context->set_time(1);
  traffic_light.CalcOutput(*context, output.get());
  computed_output =
      output->get_vector_data(traffic_light.output().get_index())->get_value();
  EXPECT_EQ(computed_output, signal_open);

  context->set_time(2);
  traffic_light.CalcOutput(*context, output.get());
  computed_output =
      output->get_vector_data(traffic_light.output().get_index())->get_value();
  EXPECT_EQ(computed_output, signal_open);

  context->set_time(3);
  traffic_light.CalcOutput(*context, output.get());
  computed_output =
      output->get_vector_data(traffic_light.output().get_index())->get_value();
  EXPECT_EQ(computed_output, signal_closed);

  context->set_time(4);
  traffic_light.CalcOutput(*context, output.get());
  computed_output =
      output->get_vector_data(traffic_light.output().get_index())->get_value();
  EXPECT_EQ(computed_output, signal_closed);

  context->set_time(5);
  traffic_light.CalcOutput(*context, output.get());
  computed_output =
      output->get_vector_data(traffic_light.output().get_index())->get_value();
  EXPECT_EQ(computed_output, signal_closed);

  context->set_time(6);
  traffic_light.CalcOutput(*context, output.get());
  computed_output =
      output->get_vector_data(traffic_light.output().get_index())->get_value();
  EXPECT_EQ(computed_output, signal_open);

  context->set_time(7);
  traffic_light.CalcOutput(*context, output.get());
  computed_output =
      output->get_vector_data(traffic_light.output().get_index())->get_value();
  EXPECT_EQ(computed_output, signal_open);

  context->set_time(8);
  traffic_light.CalcOutput(*context, output.get());
  computed_output =
      output->get_vector_data(traffic_light.output().get_index())->get_value();
  EXPECT_EQ(computed_output, signal_open);

  context->set_time(9);
  traffic_light.CalcOutput(*context, output.get());
  computed_output =
      output->get_vector_data(traffic_light.output().get_index())->get_value();
  EXPECT_EQ(computed_output, signal_closed);

  context->set_time(10);
  traffic_light.CalcOutput(*context, output.get());
  computed_output =
      output->get_vector_data(traffic_light.output().get_index())->get_value();
  EXPECT_EQ(computed_output, signal_closed);

  context->set_time(11);
  traffic_light.CalcOutput(*context, output.get());
  computed_output =
      output->get_vector_data(traffic_light.output().get_index())->get_value();
  EXPECT_EQ(computed_output, signal_closed);

  context->set_time(12);
  traffic_light.CalcOutput(*context, output.get());
  computed_output =
      output->get_vector_data(traffic_light.output().get_index())->get_value();
  EXPECT_EQ(computed_output, signal_open);
}

}  // namespace
}  // namespace automotive
}  // namespace drake
