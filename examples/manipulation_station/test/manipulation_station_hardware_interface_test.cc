#include "drake/examples/manipulation_station/manipulation_station_hardware_interface.h"  // noqa

#include <gtest/gtest.h>

#include "drake/systems/sensors/image.h"

namespace drake {
namespace examples {
namespace manipulation_station {
namespace {

using Eigen::VectorXd;
using systems::BasicVector;

GTEST_TEST(ManipulationStationHardwareInterfaceTest, CheckPorts) {
  const int kNumIiwaDofs = 7;
  const std::vector<std::string> camera_names = {"123", "456"};
  ManipulationStationHardwareInterface station(camera_names);
  auto context = station.CreateDefaultContext();

  // Check sizes and names of the input ports.
  station.GetInputPort("iiwa_position")
      .FixValue(context.get(), VectorXd::Zero(kNumIiwaDofs));
  station.GetInputPort("iiwa_feedforward_torque")
      .FixValue(context.get(), VectorXd::Zero(kNumIiwaDofs));
  station.GetInputPort("wsg_position").FixValue(context.get(), 0.);
  station.GetInputPort("wsg_force_limit").FixValue(context.get(), 0.);

  // Check sizes and names of the output ports.
  EXPECT_EQ(station.GetOutputPort("iiwa_position_commanded")
                .template Eval<BasicVector<double>>(*context)
                .size(),
            kNumIiwaDofs);
  EXPECT_EQ(station.GetOutputPort("iiwa_position_measured")
                .Eval<BasicVector<double>>(*context)
                .size(),
            kNumIiwaDofs);
  EXPECT_EQ(station.GetOutputPort("iiwa_velocity_estimated")
                .Eval<BasicVector<double>>(*context)
                .size(),
            kNumIiwaDofs);
  EXPECT_EQ(station.GetOutputPort("iiwa_torque_commanded")
                .Eval<BasicVector<double>>(*context)
                .size(),
            kNumIiwaDofs);
  EXPECT_EQ(station.GetOutputPort("iiwa_torque_measured")
                .Eval<BasicVector<double>>(*context)
                .size(),
            kNumIiwaDofs);
  EXPECT_EQ(station.GetOutputPort("iiwa_torque_external")
                .Eval<BasicVector<double>>(*context)
                .size(),
            kNumIiwaDofs);
  EXPECT_EQ(station.GetOutputPort("wsg_state_measured")
                .Eval<BasicVector<double>>(*context)
                .size(),
            2);
  EXPECT_EQ(station.GetOutputPort("wsg_force_measured")
                .Eval<BasicVector<double>>(*context)
                .size(),
            1);

  // Camera outputs will be empty images since no messages have been received.
  for (const std::string& name : camera_names) {
    EXPECT_EQ(station.GetOutputPort("camera_" + name + "_rgb_image")
                  .Eval<systems::sensors::ImageRgba8U>(*context)
                  .size(),
              0);
    EXPECT_EQ(station.GetOutputPort("camera_" + name + "_depth_image")
                  .Eval<systems::sensors::ImageDepth32F>(*context)
                  .size(),
              0);
  }

  // TODO(russt): Consider adding mock lcm tests.  But doing so right now would
  // require exposing DrakeLcmInterface when I've so far tried to hide it.
}

}  // namespace
}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake
