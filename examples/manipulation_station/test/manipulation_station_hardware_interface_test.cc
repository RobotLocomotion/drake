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
  context->FixInputPort(station.GetInputPort("iiwa_position").get_index(),
                        VectorXd::Zero(kNumIiwaDofs));
  context->FixInputPort(
      station.GetInputPort("iiwa_feedforward_torque").get_index(),
      VectorXd::Zero(kNumIiwaDofs));
  context->FixInputPort(station.GetInputPort("wsg_position").get_index(),
                        Vector1d::Zero());
  context->FixInputPort(station.GetInputPort("wsg_force_limit").get_index(),
                        Vector1d::Zero());

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

  // TODO(russt): Consider adding mock lcm tests.  But doing so right now
  // would require (1) exposing lcm to the constructor in the public interface,
  // when I've so far tried to hide it, and (2) adding (empty)
  // implementations of StartReceiveThread() and other lcm calls to the
  // DrakeMockLcm.
}

}  // namespace
}  // namespace manipulation_station
}  // namespace examples
}  // namespace drake
