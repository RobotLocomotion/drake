#include "drake/systems/sensors/depth_sensor_vis.h"

#include <cmath>
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "bot_core/pointcloud_t.hpp"

#include "drake/common/drake_path.h"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/systems/framework/system_port_descriptor.h"
#include "drake/systems/sensors/depth_sensor_output.h"
#include "drake/systems/sensors/depth_sensor_specification.h"

namespace drake {
namespace systems {
namespace sensors {
namespace {

class TestDepthSensorVis : public ::testing::Test {
 protected:
  // Initializes the Device Under Test (DUT) based on spec_ and induces it to
  // publish a bot_core::pointcloud_t message.
  //
  // @pre spec_ was initialized.
  bot_core::pointcloud_t InitializeAndPublishMessage() {
    // The Device Under Test (DUT).
    DepthSensorVis dut(kSensorName, spec_, &lcm_);
    const InputPortDescriptor<double>& input_port =
        dut.depth_readings_input_port();
    EXPECT_EQ(input_port.get_system(), &dut);
    EXPECT_EQ(input_port.size(), spec_.num_depth_readings());

    auto depth_sensor_output =
        std::make_unique<DepthSensorOutput<double>>(spec_);

    const double half_range = (spec_.max_range() - spec_.min_range()) / 2;
    depth_sensor_output->SetFromVector(Eigen::VectorXd::Ones(
        spec_.num_depth_readings()) * half_range);

    std::unique_ptr<Context<double>> context = dut.CreateDefaultContext();
    context->FixInputPort(input_port.get_index(),
        std::move(depth_sensor_output));

    dut.Publish(*context);

    return lcm_.template DecodeLastPublishedMessageAs<bot_core::pointcloud_t>(
        "DRAKE_POINTCLOUD_" + std::string(kSensorName));
  }

  const char* const kSensorName = "Test Sensor";
  lcm::DrakeMockLcm lcm_;
  DepthSensorSpecification spec_;
};

const int kX(0);
const int kY(1);
const int kZ(2);

TEST_F(TestDepthSensorVis, Octant1Test) {
  DepthSensorSpecification::set_octant_1_spec(&spec_);
  const bot_core::pointcloud_t message = InitializeAndPublishMessage();
  EXPECT_EQ(message.n_points, spec_.num_depth_readings());
  for (int i = 0; i < message.n_points; ++i) {
    EXPECT_GE(message.points.at(i).at(kX), 0);
    EXPECT_GE(message.points.at(i).at(kY), 0);
    EXPECT_GE(message.points.at(i).at(kZ), 0);
  }
}

TEST_F(TestDepthSensorVis, XyPlanarTest) {
  DepthSensorSpecification::set_xy_planar_spec(&spec_);
  const bot_core::pointcloud_t message = InitializeAndPublishMessage();
  EXPECT_EQ(message.n_points, spec_.num_depth_readings());
  const double half_range = (spec_.max_range() - spec_.min_range()) / 2;
  for (int i = 0; i < message.n_points; ++i) {
    // The following tolerance was empirically determined.
    EXPECT_NEAR(std::sqrt(std::pow(message.points.at(i).at(kX), 2) +
                          std::pow(message.points.at(i).at(kY), 2)),
                half_range, 1e-7);
    EXPECT_EQ(message.points.at(i).at(kZ), 0);
  }
}

TEST_F(TestDepthSensorVis, XzPlanarTest) {
  DepthSensorSpecification::set_xz_planar_spec(&spec_);
  const bot_core::pointcloud_t message = InitializeAndPublishMessage();
  EXPECT_EQ(message.n_points, spec_.num_depth_readings());
  const double half_range = (spec_.max_range() - spec_.min_range()) / 2;
  for (int i = 0; i < message.n_points; ++i) {
    // The following tolerance was empirically determined.
    EXPECT_NEAR(std::sqrt(std::pow(message.points.at(i).at(kX), 2) +
                          std::pow(message.points.at(i).at(kZ), 2)),
                half_range, 1e-7);
    EXPECT_EQ(message.points.at(i).at(kY), 0);
  }
}

TEST_F(TestDepthSensorVis, XyzSphericalTest) {
  DepthSensorSpecification::set_xyz_spherical_spec(&spec_);
  const bot_core::pointcloud_t message = InitializeAndPublishMessage();
  EXPECT_EQ(message.n_points, spec_.num_depth_readings());
  const double half_range = (spec_.max_range() - spec_.min_range()) / 2;
  for (int i = 0; i < message.n_points; ++i) {
    // The following tolerance was empirically determined.
    EXPECT_NEAR(std::sqrt(std::pow(message.points.at(i).at(kX), 2) +
                          std::pow(message.points.at(i).at(kY), 2) +
                          std::pow(message.points.at(i).at(kZ), 2)),
                half_range, 1e-7);
  }
}

TEST_F(TestDepthSensorVis, XLinearTest) {
  DepthSensorSpecification::set_x_linear_spec(&spec_);
  const bot_core::pointcloud_t message = InitializeAndPublishMessage();
  EXPECT_EQ(message.n_points, spec_.num_depth_readings());
  const double half_range = (spec_.max_range() - spec_.min_range()) / 2;
  for (int i = 0; i < message.n_points; ++i) {
    EXPECT_EQ(message.points.at(i).at(kX), half_range);
    EXPECT_EQ(message.points.at(i).at(kY), 0);
    EXPECT_EQ(message.points.at(i).at(kZ), 0);
  }
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
