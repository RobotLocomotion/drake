#include "drake/systems/sensors/depth_sensor_to_lcm_point_cloud_message.h"

#include <cmath>
#include <memory>
#include <utility>

#include <gtest/gtest.h>
#include "bot_core/pointcloud_t.hpp"

#include "drake/common/drake_path.h"
#include "drake/systems/rendering/pose_vector.h"
#include "drake/systems/sensors/depth_sensor_output.h"
#include "drake/systems/sensors/depth_sensor_specification.h"

namespace drake {
namespace systems {

using rendering::PoseVector;

namespace sensors {
namespace {

class TestDepthSensorToLcmPointCloudMessage : public ::testing::Test {
 protected:
  // Initializes the Device Under Test (DUT) based on spec_ and induces it to
  // output a bot_core::pointcloud_t message.
  //
  // @pre spec_ was initialized.
  const bot_core::pointcloud_t& InitializeAndOutputMessage(
      const PoseVector<double>& X_WS = PoseVector<double>(),
      bool fix_pose_input_port = true) {
    // The Device Under Test (DUT).
    DepthSensorToLcmPointCloudMessage dut(spec_);
    EXPECT_EQ(dut.get_num_input_ports(), 2);
    EXPECT_EQ(dut.get_num_output_ports(), 1);
    const InputPortDescriptor<double>& sensor_data_input_port =
        dut.depth_readings_input_port();
    EXPECT_EQ(sensor_data_input_port.get_system(), &dut);
    EXPECT_EQ(sensor_data_input_port.size(), spec_.num_depth_readings());
    const InputPortDescriptor<double>& pose_input_port =
        dut.pose_input_port();
    EXPECT_EQ(pose_input_port.size(), PoseVector<double>::kSize);

    auto depth_sensor_output =
        std::make_unique<DepthSensorOutput<double>>(spec_);
    const double half_range = (spec_.max_range() - spec_.min_range()) / 2;
    depth_sensor_output->SetFromVector(Eigen::VectorXd::Ones(
        spec_.num_depth_readings()) * half_range);

    std::unique_ptr<Context<double>> context = dut.CreateDefaultContext();
    context->FixInputPort(sensor_data_input_port.get_index(),
        std::move(depth_sensor_output));

    if (fix_pose_input_port) {
      auto pose_input =  std::make_unique<PoseVector<double>>();
      pose_input->set_translation(X_WS.get_translation());
      pose_input->set_rotation(X_WS.get_rotation());
      context->FixInputPort(pose_input_port.get_index(), std::move(pose_input));
    }

    output_ = dut.AllocateOutput(*context);

    dut.CalcOutput(*context, output_.get());

    const int output_port_index =
        dut.pointcloud_message_output_port().get_index();
    return output_->get_data(output_port_index)->
        template GetValue<bot_core::pointcloud_t>();
  }

  DepthSensorSpecification spec_;
  std::unique_ptr<SystemOutput<double>> output_;
};

const int kX(0);
const int kY(1);
const int kZ(2);

// Tests that the DUT can generate a bot_core::pointcloud_t message that
// contains a point cloud within the first octant of the world frame. The points
// within the point cloud are hard-coded to be halfway between the minimum and
// maximum depth sensing range.
TEST_F(TestDepthSensorToLcmPointCloudMessage, Octant1Test) {
  DepthSensorSpecification::set_octant_1_spec(&spec_);
  const bot_core::pointcloud_t message = InitializeAndOutputMessage();
  EXPECT_EQ(message.n_points, spec_.num_depth_readings());
  for (int i = 0; i < message.n_points; ++i) {
    EXPECT_GE(message.points.at(i).at(kX), 0);
    EXPECT_GE(message.points.at(i).at(kY), 0);
    EXPECT_GE(message.points.at(i).at(kZ), 0);
  }

  // Offsets the sensor's frame relative to the world frame. Then verifies the
  // resulting depth measurements are offset relative to the original
  // measurements. The tolerance value was determined empirically.
  const double kXOffset{1};
  const double kYOffset{2};
  const double kZOffset{3};
  PoseVector<double> X_WS;
  X_WS.set_translation({kXOffset, kYOffset, kZOffset});
  const bot_core::pointcloud_t& offset_message =
      InitializeAndOutputMessage(X_WS);
  EXPECT_EQ(offset_message.n_points, spec_.num_depth_readings());
  for (int i = 0; i < message.n_points; ++i) {
    EXPECT_NEAR(message.points.at(i).at(kX) + kXOffset,
                offset_message.points.at(i).at(kX), 1e-6);
    EXPECT_NEAR(message.points.at(i).at(kY) + kYOffset,
                offset_message.points.at(i).at(kY), 1e-6);
    EXPECT_NEAR(message.points.at(i).at(kZ) + kZOffset,
                offset_message.points.at(i).at(kZ), 1e-6);
  }
}

// Tests that the DUT can generate a bot_core::pointcloud_t message that
// contains a point cloud that resides within the x/y plane of the world frame.
// The points within the point cloud are hard-coded to be halfway between the
// minimum and maximum depth sensing range.
TEST_F(TestDepthSensorToLcmPointCloudMessage, XyPlanarTest) {
  DepthSensorSpecification::set_xy_planar_spec(&spec_);
  const bot_core::pointcloud_t& message = InitializeAndOutputMessage();
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

// Tests that the DUT can generate a bot_core::pointcloud_t message that
// contains a point cloud that resides within the x/z plane of the world frame.
// The points within the point cloud are hard-coded to be halfway between the
// minimum and maximum depth sensing range.
TEST_F(TestDepthSensorToLcmPointCloudMessage, XzPlanarTest) {
  DepthSensorSpecification::set_xz_planar_spec(&spec_);
  const bot_core::pointcloud_t& message = InitializeAndOutputMessage();
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

// Tests that the DUT can generate a bot_core::pointcloud_t message that
// contains a point cloud that forms a sphere centered at the origin plane of
// the world frame. The points within the point cloud are hard-coded to be
// halfway between the minimum and maximum depth sensing range.
TEST_F(TestDepthSensorToLcmPointCloudMessage, XyzSphericalTest) {
  DepthSensorSpecification::set_xyz_spherical_spec(&spec_);
  const bot_core::pointcloud_t& message = InitializeAndOutputMessage();
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

// Tests that the DUT can generate a bot_core::pointcloud_t message that
// contains a point cloud consisting of a single point on the x-axis of the
// world frame that is halfway between the minimum and maximum depth sensing
// range.
TEST_F(TestDepthSensorToLcmPointCloudMessage, XLinearTest) {
  DepthSensorSpecification::set_x_linear_spec(&spec_);
  const bot_core::pointcloud_t& message = InitializeAndOutputMessage();
  EXPECT_EQ(message.n_points, spec_.num_depth_readings());
  const double half_range = (spec_.max_range() - spec_.min_range()) / 2;
  for (int i = 0; i < message.n_points; ++i) {
    EXPECT_EQ(message.points.at(i).at(kX), half_range);
    EXPECT_EQ(message.points.at(i).at(kY), 0);
    EXPECT_EQ(message.points.at(i).at(kZ), 0);
  }
}

// Tests that the DUT will throw an exception if its pose input port is not
// connected.
TEST_F(TestDepthSensorToLcmPointCloudMessage, UnconnectedPoseInput) {
  DepthSensorSpecification::set_octant_1_spec(&spec_);
  EXPECT_THROW(InitializeAndOutputMessage(
      PoseVector<double>(), false /* fix_pose_input_port */),
      std::runtime_error);
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
