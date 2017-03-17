#include "drake/systems/sensors/depth_sensor.h"

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/output_port_value.h"
#include "drake/systems/sensors/depth_sensor_output.h"
#include "drake/systems/sensors/depth_sensor_specification.h"

using Eigen::Matrix3Xd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using std::string;
using std::stringstream;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace sensors {
namespace {

// Tests DepthSensor's various accessor and streaming to-string methods.
GTEST_TEST(TestDepthSensor, AccessorsAndToStringTest) {
  const char* const kSensorName = "foo sensor";

  RigidBodyTree<double> tree;
  RigidBodyFrame<double> frame("foo frame", &tree.world(),
                               Eigen::Isometry3d::Identity());

  // Defines the Device Under Test (DUT).
  DepthSensorSpecification specification;
  DepthSensorSpecification::set_octant_1_spec(&specification);

  DepthSensor dut(kSensorName, tree, frame, specification);
  EXPECT_EQ(dut.get_specification(), specification);

  stringstream string_buffer;
  string_buffer << dut;
  const string dut_string = string_buffer.str();

  EXPECT_NE(dut_string.find("DepthSensor:"), std::string::npos);
  EXPECT_NE(dut_string.find("x = ["), std::string::npos);
  EXPECT_NE(dut_string.find("y = ["), std::string::npos);
  EXPECT_NE(dut_string.find("z = ["), std::string::npos);
  EXPECT_EQ(std::count(dut_string.begin(), dut_string.end(), '\n'), 3);
}

// A helper method that evaluates the sensor-in-an-empty-world scenario. This is
// useful for verifying that the sensor can be instantiated and that it will
// return a vector of the correct size containing invalid range measurements.
void DoEmptyWorldTest(const char* const name,
                      const DepthSensorSpecification& specification) {
  RigidBodyTree<double> tree;
  tree.compile();
  RigidBodyFrame<double> frame("foo frame", &tree.world(),
                               Eigen::Isometry3d::Identity());

  DepthSensor dut(name, tree, frame, specification);

  unique_ptr<Context<double>> context = dut.CreateDefaultContext();
  unique_ptr<SystemOutput<double>> output = dut.AllocateOutput(*context);

  int input_port_index = dut.get_rigid_body_tree_state_input_port().get_index();
  context->FixInputPort(
      input_port_index,
      VectorXd::Zero(tree.get_num_positions() + tree.get_num_velocities()));

  dut.CalcOutput(*context, output.get());

  Eigen::VectorXd expected_output =
      VectorXd::Constant(dut.get_num_depth_readings(),
          DepthSensorOutput<double>::kTooFar);

  int output_port_index = dut.get_sensor_state_output_port().get_index();

  EXPECT_TRUE(CompareMatrices(
      expected_output, output->get_vector_data(output_port_index)->get_value(),
      1e-10));

  // Confirms that Clone is correct.
  std::unique_ptr<BasicVector<double>> cloned_base =
      output->get_vector_data(output_port_index)->Clone();
  const DepthSensorOutput<double>* const cloned_sub =
      dynamic_cast<const DepthSensorOutput<double>*>(cloned_base.get());
  ASSERT_NE(cloned_sub, nullptr);
  EXPECT_TRUE(CompareMatrices(
      expected_output, cloned_sub->get_value(),
      1e-10));
}

// Tests the ability to scan the sensor's X,Y plane (i.e., pitch = 0).
GTEST_TEST(TestDepthSensor, XyEmptyWorldTest) {
  DepthSensorSpecification specification;
  DepthSensorSpecification::set_xy_planar_spec(&specification);
  DoEmptyWorldTest("foo depth sensor", specification);
}

// Tests the ability to scan the sensor's X,Z plane (i.e., yaw = 0).
GTEST_TEST(TestDepthSensor, XzEmptyWorldTest) {
  DepthSensorSpecification specification;
  DepthSensorSpecification::set_xz_planar_spec(&specification);
  DoEmptyWorldTest("foo depth sensor", specification);
}

// Tests the ability to scan the sensor's surrounding X,Y,Z volume.
GTEST_TEST(TestDepthSensor, XyzEmptyWorldTest) {
  DepthSensorSpecification specification;
  DepthSensorSpecification::set_xyz_spherical_spec(&specification);
  DoEmptyWorldTest("foo depth sensor", specification);
}

const double kBoxWidth{0.1};

// A helper method that evaluates a scenario containing a depth sensor in the
// world's origin and a 0.1 x 0.1 x 0.1 meter box located at position
// (@p box_x, @p box_y, @p box_z) in the world frame. This is useful for
// verifying that the sensor can detect the box. The return value is a vector of
// depth measurements.
std::pair<VectorX<double>, Matrix3Xd> DoBoxOcclusionTest(
    const char* const name, const DepthSensorSpecification& specification,
    const Vector3d& box_xyz) {
  RigidBodyTree<double> tree;

  // Adds a box to the world at the specified location.
  auto weld_to_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(),
      RigidBodyTreeConstants::kWorldName, nullptr, box_xyz,
      Vector3d::Zero() /* rpy */);

  DrakeShapes::Box geom(Eigen::Vector3d(kBoxWidth, kBoxWidth, kBoxWidth));
  Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
  T_element_to_link.translation() = box_xyz;  // Top of the box is at z = 0.
  RigidBody<double>& world = tree.world();
  Eigen::Vector4d color(1, 1, 1, 1);
  world.AddVisualElement(
      DrakeShapes::VisualElement(geom, T_element_to_link, color));
  tree.addCollisionElement(
      DrakeCollision::Element(geom, T_element_to_link, &world), world,
      "terrain");
  tree.compile();

  auto frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "foo frame",
      &tree.world(), Eigen::Isometry3d::Identity());

  tree.addFrame(frame);
  tree.compile();

  DepthSensor dut(name, tree, *frame, specification);

  unique_ptr<Context<double>> context = dut.CreateDefaultContext();
  unique_ptr<SystemOutput<double>> output = dut.AllocateOutput(*context);

  int input_port_index = dut.get_rigid_body_tree_state_input_port().get_index();
  context->FixInputPort(input_port_index,
                        Eigen::VectorXd::Zero(tree.get_num_positions() +
                                              tree.get_num_velocities()));

  dut.CalcOutput(*context, output.get());

  int output_port_index = dut.get_sensor_state_output_port().get_index();

  const DepthSensorOutput<double>* depth_sensor_output =
      dynamic_cast<const DepthSensorOutput<double>*>(
          output->get_vector_data(output_port_index));
  DRAKE_DEMAND(depth_sensor_output != nullptr);
  return std::make_pair(depth_sensor_output->get_value(),
                        depth_sensor_output->GetPointCloud());
}

// Tests the ability to scan the sensor's X,Y plane (i.e., pitch = 0, yaw in
// [-M_PI_2, M_PI_2]) in a world containing a box at (0.5, 0, 0). This checks
// both the depth image and the point cloud.
GTEST_TEST(TestDepthSensor, XyBoxInWorldTest) {
  DepthSensorSpecification specification;
  DepthSensorSpecification::set_xy_planar_spec(&specification);

  // Adjusts the min / max yaw range to ensure the sensor is able to obtain
  // four depth measurements of the box's sensor-facing surface.
  specification.set_min_yaw(-M_PI_2);
  specification.set_max_yaw(M_PI_2);

  const Vector3d box_xyz(0.5, 0, 0);  // x, y, z location of box.

  const std::pair<VectorX<double>, Matrix3Xd> result =
      DoBoxOcclusionTest("foo depth sensor", specification, box_xyz);
  const VectorX<double> depth_measurements = std::get<0>(result);

  Eigen::VectorXd expected_depths =
      VectorXd::Constant(depth_measurements.size(),
          DepthSensorOutput<double>::kTooFar);

  const double box_distance = box_xyz(0) - kBoxWidth / 2;
  expected_depths(23) = box_distance / cos(specification.min_yaw() +
                                           23 * specification.yaw_increment());
  expected_depths(24) = box_distance / cos(specification.min_yaw() +
                                           24 * specification.yaw_increment());
  expected_depths(25) = box_distance / cos(specification.min_yaw() +
                                           25 * specification.yaw_increment());
  expected_depths(26) = box_distance / cos(specification.min_yaw() +
                                           26 * specification.yaw_increment());

  EXPECT_TRUE(CompareMatrices(depth_measurements, expected_depths, 1e-8));

  const Matrix3Xd point_cloud = std::get<1>(result);

  const int kExpectedPointCloudSize = 4;
  Matrix3Xd expected_point_cloud = Matrix3Xd::Zero(3, kExpectedPointCloudSize);
  const double yaw_23 =
      specification.min_yaw() + 23 * specification.yaw_increment();
  const double yaw_24 =
      specification.min_yaw() + 24 * specification.yaw_increment();
  const double yaw_25 =
      specification.min_yaw() + 25 * specification.yaw_increment();
  const double yaw_26 =
      specification.min_yaw() + 26 * specification.yaw_increment();
  expected_point_cloud(0, 0) = expected_depths(23) * cos(yaw_23);
  expected_point_cloud(1, 0) = expected_depths(23) * sin(yaw_23);
  expected_point_cloud(0, 1) = expected_depths(24) * cos(yaw_24);
  expected_point_cloud(1, 1) = expected_depths(24) * sin(yaw_24);
  expected_point_cloud(0, 2) = expected_depths(25) * cos(yaw_25);
  expected_point_cloud(1, 2) = expected_depths(25) * sin(yaw_25);
  expected_point_cloud(0, 3) = expected_depths(26) * cos(yaw_26);
  expected_point_cloud(1, 3) = expected_depths(26) * sin(yaw_26);

  EXPECT_TRUE(CompareMatrices(point_cloud, expected_point_cloud, 1e-8));
}

// Tests the ability to scan the sensor's X,Z plane (i.e., pitch in [0, M_PI /
// 2], yaw = 0) in a world containing a box at (0, 0, 0.5). This also verifies
// that pitch rotates the sensor's optical frame around the sensor's base
// frame's -Y axis.
GTEST_TEST(TestDepthSensor, XzBoxInWorldTest) {
  DepthSensorSpecification specification;
  DepthSensorSpecification::set_xz_planar_spec(&specification);

  const Vector3d box_xyz(0, 0, 0.5);  // x, y, z location of box.
  const std::pair<VectorX<double>, Matrix3Xd> result =
      DoBoxOcclusionTest("foo depth sensor", specification, box_xyz);
  const VectorX<double> depth_measurements = std::get<0>(result);

  Eigen::VectorXd expected_output =
      VectorXd::Constant(depth_measurements.size(),
          DepthSensorOutput<double>::kTooFar);

  const double box_distance = box_xyz(2) - kBoxWidth / 2;
  // sin() is used below because pitch is the angle between the sensor's base
  // frame's +X axis and its ray whereas box_distance is measured along the
  // sensor's base frame's +Z axis.
  expected_output(48) =
      box_distance /
      sin(specification.min_pitch() + 48 * specification.pitch_increment());
  expected_output(49) =
      box_distance /
      sin(specification.min_pitch() + 49 * specification.pitch_increment());

  EXPECT_TRUE(CompareMatrices(depth_measurements, expected_output, 1e-8,
                              MatrixCompareType::absolute));
}

// Tests that the sensor will return negative infinity when the object is less
// than the minimum sensing distance. This also covers the scenario where
// min_yaw = max_yaw = min_pitch = max_pitch = 0.
GTEST_TEST(TestDepthSensor, TestTooClose) {
  DepthSensorSpecification specification;
  DepthSensorSpecification::set_x_linear_spec(&specification);

  // Note that the minimum sensing distance is 1m but the box is placed 0.5m in
  // front of the sensor.
  const Vector3d box_xyz(0.5, 0, 0);  // x, y, z location of box.
  const std::pair<VectorX<double>, Matrix3Xd> result = DoBoxOcclusionTest(
      "foo depth sensor", specification, box_xyz);
  const VectorX<double> depth_measurements = std::get<0>(result);

  EXPECT_EQ(depth_measurements.size(), 1);
  Eigen::VectorXd expected_output =
      VectorXd::Constant(depth_measurements.size(),
          DepthSensorOutput<double>::kTooClose);

  EXPECT_TRUE(CompareMatrices(depth_measurements, expected_output, 1e-8,
                              MatrixCompareType::absolute));

  const Matrix3Xd point_cloud = std::get<1>(result);
  EXPECT_EQ(point_cloud.cols(), 0);
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
