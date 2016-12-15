#include "drake/systems/sensors/depth_sensor.h"

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/sensors/depth_sensor_specification.h"

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
  const DepthSensorSpecification& specification =
      DepthSensorSpecification::get_octant_1_spec();
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
      VectorXd::Constant(dut.get_num_depth_readings(), DepthSensor::kTooFar);

  int output_port_index = dut.get_sensor_state_output_port().get_index();

  EXPECT_TRUE(CompareMatrices(
      expected_output, output->get_vector_data(output_port_index)->get_value(),
      1e-10));
}

// Tests the ability to scan the sensor's X,Y plane (i.e., phi = 0).
GTEST_TEST(TestDepthSensor, XyEmptyWorldTest) {
  DoEmptyWorldTest("foo depth sensor",
                   DepthSensorSpecification::get_xy_planar_spec());
}

// Tests the ability to scan the sensor's X,Z plane (i.e., theta = 0).
GTEST_TEST(TestDepthSensor, XzEmptyWorldTest) {
  DoEmptyWorldTest("foo depth sensor",
                   DepthSensorSpecification::get_xz_planar_spec());
}

// Tests the ability to scan the sensor's surrounding X,Y,Z volume.
GTEST_TEST(TestDepthSensor, XyzEmptyWorldTest) {
  DoEmptyWorldTest("foo depth sensor",
                   DepthSensorSpecification::get_xyz_spherical_spec());
}

const double kBoxWidth{0.1};

// A helper method that evaluates a scenario containing of a sensor in the
// world's origin with a 0.1 x 0.1 x 0.1 meter box located at position
// (@p box_x, @p box_y, @p box_z) in the world frame. This is useful for
// verifying that the sensor can detect the box. The return value is a vector of
// depth measurements.
VectorX<double> DoBoxOcclusionTest(
    const char* const name, const DepthSensorSpecification& specification,
    const Vector3d& box_xyz) {
  RigidBodyTree<double> tree;

  // Adds a box to the world at the specified location.
  auto weld_to_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(),
      RigidBodyTree<double>::kWorldName, nullptr, box_xyz,
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
  tree.updateStaticCollisionElements();

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

  return output->get_vector_data(output_port_index)->get_value();
}

// Tests the ability to scan the sensor's X,Y plane (i.e., phi = 0, theta in
// [-M_PI / 2, M_PI / 2]) in a world containing a box at (0.5, 0, 0).
GTEST_TEST(TestDepthSensor, XyBoxInWorldTest) {
  DepthSensorSpecification specification =
      DepthSensorSpecification::get_xy_planar_spec();

  // Adjusts the min / max theta range to ensure the sensor is able to obtain
  // four depth measurements of the box's sensor-facing surface.
  specification.set_min_theta(-M_PI / 2);
  specification.set_max_theta(M_PI / 2);

  const Vector3d box_xyz(0.5, 0, 0);  // x, y, z location of box.
  const VectorX<double> depth_measurements =
      DoBoxOcclusionTest("foo depth sensor", specification, box_xyz);

  Eigen::VectorXd expected_output =
      VectorXd::Constant(depth_measurements.size(), DepthSensor::kTooFar);

  const double box_distance = box_xyz(0) - kBoxWidth / 2;
  expected_output(23) =
      box_distance /
      cos(specification.min_theta() + 23 * specification.theta_increment());
  expected_output(24) =
      box_distance /
      cos(specification.min_theta() + 24 * specification.theta_increment());
  expected_output(25) =
      box_distance /
      cos(specification.min_theta() + 25 * specification.theta_increment());
  expected_output(26) =
      box_distance /
      cos(specification.min_theta() + 26 * specification.theta_increment());

  std::string message;
  EXPECT_TRUE(CompareMatrices(depth_measurements, expected_output, 1e-8,
                              MatrixCompareType::absolute, &message))
      << message;
}

// Tests the ability to scan the sensor's X,Z plane (i.e., phi in [0, M_PI / 2],
// theta = 0) in a world containing a box at (0, 0, 0.5). This also verifies
// that phi rotates the sensor's optical frame around the sensor's base frame's
// -Y axis.
GTEST_TEST(TestDepthSensor, XzBoxInWorldTest) {
  DepthSensorSpecification specification =
      DepthSensorSpecification::get_xz_planar_spec();

  const Vector3d box_xyz(0, 0, 0.5);  // x, y, z location of box.
  const VectorX<double> depth_measurements =
      DoBoxOcclusionTest("foo depth sensor", specification, box_xyz);

  Eigen::VectorXd expected_output =
      VectorXd::Constant(depth_measurements.size(), DepthSensor::kTooFar);

  const double box_distance = box_xyz(2) - kBoxWidth / 2;
  // sin() is used below because phi is the angle between the sensor's base
  // frame's +X axis and its ray whereas box_distance is measured along the
  // sensor's base frame's +Z axis.
  expected_output(48) = box_distance / sin(specification.min_phi() +
                                           48 * specification.phi_increment());
  expected_output(49) = box_distance / sin(specification.min_phi() +
                                           49 * specification.phi_increment());

  std::string message;
  EXPECT_TRUE(CompareMatrices(depth_measurements, expected_output, 1e-8,
                              MatrixCompareType::absolute, &message))
      << message;
}

// Tests that the sensor will return negative infinity when the object is less
// than the minimum sensing distance. This also covers the scenario where
// min_theta = max_theta = min_phi = max_phi = 0.
GTEST_TEST(TestDepthSensor, TestTooClose) {
  // Note that the minimum sensing distance is 1m but the box is placed 0.5m in
  // front of the sensor.
  const Vector3d box_xyz(0.5, 0, 0);  // x, y, z location of box.
  const VectorX<double> depth_measurements = DoBoxOcclusionTest(
      "foo depth sensor", DepthSensorSpecification::get_x_linear_spec(),
      box_xyz);

  EXPECT_EQ(depth_measurements.size(), 1);
  Eigen::VectorXd expected_output =
      VectorXd::Constant(depth_measurements.size(), DepthSensor::kTooClose);

  std::string message;
  EXPECT_TRUE(CompareMatrices(depth_measurements, expected_output, 1e-8,
                              MatrixCompareType::absolute, &message))
      << message;
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
