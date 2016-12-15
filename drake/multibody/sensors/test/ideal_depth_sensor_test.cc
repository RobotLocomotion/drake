#include "drake/multibody/sensors/ideal_depth_sensor.h"

#include "gtest/gtest.h"

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system_output.h"

using Eigen::Vector3d;
using Eigen::VectorXd;

using std::string;
using std::stringstream;
using std::unique_ptr;

namespace drake {

using systems::Context;

namespace multibody {
namespace {

// Tests IdealDepthSensor's various accessor and streaming to-string methods.
GTEST_TEST(TestIdealDepthSensor, AccessorsAndToStringTest) {
  const char* const kSensorName = "foo sensor";
  const double kMinTheta = 0;
  const double kMaxTheta = M_PI / 2;
  const double kMinPhi = 0;
  const double kMaxPhi = M_PI / 2;
  const int kNumThetaValues = 10;
  const int kNumPhiValues = 5;
  const double kMinRange = 0;
  const double kMaxRange = 1;

  RigidBodyTree<double> tree;
  RigidBodyFrame<double> frame("foo frame", &tree.world(),
                               Eigen::Isometry3d::Identity());

  // Defines the Device Under Test (DUT).
  IdealDepthSensor dut(tree, kSensorName, frame, kMinTheta, kMaxTheta, kMinPhi,
                       kMaxPhi, kNumThetaValues, kNumPhiValues, kMinRange,
                       kMaxRange);

  EXPECT_EQ(dut.get_min_theta(), kMinTheta);
  EXPECT_EQ(dut.get_max_theta(), kMaxTheta);
  EXPECT_EQ(dut.get_min_phi(), kMinPhi);
  EXPECT_EQ(dut.get_max_phi(), kMaxPhi);
  EXPECT_EQ(dut.get_num_theta_values(), kNumThetaValues);
  EXPECT_EQ(dut.get_num_phi_values(), kNumPhiValues);
  EXPECT_EQ(dut.get_min_range(), kMinRange);
  EXPECT_EQ(dut.get_max_range(), kMaxRange);
  EXPECT_EQ(dut.num_pixel_rows(), dut.get_num_phi_values());
  EXPECT_EQ(dut.num_pixel_cols(), dut.get_num_theta_values());
  EXPECT_EQ(dut.get_num_depth_readings(), kNumThetaValues * kNumPhiValues);

  stringstream string_buffer;
  string_buffer << dut;
  const string dut_string = string_buffer.str();

  EXPECT_NE(dut_string.find("IdealDepthSensor:"), std::string::npos);
  EXPECT_NE(dut_string.find("x = ["), std::string::npos);
  EXPECT_NE(dut_string.find("y = ["), std::string::npos);
  EXPECT_NE(dut_string.find("z = ["), std::string::npos);
  EXPECT_EQ(std::count(dut_string.begin(), dut_string.end(), '\n'), 3);
}

// A helper method that evaluates the sensor-in-an-empty-world scenario. This is
// useful for verifying that the sensor can be instantiated and that it will
// return a vector of the correct size containing invalid range measurements.
void DoUnoccludedTest(const char* const sensor_name, const double min_theta,
                      const double max_theta, const double min_phi,
                      const double max_phi, const int num_theta_values,
                      const int num_phi_values, const double min_range,
                      const double max_range) {
  RigidBodyTree<double> tree;
  tree.compile();
  RigidBodyFrame<double> frame("foo frame", &tree.world(),
                               Eigen::Isometry3d::Identity());

  IdealDepthSensor dut(tree, sensor_name, frame, min_theta, max_theta, min_phi,
                       max_phi, num_theta_values, num_phi_values, min_range,
                       max_range);

  unique_ptr<Context<double>> context = dut.CreateDefaultContext();
  unique_ptr<SystemOutput<double>> output = dut.AllocateOutput(*context);

  int input_port_index = dut.get_rigid_body_tree_state_input_port().get_index();
  context->FixInputPort(input_port_index,
                        VectorXd::Zero(tree.get_num_states()));

  dut.CalcOutput(*context, output.get());

  Eigen::VectorXd expected_output = VectorXd::Constant(
      dut.get_num_depth_readings(), IdealDepthSensor::kInvalidDepthMeasurement);

  int output_port_index = dut.get_sensor_state_output_port().get_index();

  EXPECT_TRUE(CompareMatrices(
      expected_output, output->get_vector_data(output_port_index)->get_value(),
      1e-10));
}

// Tests the ability to scan the sensor's X,Y plane (i.e., phi = 0).
GTEST_TEST(TestIdealDepthSensor, XyEmptyWorldTest) {
  // TODO(liang.fok): Consider defining these constants inside an
  // IdealDepthSensorTrait struct.
  const char* const kSensorName = "foo depth sensor";
  const double kMinTheta = -M_PI / 2;
  const double kMaxTheta = M_PI / 2;
  const double kMinPhi = 0;
  const double kMaxPhi = 0;
  const int kNumThetaValues = 50;
  const int kNumPhiValues = 1;
  const double kMinRange = 0;
  const double kMaxRange = 1;

  DoUnoccludedTest(kSensorName, kMinTheta, kMaxTheta, kMinPhi, kMaxPhi,
                   kNumThetaValues, kNumPhiValues, kMinRange, kMaxRange);
}

// Tests the ability to scan the sensor's X,Z plane (i.e., theta = 0).
GTEST_TEST(TestIdealDepthSensor, XzEmptyWorldTest) {
  const char* const kSensorName = "foo depth sensor";
  const double kMinTheta = 0;
  const double kMaxTheta = 0;
  const double kMinPhi = -M_PI / 2;
  const double kMaxPhi = M_PI / 2;
  const int kNumThetaValues = 1;
  const int kNumPhiValues = 50;
  const double kMinRange = 0;
  const double kMaxRange = 1;

  DoUnoccludedTest(kSensorName, kMinTheta, kMaxTheta, kMinPhi, kMaxPhi,
                   kNumThetaValues, kNumPhiValues, kMinRange, kMaxRange);
}

// Tests the ability to scan the sensor's surrounding X,Y,Z volume.
GTEST_TEST(TestIdealDepthSensor, XyzEmptyWorldTest) {
  const char* const kSensorName = "foo depth sensor";
  const double kMinTheta = -M_PI;
  const double kMaxTheta = M_PI;
  const double kMinPhi = -M_PI / 2;
  const double kMaxPhi = M_PI / 2;
  const int kNumThetaValues = 50;
  const int kNumPhiValues = 50;
  const double kMinRange = 0;
  const double kMaxRange = 1;

  DoUnoccludedTest(kSensorName, kMinTheta, kMaxTheta, kMinPhi, kMaxPhi,
                   kNumThetaValues, kNumPhiValues, kMinRange, kMaxRange);
}

// A helper method that evaluates a scenario containing of a sensor in the
// world's origin with a 0.1 x 0.1 x 0.1 meter box located at position
// (@p box_x, @p box_y, @p box_z) in the world frame. This is useful for
// verifying that the sensor can detect the box. The return value is a vector of
// depth measurements.
VectorX<double> DoBoxOcclusionTest(
    const char* const sensor_name, const double min_theta,
    const double max_theta, const double min_phi, const double max_phi,
    const int num_theta_values, const int num_phi_values,
    const double min_range, const double max_range, const double box_x,
    const double box_y, const double box_z) {
  RigidBodyTree<double> tree;

  // Adds a box to the world at the specified location.
  const Vector3d xyz(box_x, box_y, box_z);
  const Vector3d rpy = Vector3d::Zero();
  auto weld_to_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(),
      RigidBodyTree<double>::kWorldName, nullptr, xyz, rpy);
  parsers::sdf::AddModelInstancesFromSdfFile(
      GetDrakePath() + "/multibody/sensors/test/box.sdf",
      multibody::joints::kFixed, weld_to_frame, &tree);

  auto frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "foo frame",
      &tree.world(), Eigen::Isometry3d::Identity());

  tree.addFrame(frame);
  tree.compile();

  IdealDepthSensor dut(tree, sensor_name, *frame, min_theta, max_theta, min_phi,
                       max_phi, num_theta_values, num_phi_values, min_range,
                       max_range);

  unique_ptr<Context<double>> context = dut.CreateDefaultContext();
  unique_ptr<SystemOutput<double>> output = dut.AllocateOutput(*context);

  int input_port_index = dut.get_rigid_body_tree_state_input_port().get_index();
  context->FixInputPort(input_port_index,
                        Eigen::VectorXd::Zero(tree.get_num_states()));

  dut.CalcOutput(*context, output.get());

  int output_port_index = dut.get_sensor_state_output_port().get_index();

  return output->get_vector_data(output_port_index)->get_value();
}

// Tests the ability to scan the sensor's X,Y plane (i.e., phi = 0) in a world
// containing a box.
GTEST_TEST(TestIdealDepthSensor, XyBoxInWorldTest) {
  const char* const kSensorName = "foo depth sensor";
  const double kMinTheta = -M_PI / 2;
  const double kMaxTheta = M_PI / 2;
  const double kMinPhi = 0;
  const double kMaxPhi = 0;
  const int kNumThetaValues = 50;
  const int kNumPhiValues = 1;
  const double kMinRange = 0;
  const double kMaxRange = 1;

  const double box_x = 0.5;
  const double box_y = 0;
  const double box_z = 0;

  const VectorX<double> depth_measurements = DoBoxOcclusionTest(
      kSensorName, kMinTheta, kMaxTheta, kMinPhi, kMaxPhi, kNumThetaValues,
      kNumPhiValues, kMinRange, kMaxRange, box_x, box_y, box_z);

  Eigen::VectorXd expected_output = VectorXd::Constant(
      depth_measurements.size(), IdealDepthSensor::kInvalidDepthMeasurement);

  const double theta_increment =
      (kMaxTheta - kMinTheta) / (kNumThetaValues - 1);

  // The following value matches the depth of the box as defined in
  // drake-distro/drake/multibody/sensors/test/box.sdf.
  const double box_width = 0.1;
  const double box_distance = box_x - box_width / 2;
  expected_output(23) = box_distance / cos(kMinTheta + 23 * theta_increment);
  expected_output(24) = box_distance / cos(kMinTheta + 24 * theta_increment);
  expected_output(25) = box_distance / cos(kMinTheta + 25 * theta_increment);
  expected_output(26) = box_distance / cos(kMinTheta + 26 * theta_increment);

  std::string message;
  EXPECT_TRUE(CompareMatrices(depth_measurements, expected_output, 1e-8,
                              MatrixCompareType::absolute, &message))
      << message;
}

}  // namespace
}  // namespace multibody
}  // namespace drake
