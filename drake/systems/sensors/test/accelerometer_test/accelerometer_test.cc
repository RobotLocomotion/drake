#include "drake/systems/sensors/accelerometer.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/sensors/accelerometer_output.h"
#include "drake/systems/sensors/test/accelerometer_test/accelerometer_example_diagram.h"

using Eigen::Vector3d;

using std::make_unique;
using std::move;
using std::string;
using std::stringstream;
using std::unique_ptr;

namespace drake {

using parsers::urdf::AddModelInstanceFromUrdfFileToWorld;

namespace systems {
namespace sensors {
namespace {

const char* const kSensorName = "foo sensor";

// Attaches an accelerometer to a box that's falling due to gravity. The
// provided `xyz` and `rpy` parameters specify the transformation between the
// sensing frame and the frame of the rigid body to which the accelerometer is
// attached.
void TestAccelerometerFreeFall(const Eigen::Vector3d& xyz,
                               const Eigen::Vector3d& rpy) {
  auto tree = make_unique<RigidBodyTree<double>>();

  // Adds a box to the RigidBodyTree and obtains its model instance ID.
  const parsers::ModelInstanceIdTable model_instance_id_table =
      AddModelInstanceFromUrdfFileToWorld(
          GetDrakePath() + "/multibody/models/box.urdf",
          drake::multibody::joints::kQuaternion, tree.get());

  // Adds a frame to the RigidBodyTree called "accelerometer frame" that is
  // coincident with the "box" body within the RigidBodyTree.
  const Eigen::Matrix3d R_BA = drake::math::rpy2rotmat(rpy);

  Eigen::Isometry3d X_BA;  // Transform from accelerometer's to body's frames.
  X_BA.matrix() << R_BA, xyz, 0, 0, 0, 1;

  auto sensor_frame = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "accelerometer frame",
      tree->FindBody("box"), X_BA);
  tree->addFrame(sensor_frame);
  EXPECT_EQ(tree->get_num_actuators(), 0);

  // Defines the Device Under Test (DUT).
  Accelerometer dut(kSensorName, *sensor_frame, *tree);

  unique_ptr<Context<double>> dut_context = dut.CreateDefaultContext();
  EXPECT_EQ(dut_context->get_num_input_ports(), 2);
  EXPECT_EQ(dut_context->get_continuous_state_vector().size(), 0);

  const int num_positions = tree->get_num_positions();
  const int num_velocities = tree->get_num_velocities();
  const int num_states = num_positions + num_velocities;

  // The RigidBodyTree has 13 DOFs: 7 generalized positions and 6 generalized
  // velocities.
  EXPECT_EQ(num_positions, 7);
  EXPECT_EQ(num_velocities, 6);

  VectorX<double> plant_state(num_states);
  plant_state << tree->getZeroConfiguration(),
                 VectorX<double>::Zero(tree->get_num_velocities());
  dut_context->FixInputPort(
      dut.get_plant_state_input_port().get_index(),
      make_unique<BasicVector<double>>(plant_state));
  dut_context->FixInputPort(
      dut.get_plant_state_derivative_input_port().get_index(),
      make_unique<BasicVector<double>>(VectorX<double>::Zero(num_states)));

  unique_ptr<SystemOutput<double>> output = dut.AllocateOutput(*dut_context);
  ASSERT_EQ(output->get_num_ports(), 1);
  dut.CalcOutput(*dut_context, output.get());

  // The frame of the RigidBody to which the sensor is attached is coincident
  // with the world frame.
  const Eigen::Matrix3d R_BW = Eigen::Matrix3d::Identity();
  // Since R_BA is orthogonal its inverse is equal to its transpose.
  // Eigen::Matrix3d::transpose() is used below because it's computationally
  // cheaper than Eigen::Matrix3d::inverse().
  const auto R_AB = R_BA.transpose();
  Vector3d expected_measurement = R_AB * R_BW * tree->a_grav.tail<3>();
  EXPECT_TRUE(CompareMatrices(output->get_vector_data(0)->get_value(),
                              expected_measurement, 1e-10,
                              MatrixCompareType::absolute));
}

// Tests that the accelerometer attached to a floating rigid body can measure
// the effects of gravity. The sensor's frame is rotated along all three axes
// relative to the body's frame.
GTEST_TEST(TestAccelerometer, TestFreeFall_VariousOrientations) {
  const double kNumIncrements = 5;
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
  for (double roll = -M_PI; roll <= M_PI; roll += M_PI * 2 / kNumIncrements) {
    for (double pitch = -M_PI_2; pitch <= M_PI_2;
         pitch += M_PI / kNumIncrements) {
      for (double yaw = -M_PI; yaw <= M_PI; yaw += M_PI * 2 / kNumIncrements) {
        Eigen::Vector3d rpy(roll, pitch, yaw);
        TestAccelerometerFreeFall(xyz, rpy);
       }
    }
  }
}

// Tests that the accelerometer attached to a floating rigid body can measure
// the effects of gravity. The sensor's frame is translated along all three axes
// relative to the body's frame.
GTEST_TEST(TestAccelerometer, TestFreeFall_VariousTranslations) {
  const double kMinRange = -1;
  const double kMaxRange = 1;
  const double kIncrementSize = (kMaxRange - kMinRange) / 5.0;
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
  for (double x = kMinRange; x <= kMaxRange; x += kIncrementSize) {
    for (double y = kMinRange; y <= kMaxRange; y += kIncrementSize) {
      for (double z = kMinRange; z <= kMaxRange; z += kIncrementSize) {
        Eigen::Vector3d xyz(x, y, z);
        TestAccelerometerFreeFall(xyz, rpy);
      }
    }
  }
}

// Tests the accelerometer using a non-identity sensor frame that's attached
// to a swinging pendulum. This also tests Accelerometer::AttachAccelerometer().
GTEST_TEST(TestAccelerometer, TestSensorAttachedToSwingingPendulum) {
  ::drake::lcm::DrakeMockLcm lcm;
  lcm.EnableLoopBack();
  AccelerometerExampleDiagram diagram(&lcm);
  diagram.Initialize();

  // Prepares to integrate.
  unique_ptr<Context<double>> context = diagram.AllocateContext();
  diagram.SetDefaultState(*context, context->get_mutable_state());
  diagram.SetInitialState(context.get(), 1.57, 0);

  Simulator<double> simulator(diagram, std::move(context));
  // Decrease the step size to get x_dot(0) to be closer to x(1).
  const double stepSize = 1e-4;
  simulator.get_mutable_integrator()->set_maximum_step_size(stepSize);
  simulator.Initialize();

  const AccelerometerTestLogger& logger = diagram.get_logger();
  const RigidBodyTree<double>& tree = diagram.get_tree();

  const double kStepToTime = 0.01;
  simulator.StepTo(kStepToTime);

  const Context<double>& simulator_context = simulator.get_context();
  const Context<double>& logger_context =
      diagram.GetSubsystemContext(simulator_context, &logger);

  const auto latest_measurement = logger.get_acceleration(logger_context);
  const Eigen::VectorXd x = logger.get_plant_state(logger_context);
  const Eigen::VectorXd x_dot =
      logger.get_plant_state_derivative(logger_context);
  const Eigen::VectorXd q = x.head(tree.get_num_positions());
  const Eigen::VectorXd v = x.tail(tree.get_num_velocities());

  const KinematicsCache<double> cache = tree.doKinematics(q, v);

  // The subsequent code implements the following math.
  //
  // Let:
  //
  //   - v be the linear acceleration of the sensor in the world frame.
  //   - v_ref be the linear velocity of a reference point in the world frame.
  //   - r be the vector from the sensor to the reference point expressed in the
  //       world frame.
  //   - w be the angular velocity of the pendulum in the world frame.
  //
  // The equation for v is:
  //
  //     v = w x r + v_ref
  //
  // Let the reference point be the world's origin. Since the pendulum is
  // anchored to the world's origin, v_ref is a vector of zeros. Thus:
  //
  //     v = w x r
  //
  // Taking the derivative of v with respect to time gives v_dot, which is
  // defined as follows:
  //
  //     v_dot = w_dot x r + w x r_dot
  //
  const Vector3d w_dot(0, x_dot(1), 0);
  const Vector3d w(0, x(1), 0);

  const Eigen::Isometry3d r =
      tree.CalcFramePoseInWorldFrame(cache,
          diagram.get_sensor_frame());
  const drake::Vector6<double> r_dot =
      tree.CalcFrameSpatialVelocityInWorldFrame(
          cache, diagram.get_sensor_frame());

  const Vector3d r_translation = r.translation();
  const Vector3d r_dot_translation = r_dot.tail<3>();

  const Vector3d w_dot_cross_r = w_dot.cross(r_translation);
  const Vector3d w_cross_r_dot = w.cross(r_dot_translation);

  Vector3d v_dot_in_world_frame = w_dot_cross_r + w_cross_r_dot;
  if (diagram.get_accelerometer().get_include_gravity()) {
     v_dot_in_world_frame += diagram.get_tree().a_grav.tail<3>();
  }
  const Vector3d v_dot = r.linear().transpose() * v_dot_in_world_frame;

  EXPECT_TRUE(CompareMatrices(latest_measurement, v_dot, 1e-11,
                              MatrixCompareType::absolute));
}

}  // namespace
}  // namespace sensors
}  // namespace systems
}  // namespace drake
