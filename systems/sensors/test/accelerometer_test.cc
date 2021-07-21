#include "drake/systems/sensors/accelerometer.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace {

using systems::BasicVector;
using systems::sensors::Accelerometer;

class AccelerometerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Plant/System initialization.
    systems::DiagramBuilder<double> builder;
    plant_ = builder.AddSystem<multibody::MultibodyPlant>(0.0);
    const std::string urdf_name =
        FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
    multibody::Parser parser(plant_);
    parser.AddModelFromFile(urdf_name);
    plant_->Finalize();

    // Connect a pendulum to the accelerometer.
    const multibody::Body<double>& arm_body = plant_->GetBodyByName("arm");
    const math::RigidTransform<double> X_BS(Eigen::Vector3d(0, 0, -r_BS_));
    gravity_ = plant_->gravity_field().gravity_vector();
    accel_default_ = builder.AddSystem<Accelerometer>(arm_body, X_BS, gravity_);

    builder.Connect(plant_->get_body_poses_output_port(),
                    accel_default_->get_body_poses_input_port());
    builder.Connect(plant_->get_body_spatial_velocities_output_port(),
                    accel_default_->get_body_velocities_input_port());
    builder.Connect(plant_->get_body_spatial_accelerations_output_port(),
                    accel_default_->get_body_accelerations_input_port());

    const math::RigidTransform<double> X_BS_rotated(
        math::RotationMatrix<double>::MakeYRotation(M_PI / 2),
        Eigen::Vector3d(0, 0, -r_BS_));
    accel_rotated_ = &Accelerometer<double>::AddToDiagram(
        arm_body, X_BS_rotated, gravity_, *plant_, &builder);

    diagram_ = builder.Build();
  }

  double r_BS_ = 0.375;
  Eigen::Vector3d gravity_;
  const Accelerometer<double>* accel_default_;
  const Accelerometer<double>* accel_rotated_;
  std::unique_ptr<systems::Diagram<double>> diagram_;
  multibody::MultibodyPlant<double>* plant_;
};

TEST_F(AccelerometerTest, DefaultRotation) {
  double tol = 10 * std::numeric_limits<double>::epsilon();

  auto diagram_context = diagram_->CreateDefaultContext();
  auto& plant_context =
      diagram_->GetMutableSubsystemContext(*plant_, diagram_context.get());
  auto& accel_context = diagram_->GetMutableSubsystemContext(
      *accel_default_, diagram_context.get());

  double angle = .5;

  // Test zero-velocity state.
  plant_->get_actuation_input_port().FixValue(&plant_context, Vector1d(0));
  plant_->SetPositions(&plant_context, Vector1d(angle));
  plant_->SetVelocities(&plant_context, Vector1d(0));

  const auto& result =
      accel_default_->get_measurement_output_port().Eval<BasicVector<double>>(
          accel_context);

  // Compute expected result:
  // g/L sin(theta)
  double angular_acceleration = -gravity_.norm() / .5 * sin(angle);
  Eigen::Vector3d expected_result(-angular_acceleration * r_BS_, 0, 0);
  Eigen::Vector3d g_S(cos(angle) * gravity_(0) - sin(angle) * gravity_(2),
                      gravity_(1),
                      cos(angle) * gravity_(2) + sin(angle) * gravity_(0));
  expected_result -= g_S;

  EXPECT_TRUE(CompareMatrices(result.get_value(), expected_result, tol));

  // Test with non-zero velocity.
  double angular_velocity = -2;
  // g/L sin(theta) - b/(m * L^2) * thetadot
  double g = 9.81;
  // Constants below from URDF.
  double L = .5;
  double m = 1;
  double b = .1;
  angular_acceleration =
      -g / L * sin(angle) - b * angular_velocity / (m * L * L);

  plant_->SetVelocities(&plant_context, Vector1d(angular_velocity));
  const auto& result_with_velocity =
      accel_default_->get_output_port(0).Eval<BasicVector<double>>(
          accel_context);
  Eigen::Vector3d expected_result_with_velocity(
      -angular_acceleration * r_BS_, 0,
      angular_velocity * angular_velocity * r_BS_);
  expected_result_with_velocity -= g_S;
  EXPECT_TRUE(CompareMatrices(result_with_velocity.get_value(),
                              expected_result_with_velocity, tol));
}

TEST_F(AccelerometerTest, Rotated) {
  double tol = 10 * std::numeric_limits<double>::epsilon();

  auto diagram_context = diagram_->CreateDefaultContext();
  auto& plant_context =
      diagram_->GetMutableSubsystemContext(*plant_, diagram_context.get());
  auto& accel_context = diagram_->GetMutableSubsystemContext(
      *accel_rotated_, diagram_context.get());

  double angle = .5;

  // Test zero-velocity state.
  plant_->get_actuation_input_port().FixValue(&plant_context, Vector1d(0));
  plant_->SetPositions(&plant_context, Vector1d(angle));
  plant_->SetVelocities(&plant_context, Vector1d(0));

  const auto& result =
      accel_rotated_->get_output_port(0).Eval<BasicVector<double>>(
          accel_context);

  // Compute expected result:
  // g/L sin(theta)
  double angular_acceleration = -gravity_.norm() / .5 * sin(angle);
  Eigen::Vector3d expected_result(0, 0, -angular_acceleration * r_BS_);
  Eigen::Vector3d g_S(-cos(angle) * gravity_(2) - sin(angle) * gravity_(0),
                      gravity_(1),
                      cos(angle) * gravity_(0) - sin(angle) * gravity_(2));
  expected_result -= g_S;

  EXPECT_TRUE(CompareMatrices(result.get_value(), expected_result, tol));

  // Test with non-zero velocity.
  double angular_velocity = -2;
  // g/L sin(theta) - b/(m * L^2) * thetadot
  double g = 9.81;
  // Constants below from URDF.
  double L = .5;
  double m = 1;
  double b = .1;
  angular_acceleration =
      -g / L * sin(angle) - b * angular_velocity / (m * L * L);

  plant_->SetVelocities(&plant_context, Vector1d(angular_velocity));
  const auto& result_with_velocity =
      accel_rotated_->get_output_port(0).Eval<BasicVector<double>>(
          accel_context);
  Eigen::Vector3d expected_result_with_velocity(
      -angular_velocity * angular_velocity * r_BS_, 0,
      -angular_acceleration * r_BS_);
  expected_result_with_velocity -= g_S;
  EXPECT_TRUE(CompareMatrices(result_with_velocity.get_value(),
                              expected_result_with_velocity, tol));
}

TEST_F(AccelerometerTest, ScalarConversionTest) {
  EXPECT_TRUE(is_autodiffxd_convertible(*accel_default_));
  EXPECT_TRUE(is_symbolic_convertible(*accel_default_));
}

}  // namespace
}  // namespace drake
