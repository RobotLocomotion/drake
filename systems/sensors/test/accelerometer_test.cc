#include "drake/systems/sensors/accelerometer.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace {

using systems::BasicVector;
using systems::sensors::Accelerometer;

GTEST_TEST(TestAccelerometer, DefaultRotation) {
  double tol = 10 * std::numeric_limits<double>::epsilon();

  // Connect a pendulum to the accelerometer
  // Plant/System initialization
  systems::DiagramBuilder<double> builder;
  auto& plant = *builder.AddSystem<multibody::MultibodyPlant>(0.0);
  const std::string urdf_name =
      FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(urdf_name);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"),
                   drake::math::RigidTransform<double>());
  plant.Finalize();

  double r_BS = .375;
  const Eigen::Vector3d& gravity = plant.gravity_field().gravity_vector();
  const auto& arm_body_index = plant.GetBodyByName("arm").index();
  math::RigidTransform<double> X_BC_(Eigen::Vector3d(0, 0, -r_BS));
  auto& accelerometer =
      *builder.AddSystem<Accelerometer>(arm_body_index, X_BC_, gravity);

  builder.Connect(plant.get_body_poses_output_port(),
                  accelerometer.get_body_poses_input_port());
  builder.Connect(plant.get_body_spatial_velocities_output_port(),
                  accelerometer.get_body_velocities_input_port());
  builder.Connect(plant.get_body_spatial_accelerations_output_port(),
                  accelerometer.get_body_accelerations_input_port());

  auto diagram = builder.Build();

  auto diagram_context = diagram->CreateDefaultContext();
  auto& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());
  auto& accel_context =
      diagram->GetMutableSubsystemContext(accelerometer, diagram_context.get());

  double angle = .5;

  // Test zero-velocity state
  plant.get_actuation_input_port().FixValue(&plant_context, Vector1d(0));
  plant.SetPositions(&plant_context, Vector1d(angle));
  plant.SetVelocities(&plant_context, Vector1d(0));

  const auto& result =
      accelerometer.get_output_port(0).Eval<BasicVector<double>>(accel_context);

  // Compute expected result
  double angular_acceleration = -9.81 / .5 * sin(angle);  // g/L sin(theta)
  Eigen::Vector3d expected_result(-angular_acceleration * r_BS, 0, 0);
  Eigen::Vector3d g_S(cos(angle) * gravity(0)  - sin(angle) * gravity(2),
                      gravity(1),
                      cos(angle) * gravity(2) + sin(angle) * gravity(0));
  expected_result -= g_S;

  EXPECT_TRUE(CompareMatrices(result.get_value(), expected_result, tol));

  // Test with non-zero velocity
  double angular_velocity = -2;
  // g/L sin(theta) - b/(m * L^2) * thetadot
  angular_acceleration =
      -9.81 / .5 * sin(angle) - .1 * angular_velocity / (.5 * .5);

  plant.SetVelocities(&plant_context, Vector1d(angular_velocity));
  const auto& result_with_velocity =
      accelerometer.get_output_port(0).Eval<BasicVector<double>>(accel_context);
  Eigen::Vector3d expected_result_with_velocity(
      -angular_acceleration * r_BS, 0,
      angular_velocity * angular_velocity * r_BS);
  expected_result_with_velocity -= g_S;
  EXPECT_TRUE(CompareMatrices(result_with_velocity.get_value(),
                              expected_result_with_velocity, tol));
}

}  // namespace
}  // namespace drake
