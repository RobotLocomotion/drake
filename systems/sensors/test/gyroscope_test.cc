#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/gyroscope_sensor.h"

namespace drake {
namespace {

using math::RotationMatrix;
using systems::BasicVector;
using systems::sensors::Gyroscope;

GTEST_TEST(TestGyroscope, Rotated) {
  double tol = 10 * std::numeric_limits<double>::epsilon();

  // Connect a pendulum to the gyroscope
  // Plant/System initialization
  systems::DiagramBuilder<double> builder;
  auto& plant = *builder.AddSystem<multibody::MultibodyPlant>(0.0);
  const std::string urdf_name =
      FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  multibody::Parser parser(&plant);
  parser.AddModelFromFile(urdf_name);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"));
  plant.Finalize();

  const multibody::BodyIndex& arm_body_index =
      plant.GetBodyByName("arm").index();
  const math::RotationMatrix<double> R_BS(
      drake::math::RollPitchYaw<double>(M_PI / 2, 0, 0));

  std::cout << R_BS.matrix() << std::endl;

  const auto& gyroscope =
      Gyroscope<double>::AddToDiagram(arm_body_index, R_BS, plant, &builder);

  auto diagram = builder.Build();

  auto diagram_context = diagram->CreateDefaultContext();
  auto& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());
  auto& gyro_context =
      diagram->GetMutableSubsystemContext(gyroscope, diagram_context.get());

  double theta = M_PI / 2;
  double omega = .5;

  // Test zero-velocity state
  plant.get_actuation_input_port().FixValue(&plant_context, Vector1d(0));
  plant.SetPositions(&plant_context, Vector1d(theta));
  plant.SetVelocities(&plant_context, Vector1d(omega));

  const auto& result =
      gyroscope.get_output_port(0).Eval<BasicVector<double>>(gyro_context);

  // Compute expected result
  // Angular velocity in world coordinates is (0, omega, 0)
  // In body coordinates, it is the same, (0, omega, 0)
  // The sensor frame is rotated by pi/2 about the x-axis, so
  // the expected measurement is (0, 0, -omega)
  Eigen::Vector3d expected_result(0, 0, -omega);

  EXPECT_TRUE(CompareMatrices(result.get_value(), expected_result, tol));
}

}  // namespace
}  // namespace drake
