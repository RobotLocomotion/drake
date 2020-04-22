#include "drake/multibody/plant/propeller.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace {

GTEST_TEST(PropellerTest, MustRegisterWithSceneGraphTest) {
  systems::DiagramBuilder<double> builder;

  const double timestep = 0.0;
  auto plant = builder.AddSystem<MultibodyPlant<double>>(timestep);

  const double mass = 1.0;
  const auto M_B = SpatialInertia<double>::MakeFromCentralInertia(
      mass, Vector3<double>::Zero(),
      mass * UnitInertia<double>::TriaxiallySymmetric(1.0));
  const RigidBody<double>& body = plant->AddRigidBody("mass", M_B);
  plant->Finalize();

  DRAKE_EXPECT_THROWS_MESSAGE(
      PropellerInfo(*plant, body, math::RigidTransform<double>::Identity()),
      std::exception, "Body 'mass' does not have geometry registered with it.");
}

GTEST_TEST(PropellerTest, SinglePropTest) {
  systems::DiagramBuilder<double> builder;

  const double timestep = 0.0;
  auto plant = builder.AddSystem<MultibodyPlant<double>>(timestep);

  // Confirm that it is sufficient to register with some SceneGraph.  (The
  // SceneGraph need not be in the diagram).
  geometry::SceneGraph<double> scene_graph;
  plant->RegisterAsSourceForSceneGraph(&scene_graph);

  const double mass = 1.0;
  const auto M_B = SpatialInertia<double>::MakeFromCentralInertia(
      mass, Vector3<double>::Zero(),
      mass * UnitInertia<double>::TriaxiallySymmetric(1.0));
  const RigidBody<double>& body = plant->AddRigidBody("mass", M_B);
  plant->Finalize();

  const double thrust_ratio = 3.2;
  const double moment_ratio = 0.4;
  auto prop = builder.AddSystem<Propeller<double>>(
      *plant, body, math::RigidTransform<double>::Identity(), thrust_ratio,
      moment_ratio);

  builder.Connect(prop->get_spatial_forces_output_port(),
                  plant->get_applied_spatial_force_input_port());
  builder.Connect(plant->get_geometry_poses_output_port(),
                  prop->get_geometry_poses_input_port());

  builder.ExportInput(prop->get_command_input_port(), "prop_command");

  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();

  for (const double command : {0.0, -1.4, 2.3}) {
    context->FixInputPort(0, Vector1d(command));
    const Eigen::VectorXd acceleration = diagram->EvalTimeDerivatives(*context)
                                            .get_generalized_velocity()
                                            .CopyToVector();
    EXPECT_EQ(acceleration.head<3>(),
              Eigen::Vector3d(0, 0, command * moment_ratio));
    EXPECT_EQ(acceleration.tail<3>(),
              Eigen::Vector3d(0, 0, command * thrust_ratio) +
                  plant->gravity_field().gravity_vector());
  }

  // Repeat the check, but with the body rotated and translated.
  plant->SetFreeBodyPose(
      &diagram->GetMutableSubsystemContext(*plant, context.get()), body,
      math::RigidTransform<double>(
          math::RotationMatrix<double>::MakeYRotation(M_PI / 2.0),
          Eigen::Vector3d::Ones()));

  for (const double command : {0.0, -1.4, 2.3}) {
    context->FixInputPort(0, Vector1d(command));
    const Eigen::VectorXd acceleration = diagram->EvalTimeDerivatives(*context)
                                            .get_generalized_velocity()
                                            .CopyToVector();
    EXPECT_TRUE(CompareMatrices(acceleration.head<3>(),
                                Eigen::Vector3d(command * moment_ratio, 0, 0),
                                1e-12));
    EXPECT_TRUE(CompareMatrices(acceleration.tail<3>(),
              Eigen::Vector3d(command * thrust_ratio, 0, 0) +
                  plant->gravity_field().gravity_vector(), 1e-12));
  }

  // Test that I can perform scalar conversion.
  diagram->ToAutoDiffXd();
  diagram->ToSymbolic();
}

GTEST_TEST(PropellerTest, BiRotorTest) {
  systems::DiagramBuilder<double> builder;

  const double timestep = 0.0;
  auto plant = builder.AddSystem<MultibodyPlant<double>>(timestep);

  // Confirm that it is sufficient to register with some SceneGraph.  (The
  // SceneGraph need not be in the diagram).
  geometry::SceneGraph<double> scene_graph;
  plant->RegisterAsSourceForSceneGraph(&scene_graph);

  const double mass = 1.0;
  const auto M_B = SpatialInertia<double>::MakeFromCentralInertia(
      mass, Vector3<double>::Zero(),
      mass * UnitInertia<double>::TriaxiallySymmetric(1.0));
  const RigidBody<double>& body = plant->AddRigidBody("mass", M_B);
  plant->Finalize();

  const double thrust_ratio = 3.2;
  const double moment_ratio = 0.4;
  const double arm_length = 0.67;  // distance from body origin to the prop.

  math::RigidTransform<double> T;
  T.set_translation(Eigen::Vector3d(arm_length, 0, 0));
  PropellerInfo prop1(*plant, body, T, thrust_ratio,
                                         moment_ratio);
  T.set_translation(Eigen::Vector3d(-arm_length, 0, 0));
  PropellerInfo prop2(*plant, body, T, thrust_ratio,
                                         moment_ratio);
  auto props = builder.AddSystem<Propeller<double>>(
      std::vector<PropellerInfo>({prop1, prop2}));

  builder.Connect(props->get_spatial_forces_output_port(),
                  plant->get_applied_spatial_force_input_port());
  builder.Connect(plant->get_geometry_poses_output_port(),
                  props->get_geometry_poses_input_port());

  builder.ExportInput(props->get_command_input_port(), "prop_command");

  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();

  const Eigen::Vector2d command(.3, .4);
  context->FixInputPort(0, command);
  const Eigen::VectorXd acceleration = diagram->EvalTimeDerivatives(*context)
                                            .get_generalized_velocity()
                                            .CopyToVector();
  EXPECT_TRUE(CompareMatrices(
      acceleration.head<3>(),
      Eigen::Vector3d(0, (command[1] - command[0]) * arm_length * thrust_ratio,
                      (command[0] + command[1]) * moment_ratio),
      1e-12));
  EXPECT_TRUE(CompareMatrices(
      acceleration.tail<3>(),
      Eigen::Vector3d(0, 0, (command[0] + command[1]) * thrust_ratio) +
          plant->gravity_field().gravity_vector(),
      1e-12));

  // Test that I can perform scalar conversion.
  diagram->ToAutoDiffXd();
  diagram->ToSymbolic();
}

}  // namespace
}  // namespace multibody
}  // namespace drake
