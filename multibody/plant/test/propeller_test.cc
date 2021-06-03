#include "drake/multibody/plant/propeller.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace {

// Test that we can create a propeller and wire it up in a diagram to produce
// the forces/moments that we expect.  Also confirms that the entire diagram can
// perform scalar conversion.
GTEST_TEST(PropellerTest, SinglePropTest) {
  systems::DiagramBuilder<double> builder;

  const double timestep = 0.0;
  auto plant = builder.AddSystem<MultibodyPlant<double>>(timestep);

  const double mass = 1.0;
  const auto M_B =
      SpatialInertia<double>(mass, Vector3<double>::Zero(),
                             UnitInertia<double>::TriaxiallySymmetric(1.0));
  const RigidBody<double>& body = plant->AddRigidBody("mass", M_B);
  plant->Finalize();

  const double thrust_ratio = 3.2;
  const double moment_ratio = 0.4;
  // Add a propeller at (0,1,0) pointing along the negative y axis.
  auto prop = builder.AddSystem<Propeller<double>>(
      body.index(),
      math::RigidTransform<double>(
          math::RotationMatrix<double>::MakeXRotation(M_PI / 2.0),
          Eigen::Vector3d(0, 1.0, 0.0)),
      thrust_ratio, moment_ratio);
  EXPECT_EQ(prop->num_propellers(), 1);

  builder.Connect(prop->get_spatial_forces_output_port(),
                  plant->get_applied_spatial_force_input_port());
  builder.Connect(plant->get_body_poses_output_port(),
                  prop->get_body_poses_input_port());

  builder.ExportInput(prop->get_command_input_port(), "prop_command");

  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();

  for (const double command : {0.0, -1.4, 2.3}) {
    diagram->get_input_port(0).FixValue(context.get(), command);
    const Eigen::VectorXd acceleration = diagram->EvalTimeDerivatives(*context)
                                             .get_generalized_velocity()
                                             .CopyToVector();
    EXPECT_TRUE(CompareMatrices(acceleration.head<3>(),
                                Eigen::Vector3d(0, -command * moment_ratio, 0),
                                1e-14));
    EXPECT_TRUE(CompareMatrices(acceleration.tail<3>(),
                                Eigen::Vector3d(0, -command * thrust_ratio, 0) +
                                    plant->gravity_field().gravity_vector(),
                                1e-14));
  }

  // Repeat the check, but with the body rotated and translated.
  // Propeller is now at (1,2,1), pointing along the positive x-axis.
  plant->SetFreeBodyPose(
      &plant->GetMyMutableContextFromRoot(context.get()), body,
      math::RigidTransform<double>(
          math::RotationMatrix<double>::MakeZRotation(M_PI / 2.0),
          Eigen::Vector3d::Ones()));

  for (const double command : {0.0, -1.4, 2.3}) {
    diagram->get_input_port(0).FixValue(context.get(), command);
    const Eigen::VectorXd acceleration = diagram->EvalTimeDerivatives(*context)
                                             .get_generalized_velocity()
                                             .CopyToVector();
    EXPECT_TRUE(CompareMatrices(acceleration.head<3>(),
                                Eigen::Vector3d(command * moment_ratio, 0, 0),
                                1e-14));
    EXPECT_TRUE(CompareMatrices(acceleration.tail<3>(),
                                Eigen::Vector3d(command * thrust_ratio, 0, 0) +
                                    plant->gravity_field().gravity_vector(),
                                1e-14));
  }

  // Test that I can perform scalar conversion.
  diagram->ToAutoDiffXd();
  diagram->ToSymbolic();
}

// Test that we can add multiple propellers to a MultibodyPlant (via a single
// Propeller system), and generate the forces/moments we expect.  Also confirms
// that the entire diagram can perform scalar conversion.
GTEST_TEST(PropellerTest, BiRotorTest) {
  systems::DiagramBuilder<double> builder;

  const double timestep = 0.0;
  auto plant = builder.AddSystem<MultibodyPlant<double>>(timestep);

  const double mass = 1.0;
  const auto M_B =
      SpatialInertia<double>(mass, Vector3<double>::Zero(),
                             UnitInertia<double>::TriaxiallySymmetric(1.0));
  const RigidBody<double>& body = plant->AddRigidBody("mass", M_B);
  plant->Finalize();

  const double thrust_ratio = 3.2;
  const double moment_ratio = 0.4;
  const double arm_length = 0.67;  // distance from body origin to the prop.

  math::RigidTransform<double> X_BP;
  X_BP.set_translation(Eigen::Vector3d(arm_length, 0, 0));
  PropellerInfo prop1(body.index(), X_BP, thrust_ratio, moment_ratio);
  X_BP.set_translation(Eigen::Vector3d(-arm_length, 0, 0));
  PropellerInfo prop2(body.index(), X_BP, thrust_ratio, moment_ratio);
  auto props = builder.AddSystem<Propeller<double>>(
      std::vector<PropellerInfo>({prop1, prop2}));
  EXPECT_EQ(props->num_propellers(), 2);

  builder.Connect(props->get_spatial_forces_output_port(),
                  plant->get_applied_spatial_force_input_port());
  builder.Connect(plant->get_body_poses_output_port(),
                  props->get_body_poses_input_port());

  builder.ExportInput(props->get_command_input_port(), "prop_command");

  auto diagram = builder.Build();

  auto context = diagram->CreateDefaultContext();

  const Eigen::Vector2d command(.3, .4);
  diagram->get_input_port(0).FixValue(context.get(), command);
  const Eigen::VectorXd acceleration = diagram->EvalTimeDerivatives(*context)
                                            .get_generalized_velocity()
                                            .CopyToVector();
  EXPECT_TRUE(CompareMatrices(
      acceleration.head<3>(),
      Eigen::Vector3d(0, (command[1] - command[0]) * arm_length * thrust_ratio,
                      (command[0] + command[1]) * moment_ratio),
      1e-14));
  EXPECT_TRUE(CompareMatrices(
      acceleration.tail<3>(),
      Eigen::Vector3d(0, 0, (command[0] + command[1]) * thrust_ratio) +
          plant->gravity_field().gravity_vector(),
      1e-14));

  // Verify that the implicit dynamics match the continuous ones.
  Eigen::VectorXd residual = diagram->AllocateImplicitTimeDerivativesResidual();
  auto derivatives = diagram->AllocateTimeDerivatives();
  diagram->CalcTimeDerivatives(*context, derivatives.get());
  diagram->CalcImplicitTimeDerivativesResidual(*context, *derivatives,
                                               &residual);
  EXPECT_TRUE(CompareMatrices(residual, Eigen::VectorXd::Zero(13), 1e-15));

  // Test that I can perform scalar conversion.
  diagram->ToAutoDiffXd();
  diagram->ToSymbolic();
}

}  // namespace
}  // namespace multibody
}  // namespace drake
