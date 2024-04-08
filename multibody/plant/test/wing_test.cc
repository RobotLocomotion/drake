#include "drake/multibody/plant/wing.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector3d;

GTEST_TEST(WingTest, BasicTest) {
  MultibodyPlant<double> plant(0.0);
  Parser(&plant).AddModelsFromUrl("package://drake/multibody/models/box.urdf");
  plant.Finalize();

  const RigidBody<double>& body = plant.GetBodyByName("box");
  Wing<double> wing(body.index(), 1.0);

  EXPECT_EQ(wing.num_input_ports(), 4);
  EXPECT_EQ(wing.num_output_ports(), 2);
}

// A falling box with a flat plate wing.
GTEST_TEST(WingTest, FallingFlatPlate) {
  const double kRho = 1.3;
  const double kSurfaceArea = 4.15;
  systems::DiagramBuilder<double> builder;

  auto* plant = builder.AddSystem<MultibodyPlant<double>>(0);
  Parser(plant).AddModelsFromUrl("package://drake/multibody/models/box.urdf");
  plant->Finalize();

  const RigidBody<double>& body = plant->GetBodyByName("box");
  Wing<double>* wing = Wing<double>::AddToBuilder(
      &builder, plant, body.index(), kSurfaceArea,
      math::RigidTransform<double>::Identity(), kRho);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  systems::Context<double>& plant_context =
      plant->GetMyMutableContextFromRoot(context.get());
  const Vector6<double> vdot_gravity_only =
      plant->CalcGravityGeneralizedForces(plant_context);

  systems::Context<double>& wing_context =
      wing->GetMyMutableContextFromRoot(context.get());

  EXPECT_EQ(wing->get_aerodynamic_center_output_port().Eval(wing_context),
            Vector3d::Zero());

  {  // Zero velocity.
    const SpatialVelocity<double> V_WB(Vector3d::Zero(), Vector3d::Zero());
    plant->SetFreeBodySpatialVelocity(&plant_context, body, V_WB);

    const Vector6<double> vdot_expected = vdot_gravity_only;
    EXPECT_TRUE(CompareMatrices(plant->EvalTimeDerivatives(plant_context)
                                    .get_generalized_velocity()
                                    .CopyToVector(),
                                vdot_expected, 1e-14));
  }

  {  // Falling straight down.
    const double zdot = -1.23;
    const SpatialVelocity<double> V_WB(Vector3d::Zero(), Vector3d{0, 0, zdot});
    plant->SetFreeBodySpatialVelocity(&plant_context, body, V_WB);

    Vector6<double> vdot_expected = vdot_gravity_only;
    vdot_expected[5] += kRho * kSurfaceArea * zdot * zdot;
    EXPECT_TRUE(CompareMatrices(plant->EvalTimeDerivatives(plant_context)
                                    .get_generalized_velocity()
                                    .CopyToVector(),
                                vdot_expected, 1e-14));
  }

  {  // Falling straight up.
    const double zdot = 1.23;
    const SpatialVelocity<double> V_WB(Vector3d::Zero(), Vector3d{0, 0, zdot});
    plant->SetFreeBodySpatialVelocity(&plant_context, body, V_WB);

    Vector6<double> vdot_expected = vdot_gravity_only;
    vdot_expected[5] -= kRho * kSurfaceArea * zdot * zdot;
    EXPECT_TRUE(CompareMatrices(plant->EvalTimeDerivatives(plant_context)
                                    .get_generalized_velocity()
                                    .CopyToVector(),
                                vdot_expected, 1e-14));
  }

  {  // Falling straight down, with orientation.
    const math::RotationMatrix<double> R_WB(
        math::RollPitchYaw<double>(0.1, 0.2, 0.3));
    plant->SetFreeBodyPose(&plant_context, body,
                           math::RigidTransform<double>(R_WB));
    const double zdot = -1.23;
    const SpatialVelocity<double> V_WB(Vector3d::Zero(), Vector3d{0, 0, zdot});
    plant->SetFreeBodySpatialVelocity(&plant_context, body, V_WB);

    Vector6<double> vdot_expected = vdot_gravity_only;
    Vector3<double> v_WindBody_Wing = R_WB.transpose() * V_WB.translational();
    const double longitudinal_velocity_norm =
        Eigen::Vector2d(v_WindBody_Wing[0], v_WindBody_Wing[2]).norm();
    vdot_expected.tail<3>() +=
        R_WB * Vector3d{0, 0,
                        -kRho * kSurfaceArea * v_WindBody_Wing[2] *
                            longitudinal_velocity_norm};
    EXPECT_TRUE(CompareMatrices(plant->EvalTimeDerivatives(plant_context)
                                    .get_generalized_velocity()
                                    .CopyToVector(),
                                vdot_expected, 1e-14));
  }

  {  // Zero ground velocity, but with wind.
    const math::RotationMatrix<double> R_WWind(
        math::RollPitchYaw<double>(0.4, 0.5, 0.6));
    const double kWindMagnitude = 1.23;
    Vector3d v_wind = R_WWind * Vector3d{0, 0, kWindMagnitude};
    wing->get_wind_velocity_input_port().FixValue(&wing_context, v_wind);
    plant->SetFreeBodyPose(&plant_context, body,
                           math::RigidTransform<double>());
    const SpatialVelocity<double> V_WB(Vector3d::Zero(), Vector3d::Zero());
    plant->SetFreeBodySpatialVelocity(&plant_context, body, V_WB);

    Vector6<double> vdot_expected = vdot_gravity_only;
    // force in z = ρ S (normal⋅v) |v|, where the normal here is (0, 0, 1).
    const double longitudinal_velocity_norm =
        Eigen::Vector2d(v_wind[0], v_wind[2]).norm();
    vdot_expected.tail<3>() += Vector3d{
        0, 0, kRho * kSurfaceArea * v_wind[2] * longitudinal_velocity_norm};
    EXPECT_TRUE(CompareMatrices(plant->EvalTimeDerivatives(plant_context)
                                    .get_generalized_velocity()
                                    .CopyToVector(),
                                vdot_expected, 1e-14));
  }
}

// Test that I can perform scalar conversion.
GTEST_TEST(WingTest, ScalarConversion) {
  systems::DiagramBuilder<double> builder;

  auto* plant = builder.AddSystem<MultibodyPlant<double>>(0);
  Parser(plant).AddModelsFromUrl("package://drake/multibody/models/box.urdf");
  plant->Finalize();

  const RigidBody<double>& body = plant->GetBodyByName("box");
  Wing<double>::AddToBuilder(&builder, plant, body.index(), 1.0);

  auto diagram = builder.Build();

  diagram->ToAutoDiffXd();
  diagram->ToSymbolic();
}

// Test fix for
// https://stackoverflow.com/a/72160960/7829525
GTEST_TEST(WingTest, DerivativesAtZeroVelocity) {
  const double kSurfaceArea = 4.15;
  systems::DiagramBuilder<double> builder;

  auto* plant = builder.AddSystem<MultibodyPlant<double>>(0);
  Parser(plant).AddModelsFromUrl("package://drake/multibody/models/box.urdf");
  plant->Finalize();
  plant->set_name("plant");

  const RigidBody<double>& body = plant->GetBodyByName("box");
  Wing<double>::AddToBuilder(&builder, plant, body.index(), kSurfaceArea,
                             math::RigidTransform<double>::Identity());

  auto diagram = builder.Build();
  auto diagram_ad = systems::System<double>::ToAutoDiffXd(*diagram);
  auto context_ad = diagram_ad->CreateDefaultContext();

  const MultibodyPlant<AutoDiffXd>* plant_ad =
      dynamic_cast<const MultibodyPlant<AutoDiffXd>*>(
          &diagram_ad->GetSubsystemByName("plant"));
  EXPECT_TRUE(plant_ad != nullptr);
  systems::Context<AutoDiffXd>& plant_context_ad =
      plant_ad->GetMyMutableContextFromRoot(context_ad.get());
  const RigidBody<AutoDiffXd>& body_ad = plant_ad->GetBodyByName("box");
  const SpatialVelocity<AutoDiffXd> V_WB(
      math::InitializeAutoDiff(Vector6d::Zero()));
  plant_ad->SetFreeBodySpatialVelocity(&plant_context_ad, body_ad, V_WB);

  const Vector6<AutoDiffXd> vdot_gravity_only =
      plant_ad->CalcGravityGeneralizedForces(plant_context_ad);
  const Vector6<AutoDiffXd> vdot =
      plant_ad->EvalTimeDerivatives(plant_context_ad)
          .get_generalized_velocity()
          .CopyToVector();
  EXPECT_TRUE(CompareMatrices(vdot, vdot_gravity_only, 1e-14));
  // This next line would fail before the fix:
  EXPECT_FALSE(math::ExtractGradient(vdot).hasNaN());
}

}  // namespace
}  // namespace multibody
}  // namespace drake
