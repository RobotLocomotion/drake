#include <limits>
#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/benchmarks/acrobot/acrobot.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {

using Eigen::Vector3d;
using math::RigidTransform;
using math::RotationMatrix;
using multibody::benchmarks::Acrobot;
using multibody::benchmarks::acrobot::AcrobotParameters;
using multibody::benchmarks::acrobot::MakeAcrobotPlant;
using std::unique_ptr;
using systems::BasicVector;
using systems::Context;

namespace multibody {
namespace {

GTEST_TEST(MultibodyPlantTest, BushingParameters) {
  // Add a plant with a few rigid bodies.
  MultibodyPlant<double> plant(0.0);

  const double sphere_radius = 1.0;
  const double sphere_mass = 2.5;
  const Vector3d sphere_com(0, 0, 0);
  const UnitInertia<double> sphere_unit_inertia =
      UnitInertia<double>::SolidSphere(sphere_radius);

  const RigidBody<double>& sphere1 = plant.AddRigidBody(
      "sphere1",
      SpatialInertia<double>(sphere_mass, sphere_com, sphere_unit_inertia));

  const RigidBody<double>& sphere2 = plant.AddRigidBody(
      "sphere2",
      SpatialInertia<double>(sphere_mass, sphere_com, sphere_unit_inertia));

  const Vector3<double> torque_stiffness(100, 100, 100);
  const Vector3<double> torque_damping(5, 5, 5);
  const Vector3<double> force_stiffness(100, 100, 100);
  const Vector3<double> force_damping(5, 5, 5);

  const LinearBushingRollPitchYaw<double>& bushing =
      plant.AddForceElement<LinearBushingRollPitchYaw>(
          sphere1.body_frame(), sphere2.body_frame(), torque_stiffness,
          torque_damping, force_stiffness, force_damping);

  plant.Finalize();

  // Create a default context.
  auto context = plant.CreateDefaultContext();

  // Verify default parameters exist and are correct.
  const Vector3<double> default_torque_stiffness =
      bushing.GetTorqueStiffnessConstants(*context);
  const Vector3<double> default_torque_damping =
      bushing.GetTorqueDampingConstants(*context);
  const Vector3<double> default_force_stiffness =
      bushing.GetForceStiffnessConstants(*context);
  const Vector3<double> default_force_damping =
      bushing.GetForceDampingConstants(*context);

  EXPECT_TRUE(CompareMatrices(torque_stiffness, default_torque_stiffness));
  EXPECT_TRUE(CompareMatrices(torque_damping, default_torque_damping));
  EXPECT_TRUE(CompareMatrices(force_stiffness, default_force_stiffness));
  EXPECT_TRUE(CompareMatrices(force_damping, default_force_damping));

  // Change parameters.
  const Vector3<double> new_torque_stiffness(50, 50, 50);
  const Vector3<double> new_torque_damping(2, 2, 2);
  const Vector3<double> new_force_stiffness(50, 50, 50);
  const Vector3<double> new_force_damping(2, 2, 2);

  bushing.SetTorqueStiffnessConstants(context.get(), new_torque_stiffness);
  bushing.SetTorqueDampingConstants(context.get(), new_torque_damping);
  bushing.SetForceStiffnessConstants(context.get(), new_force_stiffness);
  bushing.SetForceDampingConstants(context.get(), new_force_damping);

  // Verify parameter changes propogate.
  const Vector3<double> new_default_torque_stiffness =
      bushing.GetTorqueStiffnessConstants(*context);
  const Vector3<double> new_default_torque_damping =
      bushing.GetTorqueDampingConstants(*context);
  const Vector3<double> new_default_force_stiffness =
      bushing.GetForceStiffnessConstants(*context);
  const Vector3<double> new_default_force_damping =
      bushing.GetForceDampingConstants(*context);

  EXPECT_TRUE(
      CompareMatrices(new_torque_stiffness, new_default_torque_stiffness));
  EXPECT_TRUE(CompareMatrices(new_torque_damping, new_default_torque_damping));
  EXPECT_TRUE(
      CompareMatrices(new_force_stiffness, new_default_force_stiffness));
  EXPECT_TRUE(CompareMatrices(new_force_damping, new_default_force_damping));
}

GTEST_TEST(MultibodyPlantTest, RigidBodyParameters) {
  // Add a plant with a few rigid bodies.
  MultibodyPlant<double> plant(0.0);

  const double sphere_radius = 1.0;
  const double sphere_mass = 2.5;
  const Vector3d sphere_com(0, 0, 0);
  const UnitInertia<double> sphere_unit_inertia =
      UnitInertia<double>::SolidSphere(sphere_radius);
  const RigidBody<double>& sphere = plant.AddRigidBody(
      "sphere",
      SpatialInertia<double>(sphere_mass, sphere_com, sphere_unit_inertia));

  const double cube_length = 2.0;
  const double cube_mass = 5.0;
  const Vector3d cube_com(0, 0, 0);
  const UnitInertia<double> cube_unit_inertia =
      UnitInertia<double>::SolidBox(cube_length, cube_length, cube_length);
  const RigidBody<double>& cube = plant.AddRigidBody(
      "cube", SpatialInertia<double>(cube_mass, cube_com, cube_unit_inertia));

  plant.Finalize();

  // Create a default context.
  auto context = plant.CreateDefaultContext();

  // Verify default parameters exist and are correct.
  const double sphere_mass_in_context = sphere.get_mass(*context);
  const Vector3<double> sphere_com_in_context =
      sphere.CalcCenterOfMassInBodyFrame(*context);
  const SpatialInertia<double> sphere_inertia_in_context =
      sphere.CalcSpatialInertiaInBodyFrame(*context);

  const double cube_mass_in_context = cube.get_mass(*context);
  const Vector3<double> cube_com_in_context =
      cube.CalcCenterOfMassInBodyFrame(*context);
  const SpatialInertia<double> cube_inertia_in_context =
      cube.CalcSpatialInertiaInBodyFrame(*context);

  EXPECT_EQ(sphere_mass_in_context, sphere_mass);
  EXPECT_EQ(sphere_inertia_in_context.get_mass(), sphere_mass);
  EXPECT_TRUE(CompareMatrices(sphere_com_in_context, sphere_com));
  EXPECT_TRUE(CompareMatrices(sphere_inertia_in_context.get_com(), sphere_com));
  EXPECT_TRUE(CompareMatrices(
      sphere_inertia_in_context.get_unit_inertia().get_moments(),
      sphere_unit_inertia.get_moments()));
  EXPECT_TRUE(CompareMatrices(
      sphere_inertia_in_context.get_unit_inertia().get_products(),
      sphere_unit_inertia.get_products()));

  EXPECT_EQ(cube_mass_in_context, cube_mass);
  EXPECT_EQ(cube_inertia_in_context.get_mass(), cube_mass);
  EXPECT_TRUE(CompareMatrices(cube_com_in_context, cube_com));
  EXPECT_TRUE(CompareMatrices(cube_inertia_in_context.get_com(), cube_com));
  EXPECT_TRUE(
      CompareMatrices(cube_inertia_in_context.get_unit_inertia().get_moments(),
                      cube_unit_inertia.get_moments()));
  EXPECT_TRUE(
      CompareMatrices(cube_inertia_in_context.get_unit_inertia().get_products(),
                      cube_unit_inertia.get_products()));

  // Change parameters.
  const double new_sphere_radius = 1.5;
  const double new_sphere_mass = 3.9;
  const Vector3d new_sphere_com(0, 0, 0);
  const UnitInertia<double> new_sphere_unit_inertia =
      UnitInertia<double>::SolidSphere(new_sphere_radius);

  const double new_cube_length = 1.3;
  const double new_cube_mass = 3.0;
  const Vector3d new_cube_com(0, 0, 0);
  const UnitInertia<double> new_cube_unit_inertia =
      UnitInertia<double>::SolidBox(new_cube_length, new_cube_length,
                                    new_cube_length);

  SpatialInertia<double> new_sphere_params(new_sphere_mass, new_sphere_com,
                                           new_sphere_unit_inertia);

  SpatialInertia<double> new_cube_params(new_cube_mass, new_cube_com,
                                         new_cube_unit_inertia);

  sphere.SetSpatialInertiaInBodyFrame(context.get(), new_sphere_params);
  cube.SetSpatialInertiaInBodyFrame(context.get(), new_cube_params);

  // Verify parameters propagate.
  const double new_sphere_mass_in_context = sphere.get_mass(*context);
  const Vector3<double> new_sphere_com_in_context =
      sphere.CalcCenterOfMassInBodyFrame(*context);
  const SpatialInertia<double> new_sphere_inertia_in_context =
      sphere.CalcSpatialInertiaInBodyFrame(*context);

  const double new_cube_mass_in_context = cube.get_mass(*context);
  const Vector3<double> new_cube_com_in_context =
      cube.CalcCenterOfMassInBodyFrame(*context);
  const SpatialInertia<double> new_cube_inertia_in_context =
      cube.CalcSpatialInertiaInBodyFrame(*context);

  EXPECT_EQ(new_sphere_mass_in_context, new_sphere_mass);
  EXPECT_EQ(new_sphere_inertia_in_context.get_mass(), new_sphere_mass);
  EXPECT_TRUE(CompareMatrices(new_sphere_com_in_context, new_sphere_com));
  EXPECT_TRUE(
      CompareMatrices(new_sphere_inertia_in_context.get_com(), new_sphere_com));
  EXPECT_TRUE(CompareMatrices(
      new_sphere_inertia_in_context.get_unit_inertia().get_moments(),
      new_sphere_unit_inertia.get_moments()));
  EXPECT_TRUE(CompareMatrices(
      new_sphere_inertia_in_context.get_unit_inertia().get_products(),
      new_sphere_unit_inertia.get_products()));

  EXPECT_EQ(new_cube_mass_in_context, new_cube_mass);
  EXPECT_EQ(new_cube_inertia_in_context.get_mass(), new_cube_mass);
  EXPECT_TRUE(CompareMatrices(new_cube_com_in_context, new_cube_com));
  EXPECT_TRUE(
      CompareMatrices(new_cube_inertia_in_context.get_com(), new_cube_com));
  EXPECT_TRUE(CompareMatrices(
      new_cube_inertia_in_context.get_unit_inertia().get_moments(),
      new_cube_unit_inertia.get_moments()));
  EXPECT_TRUE(CompareMatrices(
      new_cube_inertia_in_context.get_unit_inertia().get_products(),
      new_cube_unit_inertia.get_products()));
}

GTEST_TEST(MultibodyPlantTest, AutoDiffAcrobotParameters) {
  const double kTolerance = 5 * std::numeric_limits<double>::epsilon();

  // Create an Acrobot plant with autodiff parameters for length and mass.
  const double m1 = 2.0;
  const double m2 = 4.0;
  const double l1 = 1.0;
  const double l2 = 2.0;
  const double lc1 = 0.5 * l1;
  const double lc2 = 0.5 * l2;
  const double Gc1 = (1.0 / 12.0) * l1 * l1;
  const double Gc2 = (1.0 / 12.0) * l2 * l2;
  const double Ic1 = m1 * Gc1;
  const double Ic2 = m2 * Gc2;

  const AcrobotParameters params(m1, m2, l1, l2, lc1, lc2, Ic1, Ic2, 0.0, 0.0,
                                 9.81);
  unique_ptr<MultibodyPlant<double>> plant = MakeAcrobotPlant(params, true);

  // Create a default context and set state.
  unique_ptr<Context<double>> context = plant->CreateDefaultContext();

  const RevoluteJoint<double>& shoulder_joint =
      plant->GetJointByName<RevoluteJoint>(params.shoulder_joint_name());
  const RevoluteJoint<double>& elbow_joint =
      plant->GetJointByName<RevoluteJoint>(params.elbow_joint_name());
  shoulder_joint.set_angle(context.get(), 0.0);
  elbow_joint.set_angle(context.get(), 0.0);

  // Scalar convert the plant to AutoDiffXd.
  unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff =
      systems::System<double>::ToAutoDiffXd(*plant);
  unique_ptr<Context<AutoDiffXd>> context_autodiff =
      plant_autodiff->CreateDefaultContext();
  context_autodiff->SetTimeStateAndParametersFrom(*context);

  // Set up our parameters as the independent variables.
  // Differentiable parameters for the acrobot using only inertial parameters.
  const AutoDiffXd m1_ad(m1, Vector3d(1, 0, 0));
  const AutoDiffXd m2_ad(m2, Vector3d(0, 1, 0));
  const AutoDiffXd l2_ad(l2, Vector3d(0, 0, 1));
  const AutoDiffXd lc2_ad = 0.5 * l2_ad;
  const AutoDiffXd Gc2_ad = (1.0 / 12.0) * l2_ad * l2_ad;

  // Frame L1's origin is located at the shoulder outboard frame.
  const Vector3<AutoDiffXd> p_L1L1cm = -lc1 * Vector3d::UnitZ();
  // Frame L2's origin is located at the elbow outboard frame.
  const Vector3<AutoDiffXd> p_L2L2cm = -lc2_ad * Vector3d::UnitZ();

  // Define each link's spatial inertia about their respective COM.
  UnitInertia<AutoDiffXd> Gc1_Bcm =
      UnitInertia<AutoDiffXd>::StraightLine(Gc1, Vector3d::UnitZ());
  SpatialInertia<AutoDiffXd> M1_L1o =
      SpatialInertia<AutoDiffXd>::MakeFromCentralInertia(m1_ad, p_L1L1cm,
                                                         Gc1_Bcm * m1_ad);

  UnitInertia<AutoDiffXd> Gc2_Bcm =
      UnitInertia<AutoDiffXd>::StraightLine(Gc2_ad, Vector3d::UnitZ());
  SpatialInertia<AutoDiffXd> M2_L2o =
      SpatialInertia<AutoDiffXd>::MakeFromCentralInertia(m2_ad, p_L2L2cm,
                                                         Gc2_Bcm * m2_ad);

  plant_autodiff->GetRigidBodyByName(params.link1_name())
      .SetSpatialInertiaInBodyFrame(context_autodiff.get(), M1_L1o);

  plant_autodiff->GetRigidBodyByName(params.link2_name())
      .SetSpatialInertiaInBodyFrame(context_autodiff.get(), M2_L2o);

  // Take the derivative of the mass matrix w.r.t. length.
  Matrix2<AutoDiffXd> mass_matrix;
  plant_autodiff->CalcMassMatrix(*context_autodiff, &mass_matrix);

  const auto& mass_matrix_grad = math::autoDiffToGradientMatrix(mass_matrix);

  // Verify numerical derivative matches analytic solution.
  // In the starting configuration q = (0, 0).
  //
  //   c1 = cos(q[0]) = 1
  //   s1 = sin(q[0]) = 0
  //   c2 = cos(q[1]) = 1
  //   s2 = sin(q[1]) = 0
  //
  //  Moments of Inertia, taken about the pivots:
  //
  //    I₁ = 4m₁Ic₁ = (1/3)m₁l₁²
  //    I₂ = 4m₂Ic₂ = (1/3)m₂l₂²
  //
  // Moment of inertia at the shoulder joint origin.
  const double I1 = 4 * Ic1;
  // Moment of inertia at the elbow joint origin.
  const double I2 = 4 * Ic2;

  // Analytic Mass Matrix, M:
  //
  //  [ I₁ + I₂ + m₂l₁² + 2m₂l₁lc₂c₂   I₂ + m₂l₁lc₂c₂ ]
  //  [      I₂ + m₂l₁lc₂c₂                 I₂        ]
  Matrix2<double> analytic_mass_matrix;
  analytic_mass_matrix << I1 + I2 + m2 * l1 * l1 + 2 * m2 * l1 * lc2,
      I2 + m2 * l1 * lc2, I2 + m2 * l1 * lc2, I2;
  EXPECT_TRUE(CompareMatrices(mass_matrix, analytic_mass_matrix, kTolerance,
                              MatrixCompareType::relative));

  // Analytic ∂M/∂m₁:
  // [ (1/3)l₁²      0 ]
  // [     0         0 ]
  Vector4<double> analytic_mass_matrix_partial_m1;
  analytic_mass_matrix_partial_m1 << (1.0 / 3.0) * l1 * l1, 0.0, 0.0, 0.0;
  EXPECT_TRUE(CompareMatrices(mass_matrix_grad.col(0),
                              analytic_mass_matrix_partial_m1, kTolerance,
                              MatrixCompareType::relative));

  // Analytic ∂M/∂m₂:
  // [ (1/3)l₂² + l₁² + 2l₁lc₂c₂     (1/3)l₂² + l₁lc₂c₂ ]
  // [    (1/3)l₂² + l₁lc₂c₂               (1/3)l₂²     ]
  Vector4<double> analytic_mass_matrix_partial_m2;
  analytic_mass_matrix_partial_m2
      << (1.0 / 3.0) * l2 * l2 + l1 * l1 + 2 * l1 * lc2,
      (1.0 / 3.0) * l2 * l2 + l1 * lc2, (1.0 / 3.0) * l2 * l2 + l1 * lc2,
      (1.0 / 3.0) * l2 * l2;
  EXPECT_TRUE(CompareMatrices(mass_matrix_grad.col(1),
                              analytic_mass_matrix_partial_m2, kTolerance,
                              MatrixCompareType::relative));

  // Analytic ∂M/∂l₂:
  // [   (2/3)m₂l₂ + m₂l₁       (2/3)m₂l₂ + (1/2)m₂l₁ ]
  // [ (2/3)m₂l₂ + (1/2)m₂l₁          (2/3)m₂l₂       ]
  Vector4<double> analytic_mass_matrix_partial_l2;
  analytic_mass_matrix_partial_l2 << (2.0 / 3.0) * m2 * l2 + m2 * l1,
      (2.0 / 3.0) * m2 * l2 + 0.5 * m2 * l1,
      (2.0 / 3.0) * m2 * l2 + 0.5 * m2 * l1, (2.0 / 3.0) * m2 * l2;
  EXPECT_TRUE(CompareMatrices(mass_matrix_grad.col(2),
                              analytic_mass_matrix_partial_l2, kTolerance,
                              MatrixCompareType::relative));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
