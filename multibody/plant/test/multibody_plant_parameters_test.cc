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

}  // namespace
}  // namespace multibody
}  // namespace drake
