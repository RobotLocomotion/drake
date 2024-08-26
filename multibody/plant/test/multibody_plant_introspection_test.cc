#include <functional>
#include <limits>
#include <memory>
#include <set>
#include <tuple>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/planar_joint.h"
#include "drake/multibody/tree/revolute_joint.h"

namespace drake {

using multibody::Parser;

namespace multibody {
namespace {

// This test verifies we can introspect MultibodyPlant information regarding
// floating bodies:
//   1. What bodies in the plant are modeled with a free quaternion mobilizer.
//   2. Where in the state the dofs for a particular body are.
// We make an interesting world (from a multibody topological perspective)
// with two Atlas, a table, and a mug.
// The mathematical model of this system, after Finalize(), should contain
// three floating bodies, one for each Atlas robot and one for the mug.
GTEST_TEST(MultibodyPlantIntrospection, FloatingBodies) {
  const std::string atlas_url =
      "package://drake_models/atlas/atlas_convex_hull.urdf";

  const std::string table_url =
      "package://drake/examples/kuka_iiwa_arm/models/table/"
      "extra_heavy_duty_table_surface_only_collision.sdf";

  const std::string mug_url =
      "package://drake/examples/simple_gripper/simple_mug.sdf";

  MultibodyPlant<double> plant(0.0);

  // Load a model of a table for the environment around the robot.
  Parser parser(&plant);
  const ModelInstanceIndex robot_table_model =
      parser.AddModelsFromUrl(table_url).at(0);
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("link", robot_table_model));

  // Load two Atlas robots.
  const ModelInstanceIndex atlas_model1 =
      Parser(&plant, "atlas1").AddModelsFromUrl(atlas_url).at(0);
  const ModelInstanceIndex atlas_model2 =
      Parser(&plant, "atlas2").AddModelsFromUrl(atlas_url).at(0);
  const RigidBody<double>& pelvis1 =
      plant.GetBodyByName("pelvis", atlas_model1);
  const RigidBody<double>& pelvis2 =
      plant.GetBodyByName("pelvis", atlas_model2);

  // Add a floating mug.
  const ModelInstanceIndex mug_model = parser.AddModelsFromUrl(mug_url).at(0);
  const RigidBody<double>& mug = plant.GetBodyByName("simple_mug", mug_model);

  // Introspection of the underlying mathematical model is not available until
  // we call Finalize().
  DRAKE_EXPECT_THROWS_MESSAGE(
      mug.is_floating(),
      ".*The model to which this rigid body belongs must be finalized.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      mug.has_quaternion_dofs(),
      ".*The model to which this rigid body belongs must be finalized.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.GetFloatingBaseBodies(),
      "Pre-finalize calls to 'GetFloatingBaseBodies\\(\\)' are not allowed.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.GetUniqueFreeBaseBodyOrThrow(robot_table_model),
      "Pre-finalize calls to 'GetUniqueFreeBaseBodyOrThrow\\(\\)' are not "
      "allowed.*");

  plant.Finalize();

  // Sanity check model sizes.
  EXPECT_EQ(plant.num_velocities(atlas_model1), 36);
  EXPECT_EQ(plant.num_positions(atlas_model1), 37);
  EXPECT_EQ(plant.num_actuators(atlas_model1), 30);

  EXPECT_EQ(plant.num_velocities(atlas_model2), 36);
  EXPECT_EQ(plant.num_positions(atlas_model2), 37);

  EXPECT_EQ(plant.num_velocities(mug_model), 6);
  EXPECT_EQ(plant.num_positions(mug_model), 7);
  EXPECT_EQ(plant.num_actuators(mug_model), 0);

  // Assert that the mug and the two Atlas robot pelvises are floating and
  // modeled with quaternions.
  ASSERT_TRUE(mug.is_floating());
  ASSERT_TRUE(mug.has_quaternion_dofs());
  ASSERT_TRUE(pelvis1.is_floating());
  ASSERT_TRUE(pelvis1.has_quaternion_dofs());
  ASSERT_TRUE(pelvis2.is_floating());
  ASSERT_TRUE(pelvis2.has_quaternion_dofs());

  // Assert that the floating base of the Atlas robot are the pelvises.
  EXPECT_TRUE(plant.HasUniqueFreeBaseBody(atlas_model1));
  EXPECT_TRUE(plant.HasUniqueFreeBaseBody(atlas_model2));
  EXPECT_EQ(plant.GetUniqueFreeBaseBodyOrThrow(atlas_model1).index(),
            pelvis1.index());
  EXPECT_EQ(plant.GetUniqueFreeBaseBodyOrThrow(atlas_model2).index(),
            pelvis2.index());

  // The float mug is its own unique floating base.
  EXPECT_TRUE(plant.HasUniqueFreeBaseBody(mug_model));
  EXPECT_EQ(plant.GetUniqueFreeBaseBodyOrThrow(mug_model).index(), mug.index());

  // The "world" is not considered as a free body.
  EXPECT_FALSE(plant.world_body().is_floating());
  // Moreover, the "world" does not have a base body because by definition, a
  // base body is a body whose parent is the world.
  EXPECT_FALSE(plant.HasUniqueFreeBaseBody(world_model_instance()));

  // The table has been anchored to the world.
  EXPECT_FALSE(plant.GetBodyByName("link", robot_table_model).is_floating());
  EXPECT_FALSE(plant.HasUniqueFreeBaseBody(robot_table_model));
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.GetUniqueFreeBaseBodyOrThrow(robot_table_model),
      "Model " + plant.GetModelInstanceName(robot_table_model) +
          " has a unique base body, but it is not free.");

  // Retrieve floating bodies.
  std::unordered_set<BodyIndex> expected_floating_bodies(
      {pelvis1.index(), pelvis2.index(), mug.index()});
  auto floating_bodies = plant.GetFloatingBaseBodies();
  EXPECT_EQ(expected_floating_bodies, floating_bodies);

  // Verify state indexes for free bodies.
  // N.B. This test assumes DOFs are in a depth-first-traversal order with
  // mobilities assigned in the order mobilizers were added to the model. In
  // addition, we use our internal knowledge that free floating mobilizers are
  // assigned in the order bodies were added to the model. This assumption
  // implies that DOFs for the first tree are first, followed by DOFs for the
  // second tree, etc.
  const int atlas_nq = plant.num_positions(atlas_model1);
  const std::unordered_set<int> expected_floating_positions_start(
      {0, atlas_nq, 2 * atlas_nq});
  const std::unordered_set<int> floating_positions_start(
      {pelvis1.floating_positions_start(), pelvis2.floating_positions_start(),
       mug.floating_positions_start()});
  EXPECT_EQ(floating_positions_start, expected_floating_positions_start);

  const int atlas_nv = plant.num_velocities(atlas_model1);
  const std::unordered_set<int> expected_floating_velocities_start_in_v(
      {0, atlas_nv, 2 * atlas_nv});
  const std::unordered_set<int> floating_velocities_start_in_v(
      {pelvis1.floating_velocities_start_in_v(),
       pelvis2.floating_velocities_start_in_v(),
       mug.floating_velocities_start_in_v()});
  EXPECT_EQ(floating_velocities_start_in_v,
            expected_floating_velocities_start_in_v);
}

GTEST_TEST(MultibodyPlantIntrospection, NonUniqueBaseBody) {
  MultibodyPlant<double> plant(0.0);
  // Add two objects to the same (default) model instance and let one of them be
  // free.
  // To avoid unnecessary warnings/errors, use a non-zero spatial inertia.
  plant.AddRigidBody("free_body", default_model_instance(),
                     SpatialInertia<double>::MakeUnitary());
  const RigidBody<double>& fixed_body = plant.AddRigidBody(
      "fixed_body", default_model_instance(), SpatialInertia<double>::NaN());
  plant.WeldFrames(plant.world_frame(), fixed_body.body_frame());
  plant.Finalize();
  // Even though there is only one free body, the base body is not unique.
  EXPECT_FALSE(plant.HasUniqueFreeBaseBody(default_model_instance()));
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.GetUniqueFreeBaseBodyOrThrow(default_model_instance()),
      "Model " + plant.GetModelInstanceName(default_model_instance()) +
          " does not have a unique base body.");
}

GTEST_TEST(MultibodyPlantIntrospection, GetContinuousJoints) {
  MultibodyPlant<double> plant(0.0);
  const RigidBody<double>& first_body = plant.AddRigidBody("first_body");
  const RigidBody<double>& second_body = plant.AddRigidBody("second_body");
  const RigidBody<double>& third_body = plant.AddRigidBody("third_body");
  const RigidBody<double>& fourth_body = plant.AddRigidBody("fourth_body");

  // Add a planar joint without limits.
  plant.AddJoint<PlanarJoint>("first_joint", plant.world_body(), {}, first_body,
                              {}, Eigen::Vector3d::Zero());

  // Add a planar joint with limits.
  std::unique_ptr<PlanarJoint<double>> second_joint_ptr(new PlanarJoint<double>(
      "second_joint", first_body.body_frame(), second_body.body_frame(),
      Eigen::Matrix<double, 3, 1>::Zero()));
  second_joint_ptr->set_position_limits(Eigen::Vector3d{-1.0, -1.0, -1.0},
                                        Eigen::Vector3d{1.0, 1.0, 1.0});
  plant.AddJoint<PlanarJoint>(std::move(second_joint_ptr));

  // Add a revolute joint without limits.
  plant.AddJoint<RevoluteJoint>("third_joint", second_body, {}, third_body, {},
                                Eigen::Matrix<double, 3, 1>{1.0, 0.0, 0.0});

  // Add a revolute joint with limits.
  plant.AddJoint<RevoluteJoint>("fourth_joint", third_body, {}, fourth_body, {},
                                Eigen::Matrix<double, 3, 1>{1.0, 0.0, 0.0},
                                -1.0, 1.0);

  plant.Finalize();

  // The configuration is stored in a vector as follows:
  // 0: first_joint (x component, no limits)        --> not continuous revolute
  // 1: first_joint (y component, no limits)        --> not continuous revolute
  // 2: first_joint (revolute component, no limits) --> continuous revolute
  // 3: second_joint (x component, limits)          --> not continuous revolute
  // 4: second_joint (y component, limits)          --> not continuous revolute
  // 5: second_joint (revolute component, limits)   --> not continuous revolute
  // 6: third_joint (revolute, no limits)           --> continuous revolute
  // 7: fourth_joint (revolute, limits)             --> not continuous revolute
  // So the output of plant.GetContinuousRevoluteJointIndices() should be a
  // vector with entries 2 and 6.
  const std::vector<int> continuous_joint_indices =
      plant.GetContinuousRevoluteJointIndices();
  ASSERT_EQ(continuous_joint_indices.size(), 2);
  EXPECT_EQ(continuous_joint_indices[0], 2);
  EXPECT_EQ(continuous_joint_indices[1], 6);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
