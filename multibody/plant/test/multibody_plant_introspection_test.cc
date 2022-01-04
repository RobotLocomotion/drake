#include <functional>
#include <limits>
#include <memory>
#include <set>
#include <tuple>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

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
  const std::string atlas_path =
      FindResourceOrThrow("drake/examples/atlas/urdf/atlas_convex_hull.urdf");

  const std::string table_sdf_path = FindResourceOrThrow(
      "drake/examples/kuka_iiwa_arm/models/table/"
      "extra_heavy_duty_table_surface_only_collision.sdf");

  const std::string mug_sdf_path =
      FindResourceOrThrow("drake/examples/simple_gripper/simple_mug.sdf");

  MultibodyPlant<double> plant(0.0);

  // Load a model of a table for the environment around the robot.
  Parser parser(&plant);
  const ModelInstanceIndex robot_table_model =
      parser.AddModelFromFile(table_sdf_path, "robot_table");
  plant.WeldFrames(plant.world_frame(),
                   plant.GetFrameByName("link", robot_table_model));

  // Load two Atlas robots.
  const ModelInstanceIndex atlas_model1 =
      parser.AddModelFromFile(atlas_path, "Atlas1");
  const ModelInstanceIndex atlas_model2 =
      parser.AddModelFromFile(atlas_path, "Atlas2");
  const Body<double>& pelvis1 = plant.GetBodyByName("pelvis", atlas_model1);
  const Body<double>& pelvis2 = plant.GetBodyByName("pelvis", atlas_model2);

  // Add a floating mug.
  const ModelInstanceIndex mug_model = parser.AddModelFromFile(mug_sdf_path);
  const Body<double>& mug = plant.GetBodyByName("main_body", mug_model);

  // Introspection of the underlying mathematical model is not available until
  // we call Finalize().
  DRAKE_EXPECT_THROWS_MESSAGE(
      mug.is_floating(), std::runtime_error,
      ".*The model to which this body belongs must be finalized.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      mug.has_quaternion_dofs(), std::runtime_error,
      ".*The model to which this body belongs must be finalized.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.GetFloatingBaseBodies(), std::logic_error,
      "Pre-finalize calls to 'GetFloatingBaseBodies\\(\\)' are not allowed.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.GetUniqueFreeBaseBodyOrThrow(robot_table_model),
      "Pre-finalize calls to 'GetUniqueFreeBaseBodyOrThrow\\(\\)' are not "
      "allowed.*");

  plant.Finalize();

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

  const int nq = plant.num_positions();
  const int atlas_nv = plant.num_velocities(atlas_model1);
  const std::unordered_set<int> expected_floating_velocities_start(
      {nq, nq + atlas_nv, nq + 2 * atlas_nv});
  const std::unordered_set<int> floating_velocities_start(
      {pelvis1.floating_velocities_start(), pelvis2.floating_velocities_start(),
       mug.floating_velocities_start()});
  EXPECT_EQ(floating_velocities_start, expected_floating_velocities_start);
}

GTEST_TEST(MultibodyPlantIntrospection, NonUniqueBaseBody) {
  MultibodyPlant<double> plant(0.0);
  // Add two objects to the same (default) model instance and let one of them be
  // free.
  plant.AddRigidBody("free_body", default_model_instance(),
                     SpatialInertia<double>());
  const Body<double>& fixed_body = plant.AddRigidBody(
      "fixed_body", default_model_instance(), SpatialInertia<double>());
  plant.WeldFrames(plant.world_frame(), fixed_body.body_frame());
  plant.Finalize();
  // Even though there is only one free body, the base body is not unique.
  EXPECT_FALSE(plant.HasUniqueFreeBaseBody(default_model_instance()));
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant.GetUniqueFreeBaseBodyOrThrow(default_model_instance()),
      "Model " + plant.GetModelInstanceName(default_model_instance()) +
          " does not have a unique base body.");
}
}  // namespace
}  // namespace multibody
}  // namespace drake
