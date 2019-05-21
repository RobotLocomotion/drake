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
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_search.h"

namespace drake {

using geometry::SceneGraph;
using multibody::Parser;

namespace multibody {
namespace {

// We make an interesting world (from a multibody topological perspective)
// with two Atlas, a table, and a mug.
class MultibodyPlantIntrospection: public ::testing::Test {
 public:
  void SetUp() override {
    atlas_path =
        FindResourceOrThrow("drake/examples/atlas/urdf/atlas_convex_hull.urdf");

    table_sdf_path = FindResourceOrThrow(
        "drake/examples/kuka_iiwa_arm/models/table/"
        "extra_heavy_duty_table_surface_only_collision.sdf");

    mug_sdf_path =
        FindResourceOrThrow("drake/examples/simple_gripper/simple_mug.sdf");

    // Load a model of a table for the environment around the robot.
    Parser parser(&plant, &scene_graph);
    robot_table_model = parser.AddModelFromFile(table_sdf_path, "robot_table");
    plant.WeldFrames(plant.world_frame(),
                     plant.GetFrameByName("link", robot_table_model));

    // Load two Atlas robots.
    atlas_model1 = parser.AddModelFromFile(atlas_path, "Atlas1");
    atlas_model2 = parser.AddModelFromFile(atlas_path, "Atlas2");
    pelvis1 = &plant.GetBodyByName("pelvis", atlas_model1);
    pelvis2 = &plant.GetBodyByName("pelvis", atlas_model2);

    // Add a floating mug.
    mug_model = parser.AddModelFromFile(mug_sdf_path);
    mug = &plant.GetBodyByName("main_body", mug_model);
  }

 protected:
  std::string atlas_path;
  std::string table_sdf_path;
  std::string mug_sdf_path;
  MultibodyPlant<double> plant;
  SceneGraph<double> scene_graph;
  ModelInstanceIndex robot_table_model;
  ModelInstanceIndex atlas_model1;
  ModelInstanceIndex atlas_model2;
  const Body<double>* pelvis1;
  const Body<double>* pelvis2;
  ModelInstanceIndex mug_model;
  const Body<double>* mug;
};

TEST_F(MultibodyPlantIntrospection, NoIntrospectionBeforeFinalize) {
 // Introspection of the underlying mathematical model is not available until
 // we call Finalize().
 DRAKE_EXPECT_THROWS_MESSAGE(
     mug->is_floating(), std::runtime_error,
     ".*The model to which this body belongs must be finalized.*");
 DRAKE_EXPECT_THROWS_MESSAGE(
     mug->has_quaternion_dofs(), std::runtime_error,
     ".*The model to which this body belongs must be finalized.*");
 DRAKE_EXPECT_THROWS_MESSAGE(
     plant.GetFloatingBaseBodies(), std::logic_error,
     "Pre-finalize calls to 'GetFloatingBaseBodies\\(\\)' are not allowed.*");
 DRAKE_EXPECT_THROWS_MESSAGE(
     MultibodySearch<double>(&plant, &scene_graph), std::logic_error,
     "Searching on a MultibodyPlant before `Finalize` is not allowed.*");
}

// This test verifies we can introspect MultibodyPlant information regarding
// floating bodies:
//   1. What bodies in the plant are modeled with a free quaternion mobilizer.
//   2. Where in the state the dofs for a particular body are.
// The mathematical model of this system, after Finalize(), should contain
// three floating bodies, one for each Atlas robot and one for the mug.
TEST_F(MultibodyPlantIntrospection, FloatingBodies) {
  plant.Finalize();

  // Assert that the mug and the two Atlas robot pelvises are floating and
  // modeled with quaternions.
  ASSERT_TRUE(mug->is_floating());
  ASSERT_TRUE(mug->has_quaternion_dofs());
  ASSERT_TRUE(pelvis1->is_floating());
  ASSERT_TRUE(pelvis1->has_quaternion_dofs());
  ASSERT_TRUE(pelvis2->is_floating());
  ASSERT_TRUE(pelvis2->has_quaternion_dofs());

  // The "world" is not considered as a free body.
  EXPECT_FALSE(plant.world_body().is_floating());

  // The table has been anchored to the world.
  EXPECT_FALSE(plant.GetBodyByName("link", robot_table_model).is_floating());

  // Retrieve floating bodies.
  std::unordered_set<BodyIndex> expected_floating_bodies(
      {pelvis1->index(), pelvis2->index(), mug->index()});
  auto floating_bodies = plant.GetFloatingBaseBodies();
  EXPECT_EQ(expected_floating_bodies, floating_bodies);

  // Verify state indexes for free bodies.
  const std::unordered_set<int> expected_floating_positions_start({0, 7, 14});
  const std::unordered_set<int> floating_positions_start(
      {pelvis1->floating_positions_start(), pelvis2->floating_positions_start(),
       mug->floating_positions_start()});
  EXPECT_EQ(floating_positions_start, expected_floating_positions_start);

  const int nq = plant.num_positions();
  const std::unordered_set<int> expected_floating_velocities_start(
      {nq, nq + 6, nq + 12});
  const std::unordered_set<int> floating_velocities_start(
      {pelvis1->floating_velocities_start(),
       pelvis2->floating_velocities_start(),
       mug->floating_velocities_start()});
  EXPECT_EQ(floating_velocities_start, expected_floating_velocities_start);
}

// This test verifies features of the MultibodySearch convenience utility.
TEST_F(MultibodyPlantIntrospection, MultibodySearch) {
  plant.Finalize();
  MultibodySearch<double> search(&plant, &scene_graph);

  // There is one world model containing one body which is the world.
  EXPECT_EQ(search.WorldModel().count(), 1);
  EXPECT_EQ(search.WorldModel().Bodies().count(), 1);
  EXPECT_EQ(search.WorldModel().Bodies().ids(), search.WorldBody().ids());
  EXPECT_EQ(search.WorldModel().Bodies().Named("WorldBody").count(), 1);

  // Check that we can start a search at a model by ID.
  const ModelInstanceIndex model_id =
      *search.WorldModel().Named("robot_table").ids().begin();
  EXPECT_EQ(search(model_id).ids(), std::set<ModelInstanceIndex>{model_id});

  // Sanity-check that Named/Names round-trip properly.
  EXPECT_EQ(search.WorldModel().Bodies().Named("WorldBody").Names(),
            std::set<std::string>{"WorldBody"});

  // There are six models in the plant (world, default, 2xAtlas, mug, table).
  EXPECT_EQ(search.AllModels().count(), 6);

  // There are great many bodies in the world.  Two of them are Atlas pelvises.
  EXPECT_EQ(search.AllModels().Bodies().count(), 143);
  EXPECT_EQ(search.AllModels().Bodies().Named("pelvis").count(), 2);

  // There are a whole lot of MBT-frames in the world.  Eight of them are
  // attached to Atlas pelvises (two each of a body frame and three joints).
  EXPECT_EQ(search.AllMultibodyFrames().count(), 292);
  auto pelvis_attached_frames = search.AllMultibodyFrames().AttachedTo(
      search.AllModels().Bodies().Named("pelvis"));
  EXPECT_EQ(pelvis_attached_frames.count(), 8);
  // Most of those frames are nameless, except for the body frames.
  std::set<std::string> expected_names{"", "pelvis"};
  EXPECT_EQ(pelvis_attached_frames.Names(), expected_names);

  // There are also a lot of geometries in the world, attached to a somewhat
  // smaller number of geometry-frames.  There is exactly one geometry frame
  // without any attached geometry, which is the world frame.
  EXPECT_EQ(search.AllGeometries().count(), 193);
  EXPECT_EQ(search.AllGeometries().Frames().count(),
            search.AllGeometryFrames().count() - 1 /* the World */);
  EXPECT_EQ(
      search.AllGeometries().Frames().Union(search.WorldGeometryFrame()).ids(),
      search.AllGeometryFrames().ids());
  EXPECT_EQ(search.AllGeometryFrames().count(), 75);

  // Searching down from geometries to bodies and models reaches all of the
  // models except the world and default.
  EXPECT_EQ(search.AllGeometries().Frames().Bodies().Models().count(),
            search.AllModels().count() - 1 /* world */ - 1 /* default */);
  EXPECT_EQ(search.AllGeometries().Frames().Bodies().Models()
            .Union(search.WorldModel())
            .Union(search.DefaultModel()).ids(),
            search.AllModels().ids());
}

}  // namespace
}  // namespace multibody
}  // namespace drake
