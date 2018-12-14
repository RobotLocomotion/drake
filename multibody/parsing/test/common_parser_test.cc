/// @file
///
/// This file contains tests which check the properties of a multibody plant
/// loaded from different formats.  The tests do not depend on which parser is
/// used, as different parsers should create the same plant from a correct
/// model in the appropriate format (URDF/SDF).

#include <string>

#include <gtest/gtest.h>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/detail_common.h"
#include "drake/multibody/parsing/test/test_loaders.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace detail {
namespace {

using geometry::GeometryId;

// Fixture to setup a simple model with both collision and visual geometry,
// loaded with the SDF parser.
class MultibodyPlantLinkTests :
      public testing::TestWithParam<test::ModelLoadFunction> {
 public:
  // Loads the MultibodyPlant part of the model. Geometry is ignored.
  void LoadMultibodyPlantOnly() {
    auto load_model = GetParam();
    load_model(base_name_, &plant_, nullptr);
    plant_.Finalize();
  }

  // Loads the entire model including the multibody dynamics part of it and the
  // geometries for both visualization and contact modeling.
  void LoadMultibodyPlantAndSceneGraph() {
    auto load_model = GetParam();
    load_model(base_name_, &plant_, &scene_graph_);
    plant_.Finalize(&scene_graph_);
  }

 protected:
  MultibodyPlant<double> plant_;
  geometry::SceneGraph<double> scene_graph_;
  const std::string base_name_{"drake/multibody/parsing/test/"
        "links_with_visuals_and_collisions"};
};

TEST_P(MultibodyPlantLinkTests, LinkWithVisuals) {
  LoadMultibodyPlantAndSceneGraph();

  EXPECT_EQ(plant_.num_bodies(), 4);  // It includes the world body.
  EXPECT_EQ(plant_.num_visual_geometries(), 5);

  const std::vector<GeometryId>& link1_visual_geometry_ids =
      plant_.GetVisualGeometriesForBody(plant_.GetBodyByName("link1"));
  EXPECT_EQ(link1_visual_geometry_ids.size(), 2);

  const std::vector<GeometryId>& link2_visual_geometry_ids =
      plant_.GetVisualGeometriesForBody(plant_.GetBodyByName("link2"));
  EXPECT_EQ(link2_visual_geometry_ids.size(), 3);

  const std::vector<GeometryId>& link3_visual_geometry_ids =
      plant_.GetVisualGeometriesForBody(plant_.GetBodyByName("link3"));
  EXPECT_EQ(link3_visual_geometry_ids.size(), 0);

  // TODO(SeanCurtis-TRI): Once SG supports it, confirm `GeometryId` maps to the
  // correct geometry.
}

// Verifies we can still parse the model dynamics if a SceneGraph is not
// supplied.
TEST_P(MultibodyPlantLinkTests, ParseWithoutASceneGraph) {
  LoadMultibodyPlantOnly();

  EXPECT_EQ(plant_.num_bodies(), 4);  // It includes the world body.
  EXPECT_EQ(plant_.num_visual_geometries(), 0);
}

// Verifies that the source registration with a SceneGraph can happen before a
// call to AddModelFromSdfFile().
TEST_P(MultibodyPlantLinkTests, RegisterWithASceneGraphBeforeParsing) {
  plant_.RegisterAsSourceForSceneGraph(&scene_graph_);
  LoadMultibodyPlantAndSceneGraph();

  EXPECT_EQ(plant_.num_bodies(), 4);  // It includes the world body.
  EXPECT_EQ(plant_.num_visual_geometries(), 5);

  const std::vector<GeometryId>& link1_visual_geometry_ids =
      plant_.GetVisualGeometriesForBody(plant_.GetBodyByName("link1"));
  EXPECT_EQ(link1_visual_geometry_ids.size(), 2);

  // TODO(sam.creasey) Verify that the path to the mesh for the second
  // visual geometry on link 1 is resolved correctly.  Currently the
  // resolved mesh filename is trapped inside the shape object within
  // the scene graph and I can't find any good way to dig it back out.
  // It would be possible to modify geometry::DispatchLoadMessage to
  // take a DrakeLcmInterface and then scrape the filename out of the
  // resulting lcmt_viewer_load_robot message, but I don't want to.

  const std::vector<GeometryId>& link2_visual_geometry_ids =
      plant_.GetVisualGeometriesForBody(plant_.GetBodyByName("link2"));
  EXPECT_EQ(link2_visual_geometry_ids.size(), 3);

  const std::vector<GeometryId>& link3_visual_geometry_ids =
      plant_.GetVisualGeometriesForBody(plant_.GetBodyByName("link3"));
  EXPECT_EQ(link3_visual_geometry_ids.size(), 0);

  // TODO(SeanCurtis-TRI): Once SG supports it, confirm `GeometryId` maps to the
  // correct geometry.
}

// Verifies we can parse link collision geometries and surface friction.
TEST_P(MultibodyPlantLinkTests, LinksWithCollisions) {
  LoadMultibodyPlantAndSceneGraph();

  EXPECT_EQ(plant_.num_bodies(), 4);  // It includes the world body.
  EXPECT_EQ(plant_.num_collision_geometries(), 3);

  const std::vector<GeometryId>& link1_collision_geometry_ids =
      plant_.GetCollisionGeometriesForBody(plant_.GetBodyByName("link1"));
  ASSERT_EQ(link1_collision_geometry_ids.size(), 2);

  EXPECT_TRUE(
      plant_.default_coulomb_friction(link1_collision_geometry_ids[0]) ==
          CoulombFriction<double>(0.8, 0.3));
  EXPECT_TRUE(
      plant_.default_coulomb_friction(link1_collision_geometry_ids[1]) ==
          CoulombFriction<double>(1.5, 0.6));

  const std::vector<GeometryId>& link2_collision_geometry_ids =
      plant_.GetCollisionGeometriesForBody(plant_.GetBodyByName("link2"));
  ASSERT_EQ(link2_collision_geometry_ids.size(), 0);

  const std::vector<GeometryId>& link3_collision_geometry_ids =
      plant_.GetCollisionGeometriesForBody(plant_.GetBodyByName("link3"));
  ASSERT_EQ(link3_collision_geometry_ids.size(), 1);
  // Verifies the default value of the friction coefficients when the user does
  // not specify them in the SDF file.
  EXPECT_TRUE(
      plant_.default_coulomb_friction(link3_collision_geometry_ids[0]) ==
      default_friction());
}


INSTANTIATE_TEST_CASE_P(SdfMultibodyPlantLinkTests,
                        MultibodyPlantLinkTests,
                        ::testing::Values(test::LoadFromSdf));

INSTANTIATE_TEST_CASE_P(UrdfMultibodyPlantLinkTests,
                        MultibodyPlantLinkTests,
                        ::testing::Values(test::LoadFromUrdf));

}  // namespace
}  // namespace detail
}  // namespace multibody
}  // namespace drake
