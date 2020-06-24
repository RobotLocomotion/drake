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
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using geometry::GeometryId;

// Fixture to setup a simple model with both collision and visual geometry,
// loaded with the SDF parser.
class MultibodyPlantLinkTests :
      public testing::TestWithParam<test::ModelLoadFunction> {
 public:
  // Loads the MultibodyPlant part of the model. Geometry is ignored.
  void LoadMultibodyPlantOnly() {
    plant_unique_ptr_ = std::make_unique<MultibodyPlant<double>>(0.0);
    auto load_model = GetParam();
    load_model(base_name_, plant_unique_ptr_.get(), nullptr);
    plant_unique_ptr_->Finalize();
  }

  // Loads the entire model including the multibody dynamics part of it and the
  // geometries for both visualization and contact modeling.
  void LoadMultibodyPlantAndSceneGraph() {
    plant_unique_ptr_ = std::make_unique<MultibodyPlant<double>>(0.0);
    scene_graph_unique_ptr_ = std::make_unique<geometry::SceneGraph<double>>();
    auto load_model = GetParam();
    load_model(base_name_, plant_unique_ptr_.get(),
               scene_graph_unique_ptr_.get());
    plant_unique_ptr_->Finalize();
  }

  // Loads the plant and scene graph, connected in a diagram.
  void LoadMultibodyPlantAndSceneGraphDiagram() {
    plant_unique_ptr_ = std::make_unique<MultibodyPlant<double>>(0.0);
    scene_graph_unique_ptr_ = std::make_unique<geometry::SceneGraph<double>>();
    systems::DiagramBuilder<double> builder;
    auto result =
        AddMultibodyPlantSceneGraph(&builder, std::move(plant_unique_ptr_),
                                    std::move(scene_graph_unique_ptr_));
    std::tie(plant_ptr_, scene_graph_ptr_) = result;
    diagram_ptr_ = builder.Build();
    auto load_model = GetParam();
    load_model(base_name_, plant_ptr_, scene_graph_ptr_);
    plant_ptr_->Finalize();
  }

  // Ownership of the plant and scene graph might be transferred to the diagram,
  // if built. Use the appropriate pointer.
  MultibodyPlant<double>& plant() {
    return (diagram_ptr_ ? *plant_ptr_ : *plant_unique_ptr_);
  }

  geometry::SceneGraph<double>& scene_graph() {
    return (diagram_ptr_ ? *scene_graph_ptr_ : *scene_graph_unique_ptr_);
  }

 protected:
  std::unique_ptr<MultibodyPlant<double>> plant_unique_ptr_;
  std::unique_ptr<geometry::SceneGraph<double>> scene_graph_unique_ptr_;
  std::unique_ptr<systems::Diagram<double>> diagram_ptr_;

  MultibodyPlant<double>* plant_ptr_{nullptr};
  geometry::SceneGraph<double>* scene_graph_ptr_{nullptr};


  const std::string base_name_{"drake/multibody/parsing/test/"
        "links_with_visuals_and_collisions"};
};

TEST_P(MultibodyPlantLinkTests, LinkWithVisuals) {
  LoadMultibodyPlantAndSceneGraphDiagram();
  MultibodyPlant<double>& plant_ = plant();
  auto diagram_context = diagram_ptr_->CreateDefaultContext();
  auto& plant_context =
      plant_.GetMyMutableContextFromRoot(diagram_context.get());

  EXPECT_EQ(plant_.num_bodies(), 4);  // It includes the world body.
  EXPECT_EQ(plant_.EvalNumVisualGeometries(&plant_context), 5);

  const std::vector<GeometryId>& link1_visual_geometry_ids =
      plant_.GetVisualGeometriesForBody(plant_context,
                                        plant_.GetBodyByName("link1"));
  EXPECT_EQ(link1_visual_geometry_ids.size(), 2);

  const std::vector<GeometryId>& link2_visual_geometry_ids =
      plant_.GetVisualGeometriesForBody(plant_context,
                                        plant_.GetBodyByName("link2"));
  EXPECT_EQ(link2_visual_geometry_ids.size(), 3);

  const std::vector<GeometryId>& link3_visual_geometry_ids =
      plant_.GetVisualGeometriesForBody(plant_context,
                                        plant_.GetBodyByName("link3"));
  EXPECT_EQ(link3_visual_geometry_ids.size(), 0);

  // TODO(SeanCurtis-TRI): Once SG supports it, confirm `GeometryId` maps to the
  // correct geometry.
}

// Verifies we can still parse the model dynamics if a SceneGraph is not
// supplied.
TEST_P(MultibodyPlantLinkTests, ParseWithoutASceneGraph) {
  LoadMultibodyPlantOnly();
  MultibodyPlant<double>& plant_ = plant();

  EXPECT_EQ(plant_.num_bodies(), 4);  // It includes the world body.
}

// Verifies that the source registration with a SceneGraph can happen before a
// call to AddModelFromSdfFile().
TEST_P(MultibodyPlantLinkTests, RegisterWithASceneGraphBeforeParsing) {
  LoadMultibodyPlantAndSceneGraphDiagram();
  MultibodyPlant<double>& plant_ = plant();
  auto diagram_context = diagram_ptr_->CreateDefaultContext();
  auto& plant_context =
      plant_.GetMyMutableContextFromRoot(diagram_context.get());

  EXPECT_EQ(plant_.num_bodies(), 4);  // It includes the world body.
  EXPECT_EQ(plant_.EvalNumVisualGeometries(&plant_context), 5);

  const std::vector<GeometryId>& link1_visual_geometry_ids =
      plant_.GetVisualGeometriesForBody(plant_context,
                                        plant_.GetBodyByName("link1"));
  EXPECT_EQ(link1_visual_geometry_ids.size(), 2);

  // TODO(sam.creasey) Verify that the path to the mesh for the second
  // visual geometry on link 1 is resolved correctly.  Currently the
  // resolved mesh filename is trapped inside the shape object within
  // the scene graph and I can't find any good way to dig it back out.
  // It would be possible to modify geometry::DispatchLoadMessage to
  // take a DrakeLcmInterface and then scrape the filename out of the
  // resulting lcmt_viewer_load_robot message, but I don't want to.

  const std::vector<GeometryId>& link2_visual_geometry_ids =
      plant_.GetVisualGeometriesForBody(plant_context,
                                        plant_.GetBodyByName("link2"));
  EXPECT_EQ(link2_visual_geometry_ids.size(), 3);

  const std::vector<GeometryId>& link3_visual_geometry_ids =
      plant_.GetVisualGeometriesForBody(plant_context,
                                        plant_.GetBodyByName("link3"));
  EXPECT_EQ(link3_visual_geometry_ids.size(), 0);

  // TODO(SeanCurtis-TRI): Once SG supports it, confirm `GeometryId` maps to the
  // correct geometry.
}

// Verifies we can parse link collision geometries and surface friction.
TEST_P(MultibodyPlantLinkTests, LinksWithCollisions) {
  LoadMultibodyPlantAndSceneGraphDiagram();
  MultibodyPlant<double>& plant_ = plant();
  geometry::SceneGraph<double>& scene_graph_ = scene_graph();

  auto diagram_context = diagram_ptr_->CreateDefaultContext();
  auto& plant_context =
      plant_.GetMyMutableContextFromRoot(diagram_context.get());

  EXPECT_EQ(plant_.num_bodies(), 4);  // It includes the world body.
  EXPECT_EQ(plant_.EvalNumCollisionGeometries(&plant_context), 3);

  const std::vector<GeometryId>& link1_collision_geometry_ids =
      plant_.GetCollisionGeometriesForBody(plant_context,
                                           plant_.GetBodyByName("link1"));
  ASSERT_EQ(link1_collision_geometry_ids.size(), 2);

  EXPECT_TRUE(
      scene_graph_.model_inspector()
          .GetProximityProperties(
              scene_graph_.model_inspector().GetGeometryIdByName(
                  plant_.GetBodyFrameIdOrThrow(
                      plant_.GetBodyByName("link1").index()),
                  geometry::Role::kProximity, "test_robot::link1_collision1"))
          ->GetProperty<CoulombFriction<double>>("material",
                                                 "coulomb_friction") ==
      CoulombFriction<double>(0.8, 0.3));
  EXPECT_TRUE(
      scene_graph_.model_inspector()
          .GetProximityProperties(
              scene_graph_.model_inspector().GetGeometryIdByName(
                  plant_.GetBodyFrameIdOrThrow(
                      plant_.GetBodyByName("link1").index()),
                  geometry::Role::kProximity, "test_robot::link1_collision2"))
          ->GetProperty<CoulombFriction<double>>("material",
                                                 "coulomb_friction") ==
      CoulombFriction<double>(1.5, 0.6));

  const std::vector<GeometryId>& link2_collision_geometry_ids =
      plant_.GetCollisionGeometriesForBody(plant_context,
                                           plant_.GetBodyByName("link2"));
  ASSERT_EQ(link2_collision_geometry_ids.size(), 0);

  const std::vector<GeometryId>& link3_collision_geometry_ids =
      plant_.GetCollisionGeometriesForBody(plant_context,
                                           plant_.GetBodyByName("link3"));
  ASSERT_EQ(link3_collision_geometry_ids.size(), 1);
  // Verifies the default value of the friction coefficients when the user does
  // not specify them in the SDF file.
  EXPECT_EQ(scene_graph_.model_inspector()
                .GetProximityProperties(link3_collision_geometry_ids[0])
                ->GetProperty<CoulombFriction<double>>("material",
                                                       "coulomb_friction"),
            default_friction());
}


INSTANTIATE_TEST_SUITE_P(SdfMultibodyPlantLinkTests,
                        MultibodyPlantLinkTests,
                        ::testing::Values(test::LoadFromSdf));

INSTANTIATE_TEST_SUITE_P(UrdfMultibodyPlantLinkTests,
                        MultibodyPlantLinkTests,
                        ::testing::Values(test::LoadFromUrdf));

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
