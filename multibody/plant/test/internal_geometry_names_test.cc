#include "drake/multibody/plant/internal_geometry_names.h"

#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/multibody/parsing/parser.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using geometry::GeometryId;
using geometry::SceneGraph;
using geometry::SceneGraphInspector;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;

using Entry = GeometryNames::Entry;

class GeometryNamesTest : public ::testing::Test {
 public:
  void SetUp() override {
    DiagramBuilder<double> builder;
    std::tie(plant_, scene_graph_) =
        AddMultibodyPlantSceneGraph(&builder, 0.001);
    Parser parser(plant_);

    // A single model, single body, single geometry.
    const std::string box_url = "package://drake/multibody/models/box.urdf";
    parser.AddModelsFromUrl(box_url);

    // A single model, single body, multiple geometries.
    const std::string bin_url =
        "package://drake_models/manipulation_station/bin.sdf";
    parser.AddModelsFromUrl(bin_url);

    // Two identical models (each one has a single body, single geometry).
    const std::string sphere =
        "package://drake_models/manipulation_station/sphere.sdf";
    Parser(plant_, "sphere1").AddModelsFromUrl(sphere);
    Parser(plant_, "sphere2").AddModelsFromUrl(sphere);

    // Build everything.
    plant_->Finalize();
    diagram_ = builder.Build();
  }

  const MultibodyPlant<double>& plant() const { return *plant_; }

  const SceneGraphInspector<double>& inspector() const {
    return scene_graph_->model_inspector();
  }

  const std::vector<GeometryId>& GetGeometryIds(
      std::string_view body_name,
      std::optional<std::string_view> model_name = {}) const {
    const RigidBody<double>* body{};
    if (model_name.has_value()) {
      ModelInstanceIndex index = plant_->GetModelInstanceByName(*model_name);
      body = &(plant_->GetBodyByName(body_name, index));
    } else {
      body = &(plant_->GetBodyByName(body_name));
    }
    return plant_->GetCollisionGeometriesForBody(*body);
  }

  GeometryId GetSoleGeometryId(
      std::string_view body_name,
      std::optional<std::string_view> model_name = {}) const {
    const auto& ids = GetGeometryIds(body_name, model_name);
    DRAKE_DEMAND(ids.size() == 1);
    return ids.front();
  }

 private:
  std::unique_ptr<Diagram<double>> diagram_;
  MultibodyPlant<double>* plant_{};
  SceneGraph<double>* scene_graph_{};
};

TEST_F(GeometryNamesTest, ResetBasic) {
  GeometryNames dut;
  EXPECT_EQ(dut.entries().size(), 0);

  dut.ResetBasic(plant());
  const size_t num_entries = dut.entries().size();
  EXPECT_GT(num_entries, 0);

  dut.ResetBasic(plant());
  EXPECT_EQ(dut.entries().size(), num_entries);
}

TEST_F(GeometryNamesTest, ResetFull) {
  GeometryNames dut;
  EXPECT_EQ(dut.entries().size(), 0);

  dut.ResetFull(plant(), inspector());
  const size_t num_entries = dut.entries().size();
  EXPECT_GT(num_entries, 0);

  dut.ResetFull(plant(), inspector());
  EXPECT_EQ(dut.entries().size(), num_entries);
}

TEST_F(GeometryNamesTest, BasicBox) {
  GeometryNames dut;
  dut.ResetBasic(plant());
  const GeometryId id = GetSoleGeometryId("box");
  const Entry& entry = dut.Find(id);
  EXPECT_EQ(entry.model_instance_name, "box");
  EXPECT_EQ(entry.body_name, "box");
  EXPECT_EQ(entry.geometry_name, std::nullopt);
  EXPECT_EQ(entry.body_name_is_unique_within_plant, true);
  EXPECT_EQ(entry.is_sole_geometry_within_body, true);
  EXPECT_EQ(dut.GetFullName(id, "$"), "box");
}

TEST_F(GeometryNamesTest, FullBox) {
  GeometryNames dut;
  dut.ResetFull(plant(), inspector());
  const GeometryId id = GetSoleGeometryId("box");
  const Entry& entry = dut.Find(id);
  EXPECT_EQ(entry.model_instance_name, "box");
  EXPECT_EQ(entry.body_name, "box");
  EXPECT_EQ(entry.geometry_name, "box");
  EXPECT_EQ(entry.body_name_is_unique_within_plant, true);
  EXPECT_EQ(entry.is_sole_geometry_within_body, true);
  EXPECT_EQ(dut.GetFullName(id, "$"), "box");
}

TEST_F(GeometryNamesTest, BasicSphere1) {
  GeometryNames dut;
  dut.ResetBasic(plant());
  const GeometryId id = GetSoleGeometryId("base_link", "sphere1::sphere");
  const Entry& entry = dut.Find(id);
  EXPECT_EQ(entry.model_instance_name, "sphere1::sphere");
  EXPECT_EQ(entry.body_name, "base_link");
  EXPECT_EQ(entry.geometry_name, std::nullopt);
  EXPECT_EQ(entry.body_name_is_unique_within_plant, false);
  EXPECT_EQ(entry.is_sole_geometry_within_body, true);
  EXPECT_EQ(dut.GetFullName(id, "$"), "sphere1::sphere$base_link");
}

TEST_F(GeometryNamesTest, FullSphere1) {
  GeometryNames dut;
  dut.ResetFull(plant(), inspector());
  const GeometryId id = GetSoleGeometryId("base_link", "sphere1::sphere");
  const Entry& entry = dut.Find(id);
  EXPECT_EQ(entry.model_instance_name, "sphere1::sphere");
  EXPECT_EQ(entry.body_name, "base_link");
  EXPECT_EQ(entry.geometry_name, "sphere_collision");
  EXPECT_EQ(entry.body_name_is_unique_within_plant, false);
  EXPECT_EQ(entry.is_sole_geometry_within_body, true);
  EXPECT_EQ(dut.GetFullName(id, "$"), "sphere1::sphere$base_link");
}

TEST_F(GeometryNamesTest, BasicBin) {
  GeometryNames dut;
  dut.ResetBasic(plant());
  const auto& ids = GetGeometryIds("bin_base");
  const GeometryId id = ids.front();
  const Entry& entry = dut.Find(id);
  EXPECT_EQ(entry.model_instance_name, "bin_model");
  EXPECT_EQ(entry.body_name, "bin_base");
  EXPECT_EQ(entry.geometry_name, std::nullopt);
  EXPECT_EQ(entry.body_name_is_unique_within_plant, true);
  EXPECT_EQ(entry.is_sole_geometry_within_body, false);
  EXPECT_THAT(dut.GetFullName(id, "$"), testing::StartsWith("bin_base$Id("));
}

TEST_F(GeometryNamesTest, FullBin) {
  GeometryNames dut;
  dut.ResetFull(plant(), inspector());
  const auto& ids = GetGeometryIds("bin_base");
  const GeometryId id = ids.front();
  const Entry& entry = dut.Find(id);
  EXPECT_EQ(entry.model_instance_name, "bin_model");
  EXPECT_EQ(entry.body_name, "bin_base");
  EXPECT_EQ(entry.geometry_name, "front");
  EXPECT_EQ(entry.body_name_is_unique_within_plant, true);
  EXPECT_EQ(entry.is_sole_geometry_within_body, false);
  EXPECT_EQ(dut.GetFullName(id, "$"), "bin_base$front");
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
