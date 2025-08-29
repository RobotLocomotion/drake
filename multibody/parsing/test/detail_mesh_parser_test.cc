#include "drake/multibody/parsing/detail_mesh_parser.h"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/diagnostic_policy_test_base.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using drake::internal::DiagnosticPolicy;
using geometry::SceneGraph;

class MeshParserTest : public test::DiagnosticPolicyTestBase {
 public:
  MeshParserTest() = default;

  static ParserInterface& TestingSelect(const DiagnosticPolicy&,
                                        const std::string&) {
    static never_destroyed<MeshParserWrapper> mesh;
    return mesh.access();
  }

  ModelInstanceIndex AddModelFromMeshFile(
      const std::string& file_name, const std::string& model_name,
      const std::optional<std::string>& parent_model_name = {}) {
    const DataSource data_source{DataSource::kFilename, &file_name};
    internal::CollisionFilterGroupResolver resolver{&plant_};
    ParsingWorkspace w{options_, package_map_, diagnostic_policy_, nullptr,
                       &plant_,  &resolver,    TestingSelect};
    // The wrapper simply delegates to AddModelFromMesh(), so we're testing
    // the underlying implementation *and* confirming that the wrapper delegates
    // appropriately.
    MeshParserWrapper wrapper;
    std::optional<ModelInstanceIndex> result =
        wrapper.AddModel(data_source, model_name, parent_model_name, w);
    return result.value_or(ModelInstanceIndex{});
  }

  ModelInstanceIndex AddModelsFromMeshFile(
      const std::string& file_name,
      const std::optional<std::string>& parent_model_name = {}) {
    const DataSource data_source{DataSource::kFilename, &file_name};
    internal::CollisionFilterGroupResolver resolver{&plant_};
    ParsingWorkspace w{options_, package_map_, diagnostic_policy_, nullptr,
                       &plant_,  &resolver,    TestingSelect};
    // The wrapper is responsible for building the vector from whatever a call
    // to AddModelFromMesh() does; this confirms invocation and successful
    // transformation of return type.
    MeshParserWrapper wrapper;
    std::vector<ModelInstanceIndex> results =
        wrapper.AddAllModels(data_source, parent_model_name, w);
    // We should either throw or return a model instance index.
    DRAKE_DEMAND(results.size() == 1);
    return results[0];
  }

 protected:
  ParsingOptions options_;
  PackageMap package_map_;
  MultibodyPlant<double> plant_{0.0};
  SceneGraph<double> scene_graph_;
};

// Tests the name generation logic for model instances and bodies. This
// includes:
//
//  - Naming protocol: 1) user given name, 2) OBJ data name, 3) file name.
//  - Combination with "parent_model_name" prefix.
//  - Prefix only applies to model name and not body name.
//  - A token call to AddModelsFromMesh (plural); we know it simply delegates
//    to AddModelFromMesh.
TEST_F(MeshParserTest, ModelInstanceNameTest) {
  // Initial plant has two model instances (model_instance.h explains why).
  ASSERT_EQ(plant_.num_model_instances(), 2);

  // These two files both define a 2x2x2 (meter) box. The former has only the
  // geometry (no object name). The latter has object name *and* material
  // library.
  const std::string obj_path = FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/meshes/box.obj");
  const std::string obj_with_object_path =
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.obj");

  // We'll add boxes with a various combination of permutations.
  const ModelInstanceIndex filename_model_instance =
      AddModelFromMeshFile(obj_path, "");

  DRAKE_EXPECT_THROWS_MESSAGE(
      AddModelFromMeshFile(obj_path, ""),
      ".+already contains a model instance named 'box'.+");

  const ModelInstanceIndex user_named_model_instance =
      AddModelFromMeshFile(obj_path, "box1");

  const ModelInstanceIndex prefix_user_model_instance =
      AddModelFromMeshFile(obj_path, "box2", "prefix");

  const ModelInstanceIndex prefix_file_model_instance =
      AddModelFromMeshFile(obj_path, "", "prefix");

  const ModelInstanceIndex geom_box_model_instance =
      AddModelFromMeshFile(obj_with_object_path, "");

  const ModelInstanceIndex user_geom_box_model_instance =
      AddModelFromMeshFile(obj_with_object_path, "geom_box");

  // This is the token call to the plural form: AddModelsFromMesh.
  const ModelInstanceIndex prefix_geom_box_model_instance =
      AddModelsFromMeshFile(obj_with_object_path, "prefix");

  // Proof that we didn't need a SceneGraph to happily add bodies.
  ASSERT_FALSE(plant_.geometry_source_is_registered());

  // We are done adding models.
  plant_.Finalize();

  ASSERT_EQ(plant_.num_model_instances(), 9);

  // All instances names are as expected.
  EXPECT_EQ(plant_.GetModelInstanceByName("box"), filename_model_instance);
  EXPECT_EQ(plant_.GetModelInstanceByName("box1"), user_named_model_instance);
  EXPECT_EQ(plant_.GetModelInstanceByName("prefix::box2"),
            prefix_user_model_instance);
  EXPECT_EQ(plant_.GetModelInstanceByName("prefix::box"),
            prefix_file_model_instance);
  EXPECT_EQ(plant_.GetModelInstanceByName("test_box"), geom_box_model_instance);
  EXPECT_EQ(plant_.GetModelInstanceByName("geom_box"),
            user_geom_box_model_instance);
  EXPECT_EQ(plant_.GetModelInstanceByName("prefix::test_box"),
            prefix_geom_box_model_instance);

  // All body names are as expected.
  EXPECT_TRUE(plant_.HasBodyNamed("box", filename_model_instance));
  EXPECT_TRUE(plant_.HasBodyNamed("box1", user_named_model_instance));
  EXPECT_TRUE(plant_.HasBodyNamed("box2", prefix_user_model_instance));
  EXPECT_TRUE(plant_.HasBodyNamed("box", prefix_file_model_instance));
  EXPECT_TRUE(plant_.HasBodyNamed("test_box", geom_box_model_instance));
  EXPECT_TRUE(plant_.HasBodyNamed("geom_box", user_geom_box_model_instance));
  EXPECT_TRUE(plant_.HasBodyNamed("test_box", prefix_geom_box_model_instance));
}

// Confirms that multiple objects are acceptable. It can't be arbitrary multiple
// objects; they must be triangles that will produce a spatial inertia that
// won't trip over the physically valid test.
TEST_F(MeshParserTest, MultiObjects) {
  // Reset the diagnostic policy to actually throw on errors.
  diagnostic_policy_ = DiagnosticPolicy();

  const std::filesystem::path temp_dir = temp_directory();
  const std::filesystem::path file = temp_dir / "two_objects.obj";
  {
    std::ofstream f(file.string());
    ASSERT_TRUE(f);
    // Two identical triangles displaced from the origin with normals pointing
    // away from the origin are sufficient to produce a physically valid-ish
    // spatial inertia.
    f << "v 0 0 1\n"
      << "v 1 0 1\n"
      << "v 0 1 1\n"
      << "o one\n"  // Create first object.
      << "f 1 2 3\n"
      << "o two\n"  // Create a second object.
      << "f 1 2 3\n";
  }
  EXPECT_NO_THROW(AddModelFromMeshFile(file.string(), ""));
}

// Various means by which we end up getting an exception.
TEST_F(MeshParserTest, ErrorModes) {
  // Reset the diagnostic policy to actually throw on errors.
  diagnostic_policy_ = DiagnosticPolicy();

  const std::filesystem::path temp_dir = temp_directory();

  // 1. No geometry.
  {
    const std::filesystem::path file = temp_dir / "empty.obj";
    {
      std::ofstream f(file.string());
      ASSERT_TRUE(f);
      f << "v 0 0 0\n";  // A single vertex with no faces.
    }

    DRAKE_EXPECT_THROWS_MESSAGE(AddModelFromMeshFile(file.string(), ""),
                                ".*OBJ data parsed contains no objects.*");
  }
  // 2. Not an OBJ.
  {
    const std::filesystem::path file = temp_dir / "empty.obj";
    {
      std::ofstream f(file.string());
      ASSERT_TRUE(f);
      f << "Non-OBJ gibberish";
    }
    DRAKE_EXPECT_THROWS_MESSAGE(AddModelFromMeshFile(file.string(), ""),
                                ".*OBJ data parsed contains no objects.*");
  }
  // 3. Called with obj data in a string.
  {
    const std::string data("Just some text");
    const DataSource data_source{DataSource::kContents, &data};
    internal::CollisionFilterGroupResolver resolver{&plant_};
    ParsingWorkspace w{options_, package_map_, diagnostic_policy_, nullptr,
                       &plant_,  &resolver,    TestingSelect};
    DRAKE_EXPECT_THROWS_MESSAGE(
        AddModelFromMesh(data_source, "", std::nullopt, w),
        ".*must be .+ file.*");
  }
  // 5. Plant has been finalized.
  {
    plant_.Finalize();
    const std::string obj_path = FindResourceOrThrow(
        "drake/multibody/parsing/test/box_package/meshes/box.obj");
    EXPECT_THROW(AddModelFromMeshFile(obj_path, ""), std::exception);
  }
}

// Confirm that the obj added has computed the correct mass. As these
// calculations delegate to other tested APIs, we'll simply look for evidence
// that they were correctly invoked and the results registered properly.
TEST_F(MeshParserTest, CorrectMass) {
  // A 2x2x2 (meter) box centered on its canonical frame.
  const std::string obj_path = FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/meshes/box.obj");
  const ModelInstanceIndex model_instance =
      AddModelFromMeshFile(obj_path, "body");

  // We are done adding models.
  plant_.Finalize();
  auto context = plant_.CreateDefaultContext();

  const RigidBody<double>& body = plant_.GetBodyByName("body", model_instance);
  const double mass = body.get_mass(*context);
  const double density = 1000;  // kg/m³
  // 2x2x2 box with proscribed density.
  EXPECT_DOUBLE_EQ(mass, 2 * 2 * 2 * density);

  const auto I_BBo_B_expected =
      SpatialInertia<double>::SolidBoxWithDensity(density, 2, 2, 2);
  const SpatialInertia<double> I_BBo_B =
      body.CalcSpatialInertiaInBodyFrame(*context);
  // For unit scale, we'd expect tolerance to be satisfied around 1e-15; with
  // a mass of 1000 kg/m³, we have to scale the tolerance accordingly.
  EXPECT_TRUE(CompareMatrices(I_BBo_B.CopyToFullMatrix6(),
                              I_BBo_B_expected.CopyToFullMatrix6(), 1e-12));
}

TEST_F(MeshParserTest, ZeroVolume) {
  const std::string obj_path =
      FindResourceOrThrow("drake/geometry/test/bad_geometry_volume_zero.obj");
  AddModelFromMeshFile(obj_path, "body");
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(".*volume.*is 0.*"));
}

// When the plant has been registered as a SceneGraph geometry source, we should
// get illustration, perception, and proximity roles associated with the body.
TEST_F(MeshParserTest, RegisteredGeometry) {
  plant_.RegisterAsSourceForSceneGraph(&scene_graph_);

  // A 2x2x2 (meter) box centered on its canonical frame.
  const std::string obj_path = FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/meshes/box.obj");

  // We'll add boxes with a various combination of permutations.
  const ModelInstanceIndex model_instance =
      AddModelFromMeshFile(obj_path, "body");

  // We are done adding models.
  plant_.Finalize();
  auto context = plant_.CreateDefaultContext();

  const RigidBody<double>& body = plant_.GetBodyByName("body", model_instance);
  std::optional<geometry::FrameId> frame_id =
      plant_.GetBodyFrameIdIfExists(body.index());
  ASSERT_TRUE(frame_id.has_value());

  const geometry::SceneGraphInspector<double>& inspector =
      scene_graph_.model_inspector();

  EXPECT_EQ(inspector.NumGeometriesForFrameWithRole(
                *frame_id, geometry::Role::kIllustration),
            1);
  EXPECT_EQ(inspector.NumGeometriesForFrameWithRole(
                *frame_id, geometry::Role::kPerception),
            1);
  EXPECT_EQ(inspector.NumGeometriesForFrameWithRole(*frame_id,
                                                    geometry::Role::kProximity),
            1);

  // Confirm that geometries registered as meshes have empty proximity
  // properties -- the only property group is the default property group.
  const std::vector<geometry::GeometryId> proximity_geometries =
      inspector.GetGeometries(*frame_id, geometry::Role::kProximity);
  for (const geometry::GeometryId id : proximity_geometries) {
    const geometry::ProximityProperties* properties =
        inspector.GetProximityProperties(id);
    ASSERT_NE(properties, nullptr);
    EXPECT_THAT(properties->GetGroupNames(),
                ::testing::ElementsAre(properties->default_group_name()));
  }
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
