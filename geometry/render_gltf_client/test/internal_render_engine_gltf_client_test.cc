#include "drake/geometry/render_gltf_client/internal_render_engine_gltf_client.h"

#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <common_robotics_utilities/base64_helpers.hpp>
#include <fmt/ranges.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkCamera.h>     // vtkRenderingCore
#include <vtkMatrix4x4.h>  // vtkCommonMath

#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/memory_file.h"
#include "drake/common/scope_exit.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/geometry/render_gltf_client/internal_http_service.h"
#include "drake/geometry/render_gltf_client/internal_merge_gltf.h"
#include "drake/geometry/render_gltf_client/render_engine_gltf_client_params.h"
#include "drake/geometry/render_gltf_client/test/internal_sample_image_data.h"
#include "drake/geometry/scene_graph.h"

// This might *seem* to be unused, but don't remove it! We rely on this to dump
// images to the console when calling `EXPECT_EQ(Image<...>, Image<...>)`.
#include "drake/systems/sensors/test_utilities/image_compare.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

namespace fs = std::filesystem;

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;
using Eigen::Vector4d;

using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;
using nlohmann::json;
using render::ColorRenderCamera;
using render::DepthRenderCamera;
using render::RenderEngine;
using render_vtk::internal::ImageType;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

class RenderEngineGltfClientTester {
 public:
  using GltfRecord = RenderEngineGltfClient::GltfRecord;

  static const std::map<GeometryId, GltfRecord>& gltfs(
      const RenderEngineGltfClient& engine) {
    return engine.gltfs_;
  }

  static void ExportScene(const RenderEngineGltfClient& engine,
                          const std::filesystem::path& path,
                          ImageType image_type) {
    engine.ExportScene(path.string(), image_type);
  }
};

namespace {

using Params = RenderEngineGltfClientParams;

// Constexpr dimensions for the actual testing images.
constexpr int kTestImageWidth = 3;
constexpr int kTestImageHeight = 2;

class RenderEngineGltfClientTest : public ::testing::Test {
 public:
  RenderEngineGltfClientTest()
      : color_camera_{{"proxy_render",
                       {kTestImageWidth, kTestImageHeight, M_PI_4},
                       {0.11, 111.111},
                       {}},
                      false},
        depth_camera_{color_camera_.core(), {0.12, 10.0}} {}

 protected:
  /* The params to create the test RenderClient are to help ensure nothing
   actually gets sent over curl.  No test should proceed with the default
   HttpServiceCurl, the HttpService backend should be changed using
   client.SetHttpService before doing anything. */
  const Params params_{.base_url = "http://notarealserver:8192",
                       .render_endpoint = "no_render",
                       .verbose = true /* Increase test coverage. */};

  const ColorRenderCamera color_camera_;
  const DepthRenderCamera depth_camera_;
};

TEST_F(RenderEngineGltfClientTest, ParameterMatching) {
  auto make_yaml = [](const RenderEngineGltfClientParams& params) {
    return yaml::SaveYamlString(params, "RenderEngineGltfClientParams");
  };
  const RenderEngineGltfClientParams params1{.verbose = true};
  const RenderEngineGltfClientParams params2{.verbose = false};

  const RenderEngineGltfClient engine(params1);
  const std::string from_engine = engine.GetParameterYaml();

  EXPECT_EQ(from_engine, make_yaml(params1));
  EXPECT_NE(from_engine, make_yaml(params2));
}

TEST_F(RenderEngineGltfClientTest, Constructor) {
  // Reference values of default parameters; we'll use this to make sure that
  // the client is default initialized with default parameters.
  const Params default_params;

  // In the case where we provide parameters, we want to make sure that they
  // all get captured by the render engine. So, we'll make a set of parameters
  // that are *completely* different from default values. However params_ is
  // only mostly different -- it leaves `cleanup` enabled. We'll use it as the
  // starting point (leaving it intact so the behavior of all the other tests
  // which use it can rely on cleanup) and change `cleanup`.
  Params test_params(params_);
  test_params.cleanup = false;
  ASSERT_NE(test_params.GetUrl(), default_params.GetUrl());
  ASSERT_NE(test_params.verbose, default_params.verbose);
  ASSERT_NE(test_params.cleanup, default_params.cleanup);

  const RenderEngineGltfClient default_engine{};
  EXPECT_EQ(default_engine.get_params().GetUrl(), default_params.GetUrl());
  EXPECT_EQ(default_engine.get_params().verbose, default_params.verbose);
  EXPECT_EQ(default_engine.get_params().cleanup, default_params.cleanup);

  // Make sure that alternative values are passed to the underlying client.
  const RenderEngineGltfClient engine{test_params};
  EXPECT_EQ(engine.get_params().GetUrl(),
            "http://notarealserver:8192/no_render");
  EXPECT_EQ(engine.get_params().verbose, true);
  EXPECT_EQ(engine.get_params().cleanup, false);
}

TEST_F(RenderEngineGltfClientTest, Clone) {
  const RenderEngineGltfClient engine{params_};
  const std::unique_ptr<RenderEngine> clone = engine.Clone();
  RenderEngineGltfClient* clone_engine =
      dynamic_cast<RenderEngineGltfClient*>(clone.get());
  ASSERT_NE(clone_engine, nullptr);

  EXPECT_EQ(engine.get_params().GetUrl(), clone_engine->get_params().GetUrl());
  EXPECT_EQ(engine.get_params().verbose, clone_engine->get_params().verbose);
  EXPECT_EQ(engine.get_params().cleanup, clone_engine->get_params().cleanup);
  /* Cloning creates a new temporary directory, the underlying RenderClient is
   not cloned and as such a new temporary directory is created. */
  EXPECT_NE(engine.temp_directory(), clone_engine->temp_directory());
}

class FakeServer : public HttpService {
 public:
  FakeServer() = default;

  /* Copies one of the test images to fake the server response. */
  HttpResponse DoPostForm(const std::string& temp_directory,
                          const std::string& /* url */,
                          const DataFieldsMap& data_fields,
                          const FileFieldsMap& /* file_fields */,
                          bool /* verbose */) override {
    const std::string image_type = data_fields.at("image_type");
    const std::string data_path = fs::path(temp_directory) / "image.response";
    std::string test_image_path;
    if (image_type == "color") {
      test_image_path = FindResourceOrThrow(
          "drake/geometry/render_gltf_client/test/test_rgba_8U.png");
    } else if (image_type == "depth") {
      test_image_path = FindResourceOrThrow(
          "drake/geometry/render_gltf_client/test/test_depth_32F.tiff");
    } else {  // image_type := "label"
      test_image_path = FindResourceOrThrow(
          "drake/geometry/render_gltf_client/test/"
          "test_colored_label_rgba_8U.png");
    }
    fs::copy_file(test_image_path, data_path);
    return HttpResponse{.http_code = 200, .data_path = data_path};
  }
};

/* Returns all regular files in a directory (*NOT* recursive). */
std::vector<fs::path> FindRegularFiles(const std::string& directory) {
  std::vector<fs::path> files;
  for (const fs::path& file : fs::directory_iterator(directory)) {
    if (fs::is_regular_file(file)) {
      files.emplace_back(file);
    }
  }
  return files;
}

/* Helper function for checking the contents of a directory after a Render*Image
 call.  If `cleanup` is true, then there should not be any files found.  If
 false, then two files should exist: a .gltf file (from the export before
 sending to the server) and an image file (from the server).  The image file
 should have the same name as the .gltf file, only with a different file
 extension, e.g., `".png"` or `".tiff"`). */
void CheckExpectedFiles(const std::string& directory, bool cleanup,
                        const std::string& extension) {
  const std::vector<fs::path> files = FindRegularFiles(directory);
  if (cleanup) {
    EXPECT_EQ(files.size(), 0);
  } else {
    ASSERT_EQ(files.size(), 2);
    EXPECT_EQ(
        std::set<std::string>({files[0].extension(), files[1].extension()}),
        std::set<std::string>({".gltf", extension}));
    EXPECT_EQ(files[0].stem(), files[1].stem());
  }
}

TEST_F(RenderEngineGltfClientTest, DoRenderColorImage) {
  for (const bool cleanup : {true, false}) {
    RenderEngineGltfClient engine{Params{.cleanup = cleanup}};
    engine.SetHttpService(std::make_unique<FakeServer>());

    ImageRgba8U color_image{kTestImageWidth, kTestImageHeight};
    DRAKE_EXPECT_NO_THROW(engine.RenderColorImage(color_camera_, &color_image));

    // Make sure the temporary directory is / is not being cleaned up.
    CheckExpectedFiles(engine.temp_directory(), cleanup, ".png");
    // Make sure the image got loaded as expected.
    EXPECT_EQ(color_image, CreateTestColorImage(false));
  }
}

TEST_F(RenderEngineGltfClientTest, DoRenderDepthImage) {
  for (const bool cleanup : {true, false}) {
    RenderEngineGltfClient engine{Params{.cleanup = cleanup}};
    engine.SetHttpService(std::make_unique<FakeServer>());

    ImageDepth32F depth_image{kTestImageWidth, kTestImageHeight};
    DRAKE_EXPECT_NO_THROW(engine.RenderDepthImage(depth_camera_, &depth_image));

    // Make sure the temporary directory is / is not being cleaned up.
    CheckExpectedFiles(engine.temp_directory(), cleanup, ".tiff");

    // Make sure the image got loaded as expected.
    EXPECT_EQ(depth_image, CreateTestDepthImage());
  }
}

TEST_F(RenderEngineGltfClientTest, DoRenderLabelImage) {
  for (const bool cleanup : {true, false}) {
    RenderEngineGltfClient engine{Params{.cleanup = cleanup}};
    engine.SetHttpService(std::make_unique<FakeServer>());

    ImageLabel16I label_image{kTestImageWidth, kTestImageHeight};
    DRAKE_EXPECT_NO_THROW(engine.RenderLabelImage(color_camera_, &label_image));

    // Make sure the temporary directory is / is not being cleaned up.
    CheckExpectedFiles(engine.temp_directory(), cleanup, ".png");

    // Make sure the image got loaded as expected. Note that it also tests
    // whether a server-returned white pixel is properly converted to
    // render::RenderLabel::kDontCare.
    EXPECT_EQ(label_image, CreateTestLabelImage());
  }
}

class RenderEngineGltfClientGltfTest : public ::testing::Test {
 public:
  RenderEngineGltfClientGltfTest()
      : X_WG_(Vector3d(4, 5, 6)),
        label_(render::RenderLabel(7)),
        temp_dir_(temp_directory()) {
    properties_.AddProperty("label", "id", label_);
  }

 protected:
  using Tester = RenderEngineGltfClientTester;

  GeometryId AddObj() {
    const GeometryId id = GeometryId::get_new_id();
    EXPECT_TRUE(engine_.RegisterVisual(
        id,
        Mesh(FindResourceOrThrow(
                 "drake/geometry/render_gltf_client/test/tri.obj"),
             scale_),
        properties_, X_WG_, /* needs_update= */ true, "tri_obj"));
    EXPECT_TRUE(engine_.has_geometry(id));
    return id;
  }

  GeometryId AddGltf() {
    const GeometryId id = GeometryId::get_new_id();
    EXPECT_TRUE(engine_.RegisterVisual(
        id,
        Mesh(FindResourceOrThrow(
                 "drake/geometry/render_gltf_client/test/tri_tree.gltf"),
             scale_),
        properties_, X_WG_, /* needs_update= */ true, "tri_tree_gltf"));
    EXPECT_TRUE(engine_.has_geometry(id));
    return id;
  }

  /* Exports the glTF file from the engine, and reads it back as json. */
  json ExportAndReadJson(std::string_view file_name, ImageType image_type) {
    const std::filesystem::path file_path = temp_dir_ / file_name;
    Tester::ExportScene(engine_, file_path, image_type);
    return ReadJsonFile(file_path);
  }

  RenderEngineGltfClient engine_;
  PerceptionProperties properties_;
  const RigidTransformd X_WG_;
  const render::RenderLabel label_;
  const Vector3d scale_ = Vector3d(2, 3, 4);
  const std::filesystem::path temp_dir_;
};

/* RenderEngineGltfClient departs from RenderEngineVtk in registering Mesh. It
 defers to RenderEngineVtk for meshes with .obj extensions and handles
 everything else.

 We do limited testing of the obj case, merely confirm that registration
 reports true and the geometry id reports as registered.

 For the gltf case, we directly examine the stored data for the gltf.

 We also confirm that other extensions report as being unregistered. */
TEST_F(RenderEngineGltfClientGltfTest, RegisteringMeshes) {
  // If the extension is neither obj nor gltf, it is not accepted. (There is
  // also a warning, but we're not testing for that).
  EXPECT_FALSE(engine_.RegisterVisual(GeometryId::get_new_id(),
                                      Mesh("some.stl"), properties_, X_WG_));

  // Objs and glTFs are both accepted.
  AddObj();
  const GeometryId gltf_id = AddGltf();

  // Now look for proof that the gltf got added correctly.
  const std::map<GeometryId, Tester::GltfRecord>& gltfs =
      Tester::gltfs(engine_);
  ASSERT_TRUE(gltfs.contains(gltf_id));
  const Tester::GltfRecord& record = gltfs.at(gltf_id);
  EXPECT_TRUE(CompareMatrices(record.scale, scale_));
  EXPECT_EQ(record.label, label_);
  // There are three nodes in the gltf, but only two root nodes. One of the
  // root nodes is empty.
  EXPECT_EQ(record.contents["nodes"].size(), 3);
  EXPECT_EQ(record.root_nodes.size(), 2);
  // The root node data extracted directly from tri_tree.gltf.
  const std::map<int, Matrix4<double>> expected_roots{
      {1, RigidTransformd(Vector3d(1, 3, -2)).GetAsMatrix4()},    // root_tri
      {2, RigidTransformd(Vector3d(-1, -3, 2)).GetAsMatrix4()}};  // empty_root
  for (const int index : {1, 2}) {
    EXPECT_TRUE(
        CompareMatrices(record.root_nodes.at(index), expected_roots.at(index)))
        << "Index: " << index;
  }
  // The pose of the root node should have been set as part of its registration.
  // We're not testing it here because defining that pose is a bit more
  // elaborate. It is tested below in a dedicated test.
}

/* RenderEngineGltfClient should accept in-memory Mesh shapes. */
TEST_F(RenderEngineGltfClientGltfTest, InMemoryMeshes) {
  // The same .obj used in AddObj().
  const std::string obj_path =
      FindResourceOrThrow("drake/geometry/render_gltf_client/test/tri.obj");
  const GeometryId obj_id = GeometryId::get_new_id();
  EXPECT_TRUE(engine_.RegisterVisual(
      obj_id, Mesh(InMemoryMesh{MemoryFile::Make(obj_path)}), properties_,
      X_WG_));

  // The same .gltf used in AddGltf().
  const std::string gltf_path = FindResourceOrThrow(
      "drake/geometry/render_gltf_client/test/tri_tree.gltf");
  const GeometryId gltf_id = GeometryId::get_new_id();
  EXPECT_TRUE(engine_.RegisterVisual(
      gltf_id, Mesh(InMemoryMesh{MemoryFile::Make(gltf_path)}, scale_),
      properties_, X_WG_));

  const std::map<GeometryId, Tester::GltfRecord>& gltfs =
      Tester::gltfs(engine_);
  ASSERT_TRUE(gltfs.contains(gltf_id));
  const Tester::GltfRecord& record = gltfs.at(gltf_id);
  EXPECT_EQ(record.scale, scale_);
  EXPECT_EQ(record.label, label_);
  // There are three nodes in the gltf, but only two root nodes. One of the
  // root nodes is empty.
  EXPECT_EQ(record.contents["nodes"].size(), 3);
  EXPECT_EQ(record.root_nodes.size(), 2);
  // The root node data extracted directly from tri_tree.gltf.
  const std::map<int, Matrix4<double>> expected_roots{
      {1, RigidTransformd(Vector3d(1, 3, -2)).GetAsMatrix4()},    // root_tri
      {2, RigidTransformd(Vector3d(-1, -3, 2)).GetAsMatrix4()}};  // empty_root
  for (const int index : {1, 2}) {
    EXPECT_TRUE(
        CompareMatrices(record.root_nodes.at(index), expected_roots.at(index)))
        << "Index: " << index;
  }
}

/* If a file referenced in a file glTF file uri is missing, we should throw. */
TEST_F(RenderEngineGltfClientGltfTest, GltfMissingSupportingFile) {
  const fs::path src_path =
      FindResourceOrThrow("drake/geometry/render/test/meshes/cube2.gltf");
  const fs::path src_dir = src_path.parent_path();
  const fs::path temp_dir = temp_directory();
  const fs::path gltf_path = temp_dir / "cube2.gltf";
  fs::copy_file(src_path, gltf_path);

  const Mesh disk_mesh(gltf_path);
  const GeometryId disk_id = GeometryId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine_.RegisterVisual(disk_id, disk_mesh, properties_, X_WG_),
      "Error reading from.*cube2.bin.*");

  const Mesh memory_mesh(InMemoryMesh{MemoryFile::Make(gltf_path)});
  const GeometryId memory_id = GeometryId::get_new_id();
  DRAKE_EXPECT_THROWS_MESSAGE(
      engine_.RegisterVisual(memory_id, memory_mesh, properties_, X_WG_),
      ".*cube2.bin.*not contained within the supporting files.");
}

/* RenderEngineGltfClient has the responsibility of turning file URIs into
 data URIs and this must work with in-memory meshes as well.

 In this case, we pass it an embedded glTF file and confirm the file doesn't
 change; everything is already a data uri. */
TEST_F(RenderEngineGltfClientGltfTest, InMemoryDataUrisPreserved) {
  // tri_tree.gltf contains all the data in a URI.
  const fs::path gltf_path = FindResourceOrThrow(
      "drake/geometry/render_gltf_client/test/tri_tree.gltf");
  const GeometryId disk_id = GeometryId::get_new_id();
  EXPECT_TRUE(engine_.RegisterVisual(disk_id, Mesh(gltf_path.string()),
                                     properties_, X_WG_));
  const GeometryId memory_id = GeometryId::get_new_id();
  InMemoryMesh gltf{MemoryFile::Make(gltf_path)};
  EXPECT_TRUE(
      engine_.RegisterVisual(memory_id, Mesh(gltf), properties_, X_WG_));

  const std::map<GeometryId, Tester::GltfRecord>& gltfs =
      Tester::gltfs(engine_);
  ASSERT_TRUE(gltfs.contains(disk_id));
  ASSERT_TRUE(gltfs.contains(memory_id));
  const json& disk_contents = gltfs.at(disk_id).contents;
  const json& memory_contents = gltfs.at(memory_id).contents;
  const json ref_contents = json::parse(gltf.mesh_file.contents());

  // The two json structures should be *equivalent*. However, we can't
  // compare directly because the engine can make small changes (e.g.,
  // transformation matrices). So, we'll compare images and buffers directly.
  EXPECT_EQ(disk_contents.value("images", json{}),
            ref_contents.value("images", json{}));
  EXPECT_EQ(disk_contents.value("buffers", json{}),
            ref_contents.value("buffers", json{}));

  EXPECT_EQ(memory_contents.value("images", json{}),
            ref_contents.value("images", json{}));
  EXPECT_EQ(memory_contents.value("buffers", json{}),
            ref_contents.value("buffers", json{}));
}

/* RenderEngineGltfClient has the responsibility of turning file URIs into
 data URIs and this must work with in-memory meshes as well.

 In this case, the glTF file references external files. We'll confirm they get
 converted. */
TEST_F(RenderEngineGltfClientGltfTest, InMemorySupportingFilesToDataUris) {
  // cube3.gltf has two images and one .bin file. So, we'll see that they all
  // get processed.
  {
    const std::filesystem::path dir("drake/geometry/render/test/meshes");
    const std::filesystem::path gltf_path =
        FindResourceOrThrow(dir / "cube3.gltf");
    const std::string gltf_content = ReadFileOrThrow(gltf_path);

    string_map<FileSource> files;
    for (const char* file_name :
         {"cube3_normal.png", "cube3_divot.png", "cube3.bin"}) {
      files.insert(
          {file_name, MemoryFile::Make(FindResourceOrThrow(dir / file_name))});
    }

    const GeometryId disk_id = GeometryId::get_new_id();
    EXPECT_TRUE(
        engine_.RegisterVisual(disk_id, Mesh(gltf_path), properties_, X_WG_));

    const GeometryId memory_id = GeometryId::get_new_id();
    EXPECT_TRUE(engine_.RegisterVisual(
        memory_id, Mesh(InMemoryMesh{MemoryFile::Make(gltf_path), files}),
        properties_, X_WG_));

    const std::map<GeometryId, Tester::GltfRecord>& gltfs =
        Tester::gltfs(engine_);
    ASSERT_TRUE(gltfs.contains(disk_id));
    ASSERT_TRUE(gltfs.contains(memory_id));

    const json& disk_contents = gltfs.at(disk_id).contents;
    const json& memory_contents = gltfs.at(memory_id).contents;
    const json ref_contents = json::parse(gltf_content);

    auto make_data_uri = [](const std::string& content) {
      return fmt::format(
          "data:application/octet-stream;base64,{}",
          common_robotics_utilities::base64_helpers::Encode(
              std::vector<uint8_t>(content.begin(), content.end())));
    };

    for (std::string_view array : {"images", "buffers"}) {
      for (size_t i = 0; i < ref_contents[array].size(); ++i) {
        const std::string& name =
            ref_contents[array][i]["uri"].get<std::string>();
        const FileSource& file_source = files[name];
        const MemoryFile* memory_file = std::get_if<MemoryFile>(&file_source);
        DRAKE_DEMAND(memory_file != nullptr);
        const std::string ref_string = make_data_uri(memory_file->contents());
        for (const json* test : {&disk_contents, &memory_contents}) {
          SCOPED_TRACE(fmt::format("{}[{}] from {}", array, i,
                                   test == &disk_contents ? "disk" : "memory"));
          auto uri = (*test)[array][i]["uri"].get<std::string_view>();
          EXPECT_EQ(uri, ref_string);
        }
      }
    }
  }
}

/* Pose computation works as follows:

   - record.contents stores the glTF file *unmodified*. Root nodes retain their
     original file-frame transforms T_FN (translation/rotation/scale or matrix).
   - record.X_WG stores the current world pose and is updated cheaply by
     DoUpdateVisualPose().
   - At ExportScene() time, a single wrapper node is injected that:
       (a) carries record.geometry_name as its "name",
       (b) carries T_WF = X_WG * S_WG * X_GF as its "matrix", and
       (c) parents all of the source glTF's root nodes as "children".
     The effective world transform of each root node N is then T_WF * T_FN.

 This test verifies:
   1. record.X_WG correctly tracks the geometry's pose after registration and
      after UpdatePoses().
   2. record.contents root nodes are unmodified (retain original file format).
   3. The exported wrapper node carries the expected T_WF matrix and children.
 */
TEST_F(RenderEngineGltfClientGltfTest, PoseComputation) {
  const GeometryId gltf_id = AddGltf();

  /* Compute the expected wrapper-node matrix:
       T_WF = X_WG * S_WG * X_GF
     where S_WG scales the columns of X_WG and X_GF is the y-up-to-z-up
     rotation (MakeXRotation(π/2)). */
  auto compute_T_WF = [](const RigidTransformd& X_WG,
                         const Vector3d& scale_in) -> Matrix4<double> {
    Matrix4<double> T = X_WG.GetAsMatrix4();
    for (int i = 0; i < 3; ++i) T.block<3, 1>(0, i) *= scale_in(i);
    T *= RigidTransformd(RotationMatrixd::MakeXRotation(M_PI / 2))
             .GetAsMatrix4();
    return T;
  };

  const std::map<GeometryId, Tester::GltfRecord>& gltfs =
      Tester::gltfs(engine_);
  const Tester::GltfRecord& record = gltfs.at(gltf_id);

  // 1a. After registration, record.X_WG should hold the initial pose.
  EXPECT_TRUE(
      CompareMatrices(record.X_WG.GetAsMatrix4(), X_WG_.GetAsMatrix4(), 1e-15));

  // 2. Contents are unmodified: glTF-original root nodes retain their original
  // file-frame pose. In this case, we exploit the fact that all root nodes
  // in tri_tree.gltf have only translation components.
  // TODO(SeanCurtis-TRI): If this proves too brittle, we'll parse the original
  // glTF and get the node transformation data directly.
  for (const auto& [node_index, T_FN_expected] : record.root_nodes) {
    const json& root = record.contents["nodes"][node_index];
    SCOPED_TRACE(fmt::format("Node {}({})", root["name"].get<std::string>(),
                             node_index));
    EXPECT_FALSE(root.contains("matrix"));
    EXPECT_TRUE(root.contains("translation"));
  }

  // 1b. After UpdatePoses(), record.X_WG should hold the updated pose.
  const RigidTransformd X_WG_update(RollPitchYawd{M_PI * 0.5, 0, 3 * M_PI / 2},
                                    Vector3d(-3, 1, 2));
  engine_.UpdatePoses(
      std::unordered_map<GeometryId, RigidTransformd>{{gltf_id, X_WG_update}});
  EXPECT_TRUE(CompareMatrices(record.X_WG.GetAsMatrix4(),
                              X_WG_update.GetAsMatrix4(), 1e-15));

  // 3. Export and verify the wrapper node carries T_WF and the right children.
  const json exported = ExportAndReadJson("pose_test.gltf", ImageType::kColor);
  const json* wrapper_node{};
  for (const auto& node : exported["nodes"]) {
    if (node.contains("name") && node["name"] == "tri_tree_gltf") {
      wrapper_node = &node;
      break;
    }
  }
  ASSERT_NE(wrapper_node, nullptr)
      << "Wrapper node named 'tri_tree_gltf' not found in exported glTF";

  ASSERT_TRUE(wrapper_node->contains("matrix"));
  const Matrix4<double> T_WF_actual =
      EigenMatrixFromGltfMatrix((*wrapper_node)["matrix"]);
  const Matrix4<double> T_WF_expected = compute_T_WF(X_WG_update, scale_);
  EXPECT_TRUE(CompareMatrices(T_WF_actual, T_WF_expected, 1e-13));

  // The wrapper must parent exactly the same number of children as the source
  // glTF had root nodes (two for tri_tree.gltf: "root_tri" and "empty_root").
  ASSERT_TRUE(wrapper_node->contains("children"));
  EXPECT_EQ(ssize((*wrapper_node)["children"]), ssize(record.root_nodes));

  // The wrapper node itself should not reference any mesh directly; it is
  // a pure pose/grouping node.
  EXPECT_FALSE(wrapper_node->contains("mesh"));
}

/* ExportScene() is responsible for merging gltf geometries into the VTK-made
 gltf file. The merging process entails the following:

   - The gltf data is present (it's been merged).
     - We're not validating all of the data; we've got unit tests on the
       merging code. We'll simply look for evidence that it has merged: its
       root nodes.
   - If exporting for a label image, the materials should become simplified
     with only material.pbrMetallicRoughness.{baseColorFactor, emissiveFactor}.
 */
TEST_F(RenderEngineGltfClientGltfTest, ExportScene) {
  /* Base case where we have a single obj controlled by VTK. */
  AddObj();
  const json gltf_obj_only =
      ExportAndReadJson("only_obj.gltf", ImageType::kColor);
  /* We should have three nodes, the mesh and VTK's "Renderer Node" and "Camera
   Node".

   Note: in VTK logic, we only get the Camera Node because we have the mesh
   node. If we don't have at least one vtkActor in VTK proper, we won't get the
   camera node. */
  ASSERT_EQ(gltf_obj_only["nodes"].size(), 3);

  /* Now we add the gltf. It adds *four* nodes: the three in the glTF file
   ("empty_root", "root_tri", and "child_tri") and a novel root node based on
   the geometry name, "tri_tree_gltf". */
  AddGltf();

  const json gltf_color = ExportAndReadJson("color.gltf", ImageType::kColor);

  // These are the nodes we expect to see. We'll remove everyone we find. Note:
  // this doesn't account for extra nodes.
  std::set<std::string> expected_nodes{
      "Camera Node", "Renderer Node", "tri_tree_gltf", "root_tri",
      "empty_root",  "child_tri",     "tri_obj"};
  for (const auto& n : gltf_color["nodes"]) {
    const std::string& name = n["name"].get<std::string>();
    expected_nodes.erase(name);
  }
  EXPECT_EQ(expected_nodes.size(), 0)
      << fmt::to_string(fmt::join(expected_nodes, ", "));

  /* Finally, let's compare label and color. */
  auto find_mat_for_node = [](const json& gltf, std::string_view node_name) {
    const json* node{};
    for (const auto& n : gltf["nodes"]) {
      if (n["name"] == node_name) {
        node = &n;
        break;
      }
    }
    DRAKE_DEMAND(node != nullptr);
    const json& mesh = gltf["meshes"][(*node)["mesh"].get<int>()];
    /* We know there's only a single primitives in the "primitives" array. */
    const json& mat =
        gltf["materials"][mesh["primitives"][0]["material"].get<int>()];
    return mat;
  };

  const json& color_mat = find_mat_for_node(gltf_color, "root_tri");
  const json gltf_label = ExportAndReadJson("label.gltf", ImageType::kLabel);
  const json& label_mat = find_mat_for_node(gltf_label, "root_tri");
  EXPECT_NE(color_mat, label_mat);

  // We're not testing the actual differences. The fact that they're different
  // shows that something is being done. And observation of label images in the
  // wild will quickly report if what is being done is bad.

  // In the case of problems where we're writing the glTF file, we should throw
  // a (somewhat) meaningful message. We use an inaccessible file as proxy for
  // the set of conditions that might prevent a successful save to disk.
  const std::filesystem::path bad_file = temp_dir_ / "no_access.gltf";
  ScopeExit guard([&bad_file]() {
    if (std::filesystem::exists(bad_file)) {
      // Make sure the file can be deleted with the temp directory.
      std::filesystem::permissions(bad_file,
                                   std::filesystem::perms::owner_all |
                                       std::filesystem::perms::group_all |
                                       std::filesystem::perms::others_all);
    }
  });
  {
    std::ofstream f(bad_file);
    f << "test";
    f.close();
    ASSERT_FALSE(f.bad());
    std::filesystem::permissions(bad_file, std::filesystem::perms::owner_read);
  }

  DRAKE_EXPECT_THROWS_MESSAGE(
      ExportAndReadJson("no_access.gltf", ImageType::kColor),
      ".*Error writing exported scene.*");
}

/* As with registering geometry, we should be able to remove both kinds of
 geometries (gltf and non-gltf). So, we'll populate it and then strip them out
 again. Things don't throw and the exported glTF shows it's all gone. */
TEST_F(RenderEngineGltfClientGltfTest, RemoveGltf) {
  // Add two kinds of meshes for later removal.
  const GeometryId obj_id = AddObj();
  const GeometryId gltf_id = AddGltf();

  ASSERT_TRUE(engine_.has_geometry(obj_id));
  EXPECT_TRUE(engine_.RemoveGeometry(obj_id));
  EXPECT_FALSE(engine_.has_geometry(obj_id));

  ASSERT_TRUE(engine_.has_geometry(gltf_id));
  EXPECT_TRUE(engine_.RemoveGeometry(gltf_id));
  EXPECT_FALSE(engine_.has_geometry(gltf_id));

  const json gltf_empty = ExportAndReadJson("empty.gltf", ImageType::kColor);
  /* An empty scene has no nodes (not even renderer and camera). */
  ASSERT_EQ(gltf_empty["nodes"].size(), 0);
}

/* When geometry is registered with a specific name via RegisterVisual, those
 names should appear as node names in the exported glTF scene. This confirms
 that geometry names assigned in SceneGraph propagate all the way through to
 the exported representation. */
GTEST_TEST(RenderEngineGltfClientNamesTest, GeometryNamesInExportedScene) {
  // 1. Instantiate a SceneGraph.
  SceneGraph<double> scene_graph;
  const SourceId source_id = scene_graph.RegisterSource("test");

  // 2. & 3. Instantiate a RenderEngineGltfClient, keeping a raw pointer.
  auto engine = std::make_unique<RenderEngineGltfClient>();
  // We'll save a reference to the engine so we can (a) exercise the ExportScene
  // functionality and (b) easily test the cloned behavior.
  const RenderEngineGltfClient& engine_ref = *engine;

  // 4. Add the render engine to SceneGraph.
  scene_graph.AddRenderer("gltf_client", std::move(engine));

  // 5. Register geometries directly on the engine with known, distinct names.
  const RigidTransformd X_WG;
  PerceptionProperties properties;
  properties.AddProperty("label", "id", render::RenderLabel(1));

  const GeometryId obj_id = scene_graph.RegisterAnchoredGeometry(
      source_id, std::make_unique<GeometryInstance>(
                     X_WG,
                     Mesh(FindResourceOrThrow(
                              "drake/geometry/render_gltf_client/test/tri.obj"),
                          Vector3d::Ones()),
                     "my_obj_geom"));
  scene_graph.AssignRole(source_id, obj_id, properties);

  const GeometryId gltf_id = scene_graph.RegisterAnchoredGeometry(
      source_id,
      std::make_unique<GeometryInstance>(
          X_WG,
          Mesh(FindResourceOrThrow(
                   "drake/geometry/render_gltf_client/test/tri_tree.gltf"),
               Vector3d::Ones()),
          "my_gltf_geom"));
  scene_graph.AssignRole(source_id, gltf_id, properties);

  // 6. Export the scene.
  const fs::path temp_dir = temp_directory();
  const fs::path output_path = temp_dir / "names_test.gltf";
  RenderEngineGltfClientTester::ExportScene(engine_ref, output_path,
                                            ImageType::kColor);

  // 7. Read the exported glTF and verify the known names appear as node names.
  const json gltf = ReadJsonFile(output_path);
  std::set<std::string> node_names;
  for (const auto& node : gltf["nodes"]) {
    if (node.contains("name")) {
      node_names.insert(node["name"].get<std::string>());
    }
  }
  EXPECT_THAT(node_names, ::testing::Contains("my_obj_geom"));
  EXPECT_THAT(node_names, ::testing::Contains("my_gltf_geom"));

  // 8. Verify that cloning the engine preserves the geometry names.
  std::unique_ptr<RenderEngine> clone_base = engine_ref.Clone();
  auto* clone = dynamic_cast<RenderEngineGltfClient*>(clone_base.get());
  ASSERT_NE(clone, nullptr);

  const fs::path clone_output_path = temp_dir / "names_test_clone.gltf";
  RenderEngineGltfClientTester::ExportScene(*clone, clone_output_path,
                                            ImageType::kColor);

  const json clone_gltf = ReadJsonFile(clone_output_path);
  std::set<std::string> clone_node_names;
  for (const auto& node : clone_gltf["nodes"]) {
    if (node.contains("name")) {
      clone_node_names.insert(node["name"].get<std::string>());
    }
  }
  EXPECT_THAT(clone_node_names, ::testing::Contains("my_obj_geom"));
  EXPECT_THAT(clone_node_names, ::testing::Contains("my_gltf_geom"));
}

}  // namespace
}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
