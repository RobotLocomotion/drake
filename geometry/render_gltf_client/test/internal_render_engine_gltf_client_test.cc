#include "drake/geometry/render_gltf_client/internal_render_engine_gltf_client.h"

#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <common_robotics_utilities/base64_helpers.hpp>
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
        properties_, X_WG_));
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
        properties_, X_WG_));
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

/* Currently, our server expects us to output a z-up gltf (because (a) Drake is
 z-up and (b) VTK does not correct it). The input gltf files are y-up.
 Therefore, we have to do some special math to apply a z-up pose to a y-up
 object.

 We do this update twice: once when the geometry is registered and once
 when we call UpdatePoses(). This will check both.

 The transform located in the gltf contents should be as follows:

    T = X_t * X_r * S * R_zy * F

 where:
    X_t: a transform that translates the geometry.
    X_r: a transform that rotates the geometry.
    S: a transform that scales the geometry (possibly non-uniformly).
    R_zy: The rotation from y-up to z-up.
    F: The transform (not necessarily rigid) of the root node in the file.

 We'll extract the matrix X out of the glTF contents, and gradually remove
 each of the T, R, S, R_zy and compare the resultant F with that stored in the
 GltfRecord. */
TEST_F(RenderEngineGltfClientGltfTest, PoseComputation) {
  const GeometryId gltf_id = AddGltf();

  /* Given the non-rigid transform of a node (T_WN), the rigid pose of the
   file's frame (T_WF) and the scale factor, extract the transform of the node
   in the file's frame (T_FN). */
  auto extract_T_FN = [](const Matrix4<double> T_WN,
                         const RigidTransformd& X_WF,
                         const Vector3d& scale_in) -> Matrix4<double> {
    Matrix4<double> X_t_inv = Matrix4<double>::Identity();
    X_t_inv.block<3, 1>(0, 3) = -X_WF.translation();
    Matrix4<double> X_R_inv = Matrix4<double>::Identity();
    X_R_inv.block<3, 3>(0, 0) = X_WF.rotation().inverse().matrix();
    Matrix4<double> S_inv = Matrix4<double>::Identity();
    S_inv(0, 0) /= scale_in.x();
    S_inv(1, 1) /= scale_in.y();
    S_inv(2, 2) /= scale_in.z();
    const RigidTransformd X_zy_inv(RotationMatrixd::MakeXRotation(-M_PI / 2));
    const Matrix4<double> R_zy_inv = X_zy_inv.GetAsMatrix4();
    return R_zy_inv * S_inv * X_R_inv * X_t_inv * T_WN;
  };

  // Extract the transform matrix from a node's specification. Also validate
  // that it *only* includes "matrix" and not "translation", "rotation", or
  // "scale".
  auto extract_T_WN = [](const json& node) {
    EXPECT_FALSE(node.contains("translation"));
    EXPECT_FALSE(node.contains("rotation"));
    EXPECT_FALSE(node.contains("scale"));
    EXPECT_TRUE(node.contains("matrix"));
    return EigenMatrixFromGltfMatrix(node["matrix"]);
  };

  // Previous tests already showed that there's a single root node. We'll get
  // that node's T_FN.
  const std::map<GeometryId, Tester::GltfRecord>& gltfs =
      Tester::gltfs(engine_);
  const Tester::GltfRecord& record = gltfs.at(gltf_id);
  for (const auto& [node_index, T_FN_expected] : record.root_nodes) {
    const json& root = record.contents["nodes"][node_index];
    SCOPED_TRACE(fmt::format("Node {}({})", root["name"].get<std::string>(),
                             node_index));
    // Confirm computation of X_WN during registration.
    const Matrix4<double> X_WN_init = extract_T_WN(root);
    const Matrix4<double> T_FN_init = extract_T_FN(X_WN_init, X_WG_, scale_);
    EXPECT_TRUE(CompareMatrices(T_FN_init, T_FN_expected, 1e-15));
  }

  const RigidTransformd X_WG_update(RollPitchYawd{M_PI * 0.5, 0, 3 * M_PI / 2},
                                    Vector3d(-3, 1, 2));
  engine_.UpdatePoses(
      std::unordered_map<GeometryId, RigidTransformd>{{gltf_id, X_WG_update}});
  for (const auto& [node_index, T_FN_expected] : record.root_nodes) {
    const json& root = record.contents["nodes"][node_index];
    SCOPED_TRACE(fmt::format("Node {}({})", root["name"].get<std::string>(),
                             node_index));
    // Confirm computation of X_WN during UpdatePoses.
    const Matrix4<double> T_WN_update = extract_T_WN(root);
    const Matrix4<double> T_FN_update =
        extract_T_FN(T_WN_update, X_WG_update, scale_);
    EXPECT_TRUE(CompareMatrices(T_FN_update, T_FN_expected, 1e-15));
  }
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
   Node". */
  ASSERT_EQ(gltf_obj_only["nodes"].size(), 3);

  /* Now we add the gltf. It adds three nodes: "empty_root", "root_tri", and
   "child_tri". */
  AddGltf();

  const json gltf_color = ExportAndReadJson("color.gltf", ImageType::kColor);
  ASSERT_EQ(gltf_color["nodes"].size(), 6);
  bool root_tri_present = false;
  bool empty_root_present = false;
  for (const auto& n : gltf_color["nodes"]) {
    const std::string& name = n["name"].get<std::string>();
    root_tri_present = root_tri_present || name == "root_tri";
    empty_root_present = empty_root_present || name == "empty_root";
  }
  EXPECT_TRUE(root_tri_present);
  EXPECT_TRUE(empty_root_present);

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

}  // namespace
}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
