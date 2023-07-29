#include "drake/geometry/render_gltf_client/internal_render_engine_gltf_client.h"

#include <filesystem>
#include <fstream>
#include <set>
#include <vector>

#include <drake_vendor/nlohmann/json.hpp>
#include <gtest/gtest.h>
#include <vtkCamera.h>
#include <vtkMatrix4x4.h>

#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/scope_exit.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/render_gltf_client/internal_http_service.h"
#include "drake/geometry/render_gltf_client/internal_merge_gltf.h"
#include "drake/geometry/render_gltf_client/render_engine_gltf_client_params.h"
#include "drake/geometry/render_gltf_client/test/internal_sample_image_data.h"

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
  EXPECT_EQ(engine.get_params().cleanup,
            clone_engine->get_params().cleanup);
  /* Cloning creates a new temporary directory, the underlying RenderClient is
   not cloned and as such a new temporary directory is created. */
  EXPECT_NE(engine.temp_directory(), clone_engine->temp_directory());
}

// TODO(svenevs): remove when VTK is updated, see implementation for details.
TEST_F(RenderEngineGltfClientTest, UpdateViewpoint) {
  RenderEngineGltfClient engine{params_};
  const Vector3d translation{0.8, 0.0, 0.5};
  const RigidTransformd X_WB(RollPitchYawd{-M_PI * 0.7, 0, M_PI / 2},
                             translation);

  // First, use the RenderEngineVtk::UpdateViewpoint and gather matrices.
  engine.RenderEngineVtk::UpdateViewpoint(X_WB);
  const Matrix4d vtk_color_mat =
      engine.CameraModelViewTransformMatrix(ImageType::kColor);
  const Matrix4d vtk_depth_mat =
      engine.CameraModelViewTransformMatrix(ImageType::kDepth);
  const Matrix4d vtk_label_mat =
      engine.CameraModelViewTransformMatrix(ImageType::kLabel);

  // Use the RenderEngineGltfClient::UpdateViewpoint and gather new matrices.
  engine.UpdateViewpoint(X_WB);
  const Matrix4d gltf_color_mat =
      engine.CameraModelViewTransformMatrix(ImageType::kColor);
  const Matrix4d gltf_depth_mat =
      engine.CameraModelViewTransformMatrix(ImageType::kDepth);
  const Matrix4d gltf_label_mat =
      engine.CameraModelViewTransformMatrix(ImageType::kLabel);

  /* A ModelView transform can be inverted directly by inverting the rotation
   via transpose, and using this inverted rotation to rotate the negated
   translation.  We do not need to be concerned about non-uniform scaling on the
   diagonal or the bottom row. */
  auto maybe_transform_inverse = [](const Eigen::Matrix4d& mat) {
    Matrix4d inverse{mat};
  #if VTK_VERSION_NUMBER > VTK_VERSION_CHECK(9, 1, 0)
      return inverse;  // The bug (vtk#8883) was fixed in 9.1.0.
  #else
    // Invert the rotation via transpose.
    inverse.topLeftCorner<3, 3>().transposeInPlace();
    // Rotate the inverted translation.
    const Vector3d t = inverse.topRightCorner<3, 1>();
    const Vector3d t_inv = inverse.topLeftCorner<3, 3>() * (-t);
    inverse.topRightCorner<3, 1>() = t_inv;
    return inverse;
  #endif
  };
  auto compare = [&](const Matrix4d& vtk, const Matrix4d& gltf) {
    constexpr double tolerance = 1e-9;

    /* We expect that the MVT from RenderEngineVtk and RenderEngineGltfClient
     the inverse of one another.  The RenderEngineGltfClient::UpdateViewpoint
     method engineers a hacked matrix to result in the effect of
     RenderEngineVtk::UpdateViewpoint modifying the underlying vtkCamera to
     produce the inverted matrix.  This is because the vtkGLTFExporter was
     incorrectly exporting the vtkCamera's ModelViewTransformMatrix, which
     transforms objects in the world into the view of the camera.  The inverse
     of this matrix is what was supposed to be exported, since the "nodes" array
     in glTF is for world coordinate transforms. */
    // Compare the rotations.
    const Matrix4d vtk_inv = maybe_transform_inverse(vtk);
    const Matrix3d R_vtk_inv = vtk_inv.topLeftCorner<3, 3>();
    const Matrix3d R_gltf = gltf.topLeftCorner<3, 3>();
    EXPECT_TRUE(R_vtk_inv.isApprox(R_gltf, tolerance)) << fmt::format(
        "Rotation matrices not similar enough:\n"
        "R_vtk_inv:\n{}\nR_gltf:\n{}\n",
        fmt_eigen(R_vtk_inv), fmt_eigen(R_gltf));

    // Compare the translations.
    const Vector3d t_vtk_inv = vtk_inv.topRightCorner<3, 1>();
    const Vector3d t_gltf = gltf.topRightCorner<3, 1>();
    EXPECT_TRUE(t_vtk_inv.isApprox(t_gltf, tolerance)) << fmt::format(
        "Translation vectors not similar enough:\n"
        "t_vtk_inv:\n{}\nt_gltf:\n{}\n",
        fmt_eigen(t_vtk_inv), fmt_eigen(t_gltf));

    // For posterity, compare the homogeneous row.
    const Vector4d gltf_homogeneous = gltf.bottomRightCorner<1, 4>();
    EXPECT_EQ(gltf_homogeneous, Vector4d(0, 0, 0, 1));
  };
  compare(vtk_color_mat, gltf_color_mat);
  compare(vtk_depth_mat, gltf_depth_mat);
  compare(vtk_label_mat, gltf_label_mat);
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
  const double scale_ = 2.0;
  const std::filesystem::path temp_dir_;
};

/* RenderEngineGltfClient departs from RenderEngineVtk in registering Mesh and
 Convex. It defers to RenderEngineVtk for meshes with .obj extensions and
 handles everything else.

 We do limited testing of the obj case, merely confirm that registration
 reports true and the geometry id reports as registered.

 For the gltf case, we directly examine the stored data for the gltf.

 We also confirm that other extensions report as being unregistered. */
TEST_F(RenderEngineGltfClientGltfTest, RegisteringMeshes) {
  // If the extension is neither obj nor gltf, it is not accepted. (There is
  // also a warning, but we're not testing for that).
  EXPECT_FALSE(engine_.RegisterVisual(GeometryId::get_new_id(),
                                      Mesh("some.stl"), properties_, X_WG_));
  // This is the only time we'll explicitly test Convex. Given Mesh and Convex
  // yield similar results *here*, we'll assume they'll be similar for
  // successful registration as well.
  EXPECT_FALSE(engine_.RegisterVisual(GeometryId::get_new_id(),
                                      Convex("some.stl"), properties_, X_WG_));

  // Objs and glTFs are both accepted.
  AddObj();
  const GeometryId gltf_id = AddGltf();

  // Now look for proof that the gltf got added correctly.
  const std::map<GeometryId, Tester::GltfRecord>& gltfs =
      Tester::gltfs(engine_);
  ASSERT_EQ(gltfs.count(gltf_id), 1);
  const Tester::GltfRecord& record = gltfs.at(gltf_id);
  EXPECT_EQ(record.scale, scale_);
  EXPECT_EQ(record.label, label_);
  // There are two nodes in the gltf, but only one root node.
  EXPECT_EQ(record.contents["nodes"].size(), 2);
  EXPECT_EQ(record.root_nodes.size(), 1);
  // The pose of the stored matrix is drawn from the gltf directly.
  EXPECT_EQ(record.contents["nodes"][record.root_nodes.begin()->first]["name"],
            "root_tri");
  EXPECT_TRUE(
      CompareMatrices(record.root_nodes.begin()->second,
                      RigidTransformd(Vector3d(1, 3, -2)).GetAsMatrix4()));
  // The pose of the root node should have been set as part of its registration.
  // We're not testing it here because defining that pose is a bit more
  // elaborate. It is tested below in a dedicated test.
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
    S: a transform that uniformly scales the geometry.
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
                         double scale_in) -> Matrix4<double> {
    Matrix4<double> X_t_inv = Matrix4<double>::Identity();
    X_t_inv.block<3, 1>(0, 3) = -X_WF.translation();
    Matrix4<double> X_R_inv = Matrix4<double>::Identity();
    X_R_inv.block<3, 3>(0, 0) = X_WF.rotation().inverse().matrix();
    Matrix4<double> S_inv = Matrix4<double>::Identity();
    S_inv(0, 0) = S_inv(1, 1) = S_inv(2, 2) = 1.0 / scale_in;
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
  const Matrix4<double>& T_FN_expected = record.root_nodes.begin()->second;

  // Now confirm its pose was updated during registration to X_WG and scale.
  const json& root = record.contents["nodes"][record.root_nodes.begin()->first];
  const Matrix4<double> X_WN_init = extract_T_WN(root);
  const Matrix4<double> T_FN_init = extract_T_FN(X_WN_init, X_WG_, scale_);
  EXPECT_TRUE(CompareMatrices(T_FN_init, T_FN_expected, 1e-15));

  // Now we'll pose the geometry and confirm the pose is as expected.
  const RigidTransformd X_WG_update(RollPitchYawd{M_PI * 0.5, 0, 3 * M_PI / 2},
                                    Vector3d(-3, 1, 2));
  engine_.UpdatePoses(
      std::unordered_map<GeometryId, RigidTransformd>{{gltf_id, X_WG_update}});
  const Matrix4<double> T_WN_update = extract_T_WN(root);
  const Matrix4<double> T_FN_update =
      extract_T_FN(T_WN_update, X_WG_update, scale_);
  EXPECT_TRUE(CompareMatrices(T_FN_update, T_FN_expected, 1e-15));
}

/* ExportScene() is responsible for merging gltf geometries into the VTK-made
 gltf file. The merging process entails the following:

   - The gltf data is present (it's been merged).
     - We're not validating all of the data; we've got unit tests on the
       merging code. We'll simply look for evidence that it has merged: its
       root node.
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

  /* Now we add the gltf. It adds two nodes: "root_tri" and "child_tri". */
  AddGltf();

  const json gltf_color  = ExportAndReadJson("color.gltf", ImageType::kColor);
  ASSERT_EQ(gltf_color["nodes"].size(), 5);
  bool gltf_root_present = false;
  for (const auto& n : gltf_color["nodes"]) {
    if (n["name"].get<std::string>() == "root_tri") {
      gltf_root_present = true;
      break;
    }
  }
  EXPECT_TRUE(gltf_root_present);

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
