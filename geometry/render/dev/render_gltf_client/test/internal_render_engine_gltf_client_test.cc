#include "drake/geometry/render/dev/render_gltf_client/internal_render_engine_gltf_client.h"

#include <fstream>

#include <fmt/format.h>
#include <gtest/gtest.h>
#include <vtkCamera.h>
#include <vtkMatrix4x4.h>

#include "drake/common/filesystem.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/geometry/render/dev/render_gltf_client/factory.h"
#include "drake/geometry/render_gltf_client/internal_http_service.h"
#include "drake/geometry/render_gltf_client/render_engine_gltf_client_params.h"
#include "drake/geometry/render_gltf_client/test/internal_sample_image_data.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

namespace fs = drake::filesystem;

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;
using Eigen::Vector4d;

using geometry::render::ColorRenderCamera;
using geometry::render::DepthRenderCamera;
using geometry::render::internal::ImageType;
using math::RigidTransformd;
using math::RollPitchYawd;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

// Exposes various private access methods and data members for validation.
class RenderEngineGltfClientTester {
 public:
  explicit RenderEngineGltfClientTester(RenderEngineGltfClient* engine)
      : engine_{engine} {
    DRAKE_DEMAND(engine_ != nullptr);
  }

  // RenderClient access methods.
  const RenderEngineGltfClientParams& get_params() const {
    return engine_->render_client_->get_params();
  }

  const std::string& temp_directory() const {
    return engine_->render_client_->temp_directory();
  }

  // Allows the HttpService backend to be swapped out.
  void SetHttpService(std::unique_ptr<HttpService> service) {
    engine_->render_client_->SetHttpService(std::move(service));
  }

  // RenderEngineGltfClient private method access.
  std::string ExportPathFor(ImageType image_type, int64_t scene_id) const {
    return engine_->ExportPathFor(image_type, scene_id);
  }

  std::string ExportScene(ImageType image_type, int64_t scene_id) const {
    return engine_->ExportScene(image_type, scene_id);
  }

  void CleanupFrame(const std::string& scene_path,
                    const std::string& image_path) const {
    engine_->CleanupFrame(scene_path, image_path);
  }

  Eigen::Matrix4d CameraModelViewTransformMatrix(ImageType image_type) const {
    return engine_->CameraModelViewTransformMatrix(image_type);
  }

 private:
  RenderEngineGltfClient* engine_{nullptr};
};

namespace {

using Tester = RenderEngineGltfClientTester;
using Engine = RenderEngineGltfClient;
using Params = RenderEngineGltfClientParams;

// Constexpr dimensions for the actual testing images.
constexpr int kTestImageWidth = 3;
constexpr int kTestImageHeight = 2;

GTEST_TEST(RenderEngineGltfClient, Constructor) {
  std::string temp_dir;
  {
    // Test the default construction values via the factory function.
    const auto engine = MakeRenderEngineGltfClient({});
    auto* actual_engine = dynamic_cast<RenderEngineGltfClient*>(engine.get());
    EXPECT_NE(actual_engine, nullptr);
    Tester tester{actual_engine};
    EXPECT_EQ(tester.get_params().GetUrl(), "http://127.0.0.1:8000/render");
    EXPECT_EQ(tester.get_params().verbose, false);
    EXPECT_EQ(tester.get_params().no_cleanup, false);

    temp_dir = tester.temp_directory();
    EXPECT_TRUE(fs::is_directory(temp_dir));
  }  // engine out of scope, deleted
  EXPECT_FALSE(fs::is_directory(temp_dir));

  {
    // Make sure that alternative values are passed to the underlying client.
    const Params params{"0.0.0.0:8000", "super_render", std::nullopt, true,
                        true};
    Engine engine{params};
    Tester tester{&engine};
    EXPECT_EQ(tester.get_params().GetUrl(), "0.0.0.0:8000/super_render");
    EXPECT_EQ(tester.get_params().verbose, true);
    EXPECT_EQ(tester.get_params().no_cleanup, true);

    temp_dir = tester.temp_directory();
    EXPECT_TRUE(fs::is_directory(temp_dir));
  }  // engine out of scope, deleted, no_cleanup was true
  EXPECT_TRUE(fs::is_directory(temp_dir));
  fs::remove(temp_dir);
}

GTEST_TEST(RenderEngineGltfClient, Clone) {
  const Params params{"192.168.1.1:2222", "endpoint",
                      std::nullopt,  true, false};
  Engine engine{params};
  Tester tester{&engine};

  const auto clone = engine.Clone();
  auto* actual_engine = dynamic_cast<RenderEngineGltfClient*>(clone.get());
  ASSERT_NE(actual_engine, nullptr);
  Tester clone_tester{actual_engine};

  EXPECT_EQ(tester.get_params().GetUrl(), clone_tester.get_params().GetUrl());
  EXPECT_EQ(tester.get_params().verbose, clone_tester.get_params().verbose);
  EXPECT_EQ(tester.get_params().no_cleanup,
            clone_tester.get_params().no_cleanup);
  /* Cloning creates a new temporary directory, the underlying RenderClient is
   not cloned and as such a new temporary directory is created. */
  EXPECT_NE(tester.temp_directory(), clone_tester.temp_directory());
}

// TODO(svenevs): remove when VTK is updated, see implementation for details.
GTEST_TEST(RenderEngineGltfClient, UpdateViewpoint) {
  const Params params;
  Engine engine{params};
  Tester tester{&engine};
  const Vector3d translation{0.8, 0.0, 0.5};
  const RigidTransformd X_WB(RollPitchYawd{-M_PI * 0.7, 0, M_PI / 2},
                             translation);

  // First, use the RenderEngineVtk::UpdateViewpoint and gather matrices.
  engine.RenderEngineVtk::UpdateViewpoint(X_WB);
  Matrix4d vtk_color_mat =
      tester.CameraModelViewTransformMatrix(ImageType::kColor);
  Matrix4d vtk_depth_mat =
      tester.CameraModelViewTransformMatrix(ImageType::kDepth);
  Matrix4d vtk_label_mat =
      tester.CameraModelViewTransformMatrix(ImageType::kLabel);

  // Use the RenderEngineGltfClient::UpdateViewpoint and gather new matrices.
  engine.UpdateViewpoint(X_WB);
  Matrix4d gltf_color_mat =
      tester.CameraModelViewTransformMatrix(ImageType::kColor);
  Matrix4d gltf_depth_mat =
      tester.CameraModelViewTransformMatrix(ImageType::kDepth);
  Matrix4d gltf_label_mat =
      tester.CameraModelViewTransformMatrix(ImageType::kLabel);

  /* A ModelView transform can be inverted directly by inverting the rotation
   via transpose, and using this inverted rotation to rotate the negated
   translation.  We do not need to be concerned about non-uniform scaling on the
   diagonal. */
  auto pseudo_inverse = [](const Eigen::Matrix4d& mat) {
    Matrix4d inverse{mat};
    // Invert the rotation via transpose.
    inverse.topLeftCorner<3, 3>().transposeInPlace();
    // Rotate the inverted translation.
    const Vector3d t = inverse.topRightCorner<3, 1>();
    const Vector3d t_inv = inverse.topLeftCorner<3, 3>() * (t * -1.0);
    inverse.topRightCorner<3, 1>() = t_inv;
    return inverse;
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
    const Matrix4d vtk_inv = pseudo_inverse(vtk);
    const Matrix3d R_vtk_inv = vtk_inv.topLeftCorner<3, 3>();
    const Matrix3d R_gltf = gltf.topLeftCorner<3, 3>();
    EXPECT_TRUE(R_vtk_inv.isApprox(R_gltf, tolerance))
        << "Rotation matrices not similar enough:\nR_vtk_inv:\n"
        << R_vtk_inv << "\nR_gltf:\n"
        << R_gltf << '\n';

    // Compare the translations.
    const Vector3d t_vtk_inv = vtk_inv.topRightCorner<3, 1>();
    const Vector3d t_gltf = gltf.topRightCorner<3, 1>();
    EXPECT_TRUE(t_vtk_inv.isApprox(t_gltf, tolerance))
        << "Translation vectors not similar enough:\nt_vtk_inv:\n"
        << t_vtk_inv << "\nt_gltf:\n"
        << t_gltf << '\n';

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

  /* Writes a testable image to temp_directory, either named color.response,
   depth.response, or label.response.  It will overwrite the file if it already
   exists. */
  HttpResponse DoPostForm(const std::string& temp_directory,
                          const std::string& url,
                          const DataFieldsMap& data_fields,
                          const FileFieldsMap& file_fields,
                          bool /* verbose */ = false) override {
    static std::atomic<int64_t> post_id{0};

    const auto image_type = data_fields.at("image_type");
    HttpResponse ret;
    ret.http_code = 200;

    const std::string path =
        fs::path(temp_directory) /
        fmt::format("{:0>19}-{}.response", ++post_id, image_type);
    std::string test_image_path;
    if (image_type == "color") {
      test_image_path = FindResourceOrThrow(
          "drake/geometry/render_gltf_client/test/test_rgba_8U.png");
    } else if (image_type == "depth") {
      test_image_path = FindResourceOrThrow(
          "drake/geometry/render_gltf_client/test/test_depth_32F.tiff");
    } else {  // image_type := "label"
      test_image_path = FindResourceOrThrow(
          "drake/geometry/render_gltf_client/test/test_label_16I.png");
    }
    fs::copy_file(test_image_path, path);
    ret.data_path = path;

    return ret;
  }
};

/* Returns all regular files found in directory (*NOT* recursive). */
std::vector<fs::path> FindRegularFiles(const std::string& directory) {
  std::vector<fs::path> files;
  for (const auto& item : fs::directory_iterator(directory)) {
    if (fs::is_regular_file(item)) {
      files.emplace_back(item);
    }
  }

  return files;
}

/* Helper method for checking the contents of the temp_directory after a
 Render*Image call.  If `no_cleanup` is false, then there should not be any
 files found.  If true, then two files should exist: a .gltf file (from the
 export before sending to the server) and an image file (from the server).  The
 image file should have the same name as the .gltf file, only with a different
 `image_extension` file extension (e.g., `".png"` or `".tiff"`). */
void CheckTempDirectoryForExpectedFiles(const std::string& temp_directory,
                                        bool no_cleanup,
                                        const std::string& image_extension) {
  auto files = FindRegularFiles(temp_directory);
  if (!no_cleanup) {
    // There should not be any files found.
    EXPECT_EQ(files.size(), 0);
  } else {
    // There should be: 1 glTF scene file, and one (renamed) image response.
    ASSERT_EQ(files.size(), 2);
    // Response should have been renamed to <scene>.<image_extension>.
    const bool gltf_image = files[0].extension() == ".gltf" &&
                            files[1].extension() == image_extension;
    const bool image_gltf = files[1].extension() == ".gltf" &&
                            files[0].extension() == image_extension;
    EXPECT_TRUE(gltf_image || image_gltf);
    EXPECT_EQ(files[0].replace_extension(""), files[1].replace_extension(""));
  }
}

GTEST_TEST(RenderEngineGltfClient, DoRenderColorImage) {
  ImageRgba8U color_image{kTestImageWidth, kTestImageHeight};
  const ColorRenderCamera color_camera{
      {"proxy_render",
       {kTestImageWidth, kTestImageHeight, M_PI_4},
       {0.11, 111.111},
       {}},
      false};

  for (const bool no_cleanup : {true, false}) {
    Params params;
    params.verbose = true;  // Increase coverage for logging calls.
    params.no_cleanup = no_cleanup;
    Engine engine{params};
    Tester tester{&engine};
    tester.SetHttpService(std::make_unique<FakeServer>());

    DRAKE_EXPECT_NO_THROW(engine.RenderColorImage(color_camera, &color_image));

    // Make sure the temporary directory is / is not being cleaned up.
    CheckTempDirectoryForExpectedFiles(tester.temp_directory(), no_cleanup,
                                       ".png");

    // Make sure the image got loaded as expected.
    EXPECT_EQ(color_image, CreateTestColorImage(false));
  }
}

GTEST_TEST(RenderEngineGltfClient, DoRenderDepthImage) {
  ImageDepth32F depth_image{kTestImageWidth, kTestImageHeight};
  const DepthRenderCamera depth_camera{
      {"proxy_render",
       {kTestImageWidth, kTestImageHeight, M_PI_4},
       {0.11, 111.111},
       {}},
      {0.12, 10.0}};

  for (const bool no_cleanup : {true, false}) {
    Params params;
    params.verbose = true;  // Increase coverage for logging calls.
    params.no_cleanup = no_cleanup;
    Engine engine{params};
    Tester tester{&engine};
    tester.SetHttpService(std::make_unique<FakeServer>());

    DRAKE_EXPECT_NO_THROW(engine.RenderDepthImage(depth_camera, &depth_image));

    // Make sure the temporary directory is / is not being cleaned up.
    CheckTempDirectoryForExpectedFiles(tester.temp_directory(), no_cleanup,
                                       ".tiff");

    // Make sure the image got loaded as expected.
    EXPECT_EQ(depth_image, CreateTestDepthImage());
  }
}

GTEST_TEST(RenderEngineGltfClient, DoRenderLabelImage) {
  ImageLabel16I label_image{kTestImageWidth, kTestImageHeight};
  const ColorRenderCamera color_camera{
      {"proxy_render",
       {kTestImageWidth, kTestImageHeight, M_PI_4},
       {0.11, 111.111},
       {}},
      false};

  for (const bool no_cleanup : {true, false}) {
    Params params;
    params.verbose = true;  // Increase coverage for logging calls.
    params.no_cleanup = no_cleanup;
    Engine engine{params};
    Tester tester{&engine};
    tester.SetHttpService(std::make_unique<FakeServer>());

    DRAKE_EXPECT_NO_THROW(engine.RenderLabelImage(color_camera, &label_image));

    // Make sure the temporary directory is / is not being cleaned up.
    CheckTempDirectoryForExpectedFiles(tester.temp_directory(), no_cleanup,
                                       ".png");

    // Make sure the image got loaded as expected.
    EXPECT_EQ(label_image, CreateTestLabelImage());
  }
}

// Tests for both ExportPathFor and ExportScene.
GTEST_TEST(RenderEngineGltfClient, Export) {
  const Params params;
  Engine engine{params};
  Tester tester{&engine};

  for (const auto image_type :
       {ImageType::kColor, ImageType::kDepth, ImageType::kLabel}) {
    for (const int64_t scene_id : {1, 11, 111}) {  // validate name formatting
      // Reconstruct the expected export path.
      std::string image_type_str;
      if (image_type == ImageType::kColor) {
        image_type_str = "color";
      } else if (image_type == ImageType::kDepth) {
        image_type_str = "depth";
      } else {  // image_type := ImageType::kLabel
        image_type_str = "label";
      }
      const std::string basename =
          fmt::format("{:0>19}-{}.gltf", scene_id, image_type_str);
      const std::string expected = fs::path(tester.temp_directory()) / basename;

      EXPECT_EQ(expected, tester.ExportPathFor(image_type, scene_id));

      // Export the scene, make sure it is there, then delete it.
      const auto exported = tester.ExportScene(image_type, scene_id);
      EXPECT_EQ(expected, exported);
      EXPECT_TRUE(fs::is_regular_file(exported));
      fs::remove(exported);
    }
  }
}

GTEST_TEST(RenderEngineGltfClient, CleanupFrame) {
  Params params;
  params.verbose = true;  // Increase coverage for logging calls.
  auto make_file = [](const std::string& path, const std::string& text) {
    std::ofstream f{path};
    f << text;
    f.close();
  };

  Engine engine{params};
  Tester tester{&engine};

  // Create two file paths to experiment with.
  const auto temp_dir = fs::path(tester.temp_directory());
  const auto sub_dir = temp_dir / "subdirectory";
  fs::create_directory(sub_dir);
  const std::string scene_path = temp_dir / "fake_scene.gltf";
  const auto scene_contents = "Not a real glTF!\n";
  const std::string image_path = sub_dir / "fake_image.png";
  const auto image_contents = "Not a real PNG!\n";

  // Case 1: successful deletion.
  make_file(scene_path, scene_contents);
  make_file(image_path, image_contents);
  EXPECT_TRUE(fs::is_regular_file(scene_path));
  EXPECT_TRUE(fs::is_regular_file(image_path));
  tester.CleanupFrame(scene_path, image_path);
  EXPECT_FALSE(fs::is_regular_file(scene_path));
  EXPECT_FALSE(fs::is_regular_file(image_path));

  // Case 2: engineer failure scenario that cannot delete a file.
  make_file(scene_path, scene_contents);
  make_file(image_path, image_contents);
  // Removing directory permissions: image_path cannot be deleted.
  const auto orig_perms = fs::status(sub_dir).permissions();
  EXPECT_TRUE(fs::is_regular_file(scene_path));
  EXPECT_TRUE(fs::is_regular_file(image_path));
  fs::permissions(sub_dir, fs::perms::all, fs::perm_options::remove);
  tester.CleanupFrame(scene_path, image_path);
  fs::permissions(sub_dir, orig_perms, fs::perm_options::replace);
  EXPECT_FALSE(fs::is_regular_file(scene_path));
  EXPECT_TRUE(fs::is_regular_file(image_path));

  // Cleanup for next iteration.
  fs::remove_all(temp_dir);
}

}  // namespace
}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
