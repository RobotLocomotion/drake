#include "drake/geometry/render_gltf_client/internal_render_engine_gltf_client.h"

#include <filesystem>
#include <fstream>
#include <set>
#include <vector>

#include <gtest/gtest.h>
#include <vtkCamera.h>
#include <vtkMatrix4x4.h>

#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/geometry/render_gltf_client/internal_http_service.h"
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

using geometry::render::ColorRenderCamera;
using geometry::render::DepthRenderCamera;
using geometry::render::RenderEngine;
using geometry::render::internal::ImageType;
using math::RigidTransformd;
using math::RollPitchYawd;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

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
  // Make sure that default values are passed to the underlying client.
  const RenderEngineGltfClient default_engine{Params{}};
  EXPECT_EQ(default_engine.get_params().GetUrl(),
            "http://127.0.0.1:8000/render");
  EXPECT_EQ(default_engine.get_params().verbose, false);
  EXPECT_EQ(default_engine.get_params().cleanup, true);

  // Make sure that alternative values are passed to the underlying client.
  const RenderEngineGltfClient engine{params_};
  EXPECT_EQ(engine.get_params().GetUrl(),
            "http://notarealserver:8192/no_render");
  EXPECT_EQ(engine.get_params().verbose, true);
  EXPECT_EQ(engine.get_params().cleanup, true);
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
  auto transform_inverse = [](const Eigen::Matrix4d& mat) {
    Matrix4d inverse{mat};
    // Invert the rotation via transpose.
    inverse.topLeftCorner<3, 3>().transposeInPlace();
    // Rotate the inverted translation.
    const Vector3d t = inverse.topRightCorner<3, 1>();
    const Vector3d t_inv = inverse.topLeftCorner<3, 3>() * (-t);
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
    const Matrix4d vtk_inv = transform_inverse(vtk);
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

    // Make sure the image got loaded as expected.
    EXPECT_EQ(label_image, CreateTestLabelImage());
  }
}

}  // namespace
}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
