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
