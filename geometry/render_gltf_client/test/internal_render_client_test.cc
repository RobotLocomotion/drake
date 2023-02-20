#include "drake/geometry/render_gltf_client/internal_render_client.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <fmt/ostream.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/fmt_eigen.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/render_gltf_client/internal_http_service.h"
#include "drake/geometry/render_gltf_client/test/internal_sample_image_data.h"

namespace drake {
namespace systems {
namespace sensors {
// Add support for printing EXPECT_EQ(Image, Image) failures.
template <PixelType kPixelType>
void PrintTo(const Image<kPixelType>& image, std::ostream* os) {
  using T = typename Image<kPixelType>::T;
  using Promoted = std::conditional_t<std::is_integral_v<T>, int, T>;
  constexpr int num_channels = Image<kPixelType>::kNumChannels;
  const int width = image.width();
  const int height = image.height();
  *os << "\n";
  for (int z = 0; z < num_channels; ++z) {
    const T* const base = image.at(0, 0) + z;
    using Stride = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;
    Eigen::Map<const MatrixX<T>, 0, Stride> eigen(
        base, height, width, Stride(num_channels, width * num_channels));
    fmt::print(*os, "Channel {}:\n", z);
    fmt::print(*os, "{}\n", fmt_eigen(eigen.template cast<Promoted>()));
  }
}
}  // namespace sensors
}  // namespace systems
namespace geometry {
namespace render_gltf_client {
namespace internal {

namespace fs = std::filesystem;

using Params = RenderEngineGltfClientParams;
using geometry::render::ClippingRange;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRange;
using geometry::render::DepthRenderCamera;
using geometry::render::RenderCameraCore;
using systems::sensors::CameraInfo;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

namespace {

// Constexpr dimensions for the actual testing images.
constexpr int kTestImageWidth = 3;
constexpr int kTestImageHeight = 2;

// The paths for the testing images used across different unit tests.
const auto kTestRgbImagePath = FindResourceOrThrow(
    "drake/geometry/render_gltf_client/test/test_rgb_8U.png");
const auto kTestRgbaImagePath = FindResourceOrThrow(
    "drake/geometry/render_gltf_client/test/test_rgba_8U.png");
const auto kTestDepthImage32FPath = FindResourceOrThrow(
    "drake/geometry/render_gltf_client/test/test_depth_32F.tiff");
const auto kTestDepthImage16UTiffPath = FindResourceOrThrow(
    "drake/geometry/render_gltf_client/test/test_depth_16U.tiff");
const auto kTestDepthImage16UPngPath = FindResourceOrThrow(
    "drake/geometry/render_gltf_client/test/test_depth_16U.png");
const auto kTestColoredLabelImagePath = FindResourceOrThrow(
    "drake/geometry/render_gltf_client/test/test_colored_label_rgba_8U.png");

class RenderClientTest : public ::testing::Test {
 public:
  RenderClientTest()
      : color_camera_{{"proxy_render", {798, 247, M_PI_4}, {0.11, 111.111}, {}},
                      false},
        depth_camera_{color_camera_.core(), {0.12, 21.12}} {
    Touch(fake_scene_path_);
  }

  // Creates the given filename (and returns the filename for convenience).
  std::string Touch(const std::string& filename) {
    std::ofstream stream{filename};
    stream << "## RenderClientTest sample file." << "\n";
    DRAKE_DEMAND(stream.good());
    return filename;
  }

  std::string sha256() {
    // This value is from: echo '## RenderClientTest sample file.' | sha256sum
    return "ebf937a31924de40bef25563110837a2608a3e832f1449145e8f5b3b148c1f3b";
  }

 protected:
  // A per-test-case temporary directory.
  const fs::path scratch_{drake::temp_directory()};
  const std::string fake_scene_path_{scratch_ / "fake_scene.gltf"};

  /* The params to create the test RenderClient are to help ensure nothing
   actually gets sent over curl.  No test should proceed with the default
   HttpServiceCurl, the HttpService backend should be changed using
   client.SetHttpService before doing anything. */
  const Params params_{.base_url = "notarealserver:8192",
                       .render_endpoint = "no_render"};

  /* Create some render camera instances to validate against.  Note that the
   atypical values chosen for e.g., clipping or depth ranges is to ensure there
   are no hard-coded or default values leaking in anywhere. */
  const ColorRenderCamera color_camera_;
  const DepthRenderCamera depth_camera_;
};

// Constructor / destructor ----------------------------------------------------
TEST_F(RenderClientTest, Constructor) {
  // Verify default attributes are set properly given a default `Params`.
  const RenderClient default_client{Params{}};
  EXPECT_EQ(
      default_client.get_params().GetUrl(), "http://127.0.0.1:8000/render");
  EXPECT_EQ(default_client.get_params().verbose, false);
  EXPECT_EQ(default_client.get_params().cleanup, true);

  // Verify attributes are set properly given a specific `Params`.
  const std::string base_url{"http://127.0.0.1:1234"};
  const std::string render_endpoint{"testing"};
  const bool verbose = true;
  const bool cleanup = false;
  const RenderClient client{
      Params{base_url, render_endpoint, std::nullopt, verbose, cleanup}};
  EXPECT_EQ(client.get_params().GetUrl(), base_url + "/" + render_endpoint);
  EXPECT_EQ(client.get_params().verbose, verbose);
  EXPECT_EQ(client.get_params().cleanup, cleanup);
}

TEST_F(RenderClientTest, Destructor) {
  for (const bool cleanup : {true, false}) {
    std::string temp_dir_path;
    {
      const RenderClient client{Params{.cleanup = cleanup}};
      temp_dir_path = client.temp_directory();
      EXPECT_TRUE(fs::is_directory(temp_dir_path));
    }  // Client is deleted.

    // Test whether temp_directory is handled properly.
    if (cleanup) {
      EXPECT_FALSE(fs::is_directory(temp_dir_path));
    } else {
      EXPECT_TRUE(fs::is_directory(temp_dir_path));
    }
  }
}

// RenderOnServer --------------------------------------------------------------

using DoPostFormCallback = typename std::function<HttpResponse(
    const DataFieldsMap&, const FileFieldsMap&)>;

// TODO(zachfang): consider replacing this with GMock.
/* A proxy HttpService that can be constructed with an std::function to directly
 modify the behavior of DoPostForm(). The following tests fake different server
 responses, such as http_code and data_path, and examine the corresponding
 behavior of RenderClient::RenderOnServer(). */
class ProxyService : public HttpService {
 public:
  explicit ProxyService(const DoPostFormCallback& callback)
      : do_post_form_callback_{callback} {}

  HttpResponse DoPostForm(const std::string& /* temp_directory */,
                          const std::string& /* url */,
                          const DataFieldsMap& data_fields,
                          const FileFieldsMap& file_fields,
                          bool /* verbose */) override {
    return do_post_form_callback_(data_fields, file_fields);
  }

  DoPostFormCallback do_post_form_callback_;
};

/* These tests are only for the various error conditions that can come up in
 RenderOnServer.  They are tested linearly start to finish following the
 implementation.  There are additional tests at the end to ensure that valid
 images returned from the server (PNG and TIFF) are accepted, however these
 tests do *NOT* validate against image dimensions / content.  Image content
 tests take place in Load{Color,Depth,Label}Image tests, these tests are just
 for "round-trip" communications with the HttpService. */

/* Verifies DepthRange is specified whenever appropriate. */
TEST_F(RenderClientTest, RenderOnServerDepthRange) {
  RenderClient client{params_};

  /* A null callback (which will never be called). */
  DoPostFormCallback callback;
  client.SetHttpService(std::make_unique<ProxyService>(callback));

  // Forgetting to include the depth_range for a depth render should raise.
  DRAKE_EXPECT_THROWS_MESSAGE(
      client.RenderOnServer(depth_camera_.core(),
                            RenderImageType::kDepthDepth32F, fake_scene_path_),
      ".*is_depth_type.*");
  // Providing depth_range for non-depth (color or label) should raise.
  DRAKE_EXPECT_THROWS_MESSAGE(
      client.RenderOnServer(color_camera_.core(), RenderImageType::kColorRgba8U,
                            fake_scene_path_, std::nullopt,
                            depth_camera_.depth_range()),
      ".*is_depth_type.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      client.RenderOnServer(color_camera_.core(), RenderImageType::kLabel16I,
                            fake_scene_path_, std::nullopt,
                            depth_camera_.depth_range()),
      ".*is_depth_type.*");
}

/* Verifies that the RenderClient::RenderOnServer adheres to the API contract by
 populating the correct data and file fields.  Tests in this section use
 DRAKE_EXPECT_THROWS_MESSAGE all looking for the same error being raised, as no
 image response is provided. */
TEST_F(RenderClientTest, RenderOnServerFieldsCheck) {
  RenderClient client{params_};

  // Placeholders for different values when calling RenderOnServer().
  RenderImageType image_type{};
  std::optional<std::string> mime_type{std::nullopt};
  const DepthRange depth_range = depth_camera_.depth_range();

  /* This callback validates all of the expected fields.  It also implicitly
   validates that every expected key has actually been provided since the test
   will fail on directly accessing a key that does not exist. */
  DoPostFormCallback callback = [&](const DataFieldsMap& data_fields,
                                    const FileFieldsMap& file_fields) {
    switch (image_type) {
      case RenderImageType::kColorRgba8U: {
        EXPECT_EQ(data_fields.at("image_type"), "color");
        break;
      }
      case RenderImageType::kDepthDepth32F: {
        EXPECT_EQ(data_fields.at("image_type"), "depth");
        break;
      }
      case RenderImageType::kLabel16I: {
        EXPECT_EQ(data_fields.at("image_type"), "label");
        break;
      }
    }
    EXPECT_EQ(data_fields.at("scene_sha256"), sha256());
    const CameraInfo& intrinsics = color_camera_.core().intrinsics();
    EXPECT_EQ(data_fields.at("width"), std::to_string(intrinsics.width()));
    EXPECT_EQ(data_fields.at("height"), std::to_string(intrinsics.height()));
    const ClippingRange& clipping = color_camera_.core().clipping();
    EXPECT_EQ(data_fields.at("near"), std::to_string(clipping.near()));
    EXPECT_EQ(data_fields.at("far"), std::to_string(clipping.far()));
    EXPECT_EQ(data_fields.at("focal_x"), std::to_string(intrinsics.focal_x()));
    EXPECT_EQ(data_fields.at("focal_y"), std::to_string(intrinsics.focal_y()));
    EXPECT_EQ(data_fields.at("fov_x"), std::to_string(intrinsics.fov_x()));
    EXPECT_EQ(data_fields.at("fov_y"), std::to_string(intrinsics.fov_y()));
    EXPECT_EQ(data_fields.at("center_x"),
              std::to_string(intrinsics.center_x()));
    EXPECT_EQ(data_fields.at("center_y"),
              std::to_string(intrinsics.center_y()));
    // min_depth and max_depth should only be provided for depth renders.
    if (image_type == RenderImageType::kDepthDepth32F) {
      EXPECT_EQ(data_fields.at("min_depth"),
                std::to_string(depth_range.min_depth()));
      EXPECT_EQ(data_fields.at("max_depth"),
                std::to_string(depth_range.max_depth()));
    } else {
      EXPECT_EQ(data_fields.find("min_depth"), data_fields.end());
      EXPECT_EQ(data_fields.find("max_depth"), data_fields.end());
    }
    EXPECT_EQ(data_fields.at("submit"), "Render");

    // Make sure no extra fields are being added to data_fields.
    std::set<std::string> data_fields_keys{
        "scene_sha256", "image_type", "width",   "height", "near",
        "far",          "focal_x",    "focal_y", "fov_x",  "fov_y",
        "center_x",     "center_y",   "submit"};
    if (image_type == RenderImageType::kDepthDepth32F) {
      data_fields_keys.insert("min_depth");
      data_fields_keys.insert("max_depth");
    }

    std::set<std::string> actual_data_fields_keys;
    for (const auto& pair : data_fields) {
      actual_data_fields_keys.insert(pair.first);
    }
    EXPECT_EQ(data_fields_keys, actual_data_fields_keys);

    // Make sure the scene file path has been provided with the expected mime.
    const auto& [path, mime] = file_fields.at("scene");
    EXPECT_EQ(path, fake_scene_path_);
    EXPECT_EQ(mime, mime_type);

    // Only one file should be getting added.
    EXPECT_EQ(file_fields.size(), 1);

    // Fail the test so that no attempted image loading occurs.
    return HttpResponse{.http_code = 500,
                        .service_error_message = "Always fails."};
  };
  client.SetHttpService(std::make_unique<ProxyService>(callback));

  // Check fields for a color render.
  image_type = RenderImageType::kColorRgba8U;
  DRAKE_EXPECT_THROWS_MESSAGE(
      client.RenderOnServer(color_camera_.core(), image_type, fake_scene_path_),
      "[\\s\\S]*ERROR doing POST:[\\s\\S]*");

  // Check fields for a label render.
  image_type = RenderImageType::kLabel16I;
  DRAKE_EXPECT_THROWS_MESSAGE(
      client.RenderOnServer(color_camera_.core(), image_type, fake_scene_path_),
      "[\\s\\S]*ERROR doing POST:[\\s\\S]*");

  /* Check fields for a depth render.  This test also includes a verification
   that the provided mime_type is propagated correctly.  There is no special
   reason for this to be checked with the depth render. */
  image_type = RenderImageType::kDepthDepth32F;
  mime_type = "test/mime_type";
  DRAKE_EXPECT_THROWS_MESSAGE(
      client.RenderOnServer(depth_camera_.core(), image_type, fake_scene_path_,
                            mime_type, depth_range),
      "[\\s\\S]*ERROR doing POST:[\\s\\S]*");
}

/* Tests bad `HttpResponse`s that have a server message provided. Edge cases
 should all produce 'None.' as the server message.
 NOTE: File extensions do not matter and all tests must use http code 400. */
TEST_F(RenderClientTest, RenderOnServerMessageFromServer) {
  RenderClient client{params_};
  DoPostFormCallback callback;

  // Case 1: edge case, service populated data_path but file is length 0.
  callback = [&](const DataFieldsMap&, const FileFieldsMap&) {
    const std::string response_path = scratch_ / "zero_length.response";
    std::ofstream response{response_path};
    return HttpResponse{.http_code = 400, .data_path = response_path};
  };
  client.SetHttpService(std::make_unique<ProxyService>(callback));
  DRAKE_EXPECT_THROWS_MESSAGE(
      client.RenderOnServer(color_camera_.core(), RenderImageType::kColorRgba8U,
                            fake_scene_path_),
      "[\\s\\S]*Server Message:[\\s\\S]*None.[\\s\\S]*");

  // Case 2: edge case, bad response but provided message "too long".
  callback = [&](const DataFieldsMap&, const FileFieldsMap&) {
    const std::string response_path = scratch_ / "too_long.response";
    std::ofstream response{response_path};
    // NOTE: this value is hard-coded in RenderClient::RenderOnServer.
    for (int i = 0; i < 8192; ++i) response << '0';
    return HttpResponse{.http_code = 400, .data_path = response_path};
  };
  client.SetHttpService(std::make_unique<ProxyService>(callback));
  DRAKE_EXPECT_THROWS_MESSAGE(
      client.RenderOnServer(color_camera_.core(), RenderImageType::kLabel16I,
                            fake_scene_path_),
      "[\\s\\S]*Server Message:[\\s\\S]*None.[\\s\\S]*");

  // Case 3: server response that can be read.
  callback = [&](const DataFieldsMap&, const FileFieldsMap&) {
    const std::string response_path = scratch_ / "can_read.response";
    Touch(response_path);
    return HttpResponse{.http_code = 400, .data_path = response_path};
  };
  client.SetHttpService(std::make_unique<ProxyService>(callback));
  DRAKE_EXPECT_THROWS_MESSAGE(
      client.RenderOnServer(color_camera_.core(), RenderImageType::kLabel16I,
                            fake_scene_path_),
      "[\\s\\S]*RenderClientTest sample file.[\\s\\S]*");
}

TEST_F(RenderClientTest, RenderOnServerNoFileReturn) {
  RenderClient client{params_};

  // No file response from server should be reported correctly.
  DoPostFormCallback callback = [&](const DataFieldsMap&,
                                    const FileFieldsMap&) {
    return HttpResponse{.http_code = 200};
  };
  client.SetHttpService(std::make_unique<ProxyService>(callback));
  DRAKE_EXPECT_THROWS_MESSAGE(
      client.RenderOnServer(color_camera_.core(), RenderImageType::kColorRgba8U,
                            fake_scene_path_),
      "ERROR doing POST.*supposed to respond with a file but did not.");
}

TEST_F(RenderClientTest, RenderOnServerInvalidImageReturn) {
  RenderClient client{params_};

  // File response cannot be loaded as image should be reported correctly.
  DoPostFormCallback callback = [&](const DataFieldsMap&,
                                    const FileFieldsMap&) {
    const std::string response_path = scratch_ / "not_an_image.response";
    Touch(response_path);
    return HttpResponse{.http_code = 200, .data_path = response_path};
  };
  client.SetHttpService(std::make_unique<ProxyService>(callback));
  DRAKE_EXPECT_THROWS_MESSAGE(
      client.RenderOnServer(color_camera_.core(), RenderImageType::kColorRgba8U,
                            fake_scene_path_),
      ".*is not understood as an image type that is supported.*");
}

TEST_F(RenderClientTest, RenderOnServerValidImageReturn) {
  RenderClient client{params_};
  DoPostFormCallback callback;

  /* Return a PNG or TIFF file from the server and check that it is renamed.
   Note that the dimensions (width, height) of this image to not match the
   configured camera, but that doesn't matter to RenderOnServe. It only checks
   for a well-formed image file, not the details of the image. */
  callback = [&](const DataFieldsMap&, const FileFieldsMap&) {
    const std::string response_path = scratch_ / "valid_png.response";
    fs::copy_file(kTestRgbaImagePath, response_path);
    return HttpResponse{.http_code = 200, .data_path = response_path};
  };
  client.SetHttpService(std::make_unique<ProxyService>(callback));
  const std::string expected_png_path =
      fs::path(fake_scene_path_).replace_extension(".png");
  const std::string response_png = client.RenderOnServer(
      color_camera_.core(), RenderImageType::kColorRgba8U, fake_scene_path_);
  EXPECT_EQ(response_png, expected_png_path);

  callback = [&](const DataFieldsMap&, const FileFieldsMap&) {
    const std::string response_path = scratch_ / "valid_tiff.response";
    fs::copy_file(kTestDepthImage32FPath, response_path);
    return HttpResponse{.http_code = 200, .data_path = response_path};
  };
  client.SetHttpService(std::make_unique<ProxyService>(callback));
  const std::string expected_tiff_path =
      fs::path(fake_scene_path_).replace_extension(".tiff");
  const std::string response_tiff = client.RenderOnServer(
      depth_camera_.core(), RenderImageType::kDepthDepth32F, fake_scene_path_,
      "test/mime_type", depth_camera_.depth_range());
  EXPECT_EQ(response_tiff, expected_tiff_path);
}

TEST_F(RenderClientTest, ComputeSha256Good) {
  // To obtain this magic number, use a bash command:
  //   sha256sum geometry/render_gltf_client/test/test_depth_32F.tiff
  EXPECT_EQ(
      RenderClient::ComputeSha256(kTestDepthImage32FPath),
      "6bb5621f3cdf06bb43c7104eb9a2dc5ab85db79b2c491be69c7704d03c476c1b");
}

TEST_F(RenderClientTest, ComputeSha256Bad) {
  // Failure case: provided input file does not exist.
  DRAKE_EXPECT_THROWS_MESSAGE(
        RenderClient::ComputeSha256("/no/such/file"),
        ".*cannot open.*/no/such/file.*");
}

TEST_F(RenderClientTest, RenameHttpServiceResponseGood) {
  /* A testing tuple for RenameHttpService inputs and expected output. */
  struct RenameResult {
    // Input arguments.
    std::string response_data_path;
    std::string reference_path;
    std::string extension;
    // Expected return value.
    std::string expected;
  };

  /* NOTE: Files are created, not deleted, and do not reuse names.
   Additionally, do not test for no extension input_scene + ext="", this is
   already tested for above as an expected exception. */
  const fs::path base = scratch_;
  const fs::path sub = base/"sub";
  fs::create_directory(sub);
  const std::vector<RenameResult> renames{
      // All components have extensions.
      {base/"path_0.bin",  base/"input_0.gltf", ".png",  base/"input_0.png"},
      // Input scene does not have a file extension.
      {base/"path_1.curl", base/"input_1",      ".foo",  base/"input_1.foo"},
      // Input path does not have an extension.
      {base/"path_2",      base/"input_2.gltf", ".tiff", base/"input_2.tiff"},
      // Input and path do not have an extension.
      {base/"path_3",      base/"input_3",      ".zip",  base/"input_3.zip"},
      // Empty extension is allowed.
      {base/"path_4.txt",  base/"input_4.txt",  "",      base/"input_4"},
      // Source is in different directory.
      {sub/"path_5.bin",   base/"input_5.gltf", ".jpg",  base/"input_5.jpg"}
  };

  for (const auto& r : renames) {
    Touch(r.response_data_path);
    // Before renaming, expected should not exist.
    EXPECT_FALSE(fs::is_regular_file(r.expected));

    // Renaming the file should return the expected value.
    const std::string result = RenderClient::RenameHttpServiceResponse(
        r.response_data_path, r.reference_path, r.extension);
    EXPECT_EQ(result, r.expected);

    // Check that the renaming actually happened.
    EXPECT_FALSE(fs::is_regular_file(r.response_data_path));
    EXPECT_TRUE(fs::is_regular_file(r.expected));
  }
}

TEST_F(RenderClientTest, RenameHttpServiceResponseBad) {
  // Failure case 1: file to rename does not exist.
  DRAKE_EXPECT_THROWS_MESSAGE(RenderClient::RenameHttpServiceResponse(
                                  "/no/such/file", fake_scene_path_, ".png"),
                              ".*[Nn]o such file.*/no/such/file.*");

  // Failure case 2: destination file already exists.
  DRAKE_EXPECT_THROWS_MESSAGE(RenderClient::RenameHttpServiceResponse(
                                  fake_scene_path_, fake_scene_path_, ".gltf"),
                              ".*refusing to rename.*file already exists.*");
}

TEST_F(RenderClientTest, LoadColorImageGood) {
  // Loading a three channel (RGB) PNG file should work as expected.
  ImageRgba8U rgb(kTestImageWidth, kTestImageHeight, 0);
  RenderClient::LoadColorImage(kTestRgbImagePath, &rgb);
  EXPECT_EQ(rgb, CreateTestColorImage(true));

  // Loading a four channel (RGBA) PNG file should work as expected.
  ImageRgba8U rgba(kTestImageWidth, kTestImageHeight, 0);
  RenderClient::LoadColorImage(kTestRgbaImagePath, &rgba);
  EXPECT_EQ(rgba, CreateTestColorImage(false));
}

TEST_F(RenderClientTest, LoadColorImageBad) {
  ImageRgba8U ignored(kTestImageWidth, kTestImageHeight, 0);

  // Failure case 1: no such file.
  DRAKE_EXPECT_THROWS_MESSAGE(
      RenderClient::LoadColorImage("/no/such/file", &ignored),
      ".*cannot load.*/no/such/file.*");

  // Failure case 2: not a valid image file.
  DRAKE_EXPECT_THROWS_MESSAGE(
      RenderClient::LoadColorImage(Touch(scratch_/"fake.png"), &ignored),
      ".*cannot load.*fake.*");

  // Failure case 3: wrong image dimensions (on each axis).
  for (const bool selector : {true, false}) {
    int width = kTestImageWidth;
    int height = kTestImageHeight;
    if (selector) { ++width; } else { ++height; }
    ImageRgba8U wrong_size(width, height, 0);
    DRAKE_EXPECT_THROWS_MESSAGE(
        RenderClient::LoadColorImage(kTestRgbaImagePath, &wrong_size),
        ".*expected.*but got.*width=.*height=.*");
  }

  // Failure case 4: wrong number of channels (== 1) instead of 3 or 4.
  DRAKE_EXPECT_THROWS_MESSAGE(
      RenderClient::LoadColorImage(kTestDepthImage16UPngPath, &ignored),
      ".*PNG image.*has 1 channel.*");
}

TEST_F(RenderClientTest, LoadDepth32FGood) {
  // Loading a single channel 32-bit TIFF file should work as expected.
  ImageDepth32F depth(kTestImageWidth, kTestImageHeight, 0);
  RenderClient::LoadDepthImage(kTestDepthImage32FPath, &depth);
  EXPECT_EQ(depth, CreateTestDepthImage());
}

TEST_F(RenderClientTest, LoadDepth16UTiffGood) {
  // Loading a single channel 16-bit TIFF file should work as expected.
  ImageDepth32F depth(kTestImageWidth, kTestImageHeight, 0);
  RenderClient::LoadDepthImage(kTestDepthImage16UTiffPath, &depth);
  EXPECT_EQ(depth, CreateTestDepthImage());
}

TEST_F(RenderClientTest, LoadDepth16UPngGood) {
  // Loading a single channel 16-bit PNG file should work as expected.
  ImageDepth32F depth(kTestImageWidth, kTestImageHeight, 0);
  RenderClient::LoadDepthImage(kTestDepthImage16UPngPath, &depth);
  EXPECT_EQ(depth, CreateTestDepthImage());
}

TEST_F(RenderClientTest, LoadDepthImageBad) {
  ImageDepth32F ignored(kTestImageWidth, kTestImageHeight, 0);

  // Failure case 1: invalid extension.
  DRAKE_EXPECT_THROWS_MESSAGE(
      RenderClient::LoadDepthImage("/no/such/file_ext.foo", &ignored),
      "Unsupported file extension");

  // Failure case 2: not a valid image file.
  DRAKE_EXPECT_THROWS_MESSAGE(
      RenderClient::LoadDepthImage(Touch(scratch_/"fake.tiff"), &ignored),
      ".*cannot load.*fake.*");

  // Failure case 3: wrong image dimensions (on each axis).
  for (const bool selector : {true, false}) {
    int width = kTestImageWidth;
    int height = kTestImageHeight;
    if (selector) { ++width; } else { ++height; }
    ImageDepth32F wrong_size(width, height, 0);
    DRAKE_EXPECT_THROWS_MESSAGE(
        RenderClient::LoadDepthImage(kTestDepthImage32FPath, &wrong_size),
        ".*expected.*but got.*width=.*height=.*");
  }
}

}  // namespace
}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
