#include "drake/geometry/render_gltf_client/internal_render_client.h"

#include <cstdio>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/filesystem.h"
#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/render_gltf_client/internal_http_service.h"
#include "drake/geometry/render_gltf_client/test/internal_sample_image_data.h"

namespace drake {
namespace geometry {
namespace render_gltf_client {
namespace internal {

namespace fs = drake::filesystem;

using Params = RenderEngineGltfClientParams;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRange;
using geometry::render::DepthRenderCamera;
using geometry::render::RenderCameraCore;
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
const auto kTestDepthImagePath = FindResourceOrThrow(
    "drake/geometry/render_gltf_client/test/test_depth_32F.tiff");
const auto kTestLabelImagePath = FindResourceOrThrow(
    "drake/geometry/render_gltf_client/test/test_label_16I.png");

class RenderClientTest : public ::testing::Test {
 public:
  RenderClientTest() {}

  // Creates the given filename (and returns the filename for convenience).
  std::string Touch(const std::string& filename) {
    std::ofstream stream{filename};
    stream << "## RenderClientTest sample file " << filename << "\n";
    DRAKE_DEMAND(stream.good());
    return filename;
  }

 protected:
  // A per-test-case temporary directory.
  const fs::path scratch_{drake::temp_directory()};
};

// Constructor / destructor ----------------------------------------------------
TEST_F(RenderClientTest, Constructor) {
  const std::string base_url{"127.0.0.1:8000"};
  const std::string render_endpoint{"render"};
  const bool verbose = false;

  // Verify attributes after valid construction.
  auto make_client_and_verify = [&](bool p_no_cleanup) {
    std::string temp_directory;
    {
      RenderClient client{Params{base_url, render_endpoint, std::nullopt,
                                 verbose, p_no_cleanup}};
      temp_directory = client.temp_directory();
      EXPECT_EQ(client.get_params().GetUrl(), "127.0.0.1:8000/render");
      EXPECT_EQ(client.get_params().verbose, verbose);
      EXPECT_EQ(client.get_params().no_cleanup, p_no_cleanup);
      EXPECT_TRUE(fs::is_directory(client.temp_directory()));
    }  // Client is deleted.
    if (p_no_cleanup) {
      EXPECT_TRUE(fs::is_directory(temp_directory));
    } else {
      EXPECT_FALSE(fs::is_directory(temp_directory));
    }
  };
  make_client_and_verify(true);
  make_client_and_verify(false);
}

TEST_F(RenderClientTest, Destructor) {
  const std::string base_url{"127.0.0.1:8000"};
  const std::string render_endpoint = "render";
  const bool verbose = false;

  std::string temp_dir_path;
  // Construction with no_cleanup=false: temp_directory should be gone.
  {
    const RenderClient client{
        Params{base_url, render_endpoint, std::nullopt, verbose, false}};
    temp_dir_path = client.temp_directory();
    EXPECT_TRUE(fs::is_directory(temp_dir_path));
  }  // Client is deleted.
  EXPECT_FALSE(fs::is_directory(temp_dir_path));

  // Construction with no_cleanup=true: temp_directory should remain.
  {
    const RenderClient client{
        Params{base_url, render_endpoint, std::nullopt, verbose, true}};
    temp_dir_path = client.temp_directory();
    EXPECT_TRUE(fs::is_directory(temp_dir_path));
  }  // Client is deleted.
  EXPECT_TRUE(fs::is_directory(temp_dir_path));
  fs::remove(temp_dir_path);
}

// RenderOnServer --------------------------------------------------------------
// A simple HttpService that always fails.
class FailService : public HttpService {
 public:
  FailService() = default;

  HttpResponse DoPostForm(const std::string& /* temp_directory */,
                          const std::string& /* url */,
                          const DataFieldsMap& /* data_fields */,
                          const FileFieldsMap& /* file_fields */,
                          bool /* verbose */ = false) override {
    HttpResponse ret;
    ret.http_code = 500;
    ret.service_error_message = "FailService always fails.";
    return ret;
  }
};

/* Verifies the contract fullfilled by RenderClient::RenderOnServer, checking
 that all fields are exactly as expected (and where min_depth / max_depth are
 concerned, they are included / excluded as expected).  The code checks in
 DoPostForm are more or less a duplication of RenderClient::RenderOnServer,
 meaning any changes to the code populating the <form> will break this test case
 (intentionally).

 A test using a FieldCheckService will construct with the exact same parameters
 that RenderClient::RenderOnServer is going to use, so that when PostForm is
 called behind the scenes we have all the information to cross-check against. */
class FieldCheckService : public HttpService {
 public:
  /* All parameters for HttpService, followed by RenderClient::RenderOnServer
   with the addition of the sha256. */
  FieldCheckService(const RenderCameraCore& camera_core,
                    RenderImageType image_type, const std::string& scene_path,
                    const std::string& scene_sha256,
                    const std::optional<std::string>& mime_type = std::nullopt,
                    const std::optional<DepthRange>& depth_range = std::nullopt)
      : HttpService(),
        camera_core_{camera_core},
        image_type_{image_type},
        scene_path_{scene_path},
        scene_sha256_{scene_sha256},
        mime_type_{mime_type},
        depth_range_{depth_range} {}

  // Checks all of the <form> fields and always responds with a failure HTTP
  // response code (500).
  HttpResponse DoPostForm(const std::string& /* temp_directory */,
                          const std::string& /* url */,
                          const DataFieldsMap& data_fields,
                          const FileFieldsMap& file_fields,
                          bool /* verbose */ = false) override {
    /* Validate all of the expected fields.  This also implicitly validates that
     every expected key has actually been provided since the test will fail on
     directly accessing a key that does not exist. */
    EXPECT_EQ(data_fields.at("scene_sha256"), scene_sha256_);
    if (image_type_ == RenderImageType::kColorRgba8U) {
      EXPECT_EQ(data_fields.at("image_type"), "color");
    } else if (image_type_ == RenderImageType::kDepthDepth32F) {
      EXPECT_EQ(data_fields.at("image_type"), "depth");
    } else {  // image_type_ := RenderImageType::kLabel16I
      EXPECT_EQ(data_fields.at("image_type"), "label");
    }
    const auto& intrinsics = camera_core_.intrinsics();
    EXPECT_EQ(data_fields.at("width"), std::to_string(intrinsics.width()));
    EXPECT_EQ(data_fields.at("height"), std::to_string(intrinsics.height()));
    const auto& clipping = camera_core_.clipping();
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
    if (image_type_ == RenderImageType::kDepthDepth32F) {
      const auto& range = depth_range_.value();
      EXPECT_EQ(data_fields.at("min_depth"), std::to_string(range.min_depth()));
      EXPECT_EQ(data_fields.at("max_depth"), std::to_string(range.max_depth()));
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
    if (image_type_ == RenderImageType::kDepthDepth32F) {
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
    EXPECT_EQ(path, scene_path_);
    EXPECT_EQ(mime, mime_type_);

    // Only one file should be getting added.
    EXPECT_EQ(file_fields.size(), 1);

    // Fail the test so that no attempted image loading occurs.
    HttpResponse ret;
    ret.http_code = 500;
    return ret;
  }

  const RenderCameraCore& camera_core_;
  /* NOTE: do not store references for the data fields, EXPECT_EQ may not work
   correctly for value comparisons. */
  const RenderImageType image_type_;
  const std::string scene_path_;
  const std::string scene_sha256_;
  const std::optional<std::string> mime_type_;
  const std::optional<DepthRange> depth_range_;
};

using PostFormCallback = typename std::function<HttpResponse()>;

/* A proxy HttpService that can be constructed with an std::function to directly
 modify the behavior of DoPostForm(). The following tests fake different server
 responses, such as http_code and data_path, and examine the corresponding
 behavior of RenderClient::RenderOnServer(). */
class ProxyService : public HttpService {
 public:
  explicit ProxyService(const PostFormCallback& callback)
      : HttpService(), post_form_callback_{callback} {}

  HttpResponse DoPostForm(const std::string& /* temp_directory */,
                          const std::string& /* url */,
                          const DataFieldsMap& /* data_fields */,
                          const FileFieldsMap& /* file_fields */,
                          bool /* verbose */ = false) override {
    return post_form_callback_();
  }

  PostFormCallback post_form_callback_;
};

/* These tests are only for the various error conditions that can come up in
 RenderOnServer.  They are tested linearly start to finish following the
 implementation.  There are additional tests at the end to ensure that valid
 images returned from the server (png and tiff) are accepted, however these
 tests do *NOT* validate against image dimensions / content.  Image content
 tests take place in Load{Color,Depth,Label}Image tests, these tests are just
 for "round-trip" communications with the HttpService. */
TEST_F(RenderClientTest, RenderOnServer) {
  /* NOTE: these values help ensure nothing actually gets sent over curl.  No
   test should proceed with the default HttpServiceCurl, the HttpService backend
   should be changed using client.SetHttpService before doing anything. */
  const std::string base_url{"notarealserver:8192"};
  const std::string render_endpoint{"no_render"};
  const bool verbose{false};
  const bool no_cleanup{false};

  // Create a client and proxy HttpService creation helper.
  RenderClient client(
      Params{base_url, render_endpoint, std::nullopt, verbose, no_cleanup});
  // All the files generated under `temp_dir_path` will be purged in the end.
  const auto temp_dir_path = fs::path(client.temp_directory());
  const std::string url = client.get_params().GetUrl();

  // Create a fake scene to "upload" to the "server" and fake response file.
  const auto fake_scene_path = (temp_dir_path / "fake_scene.gltf").string();
  std::ofstream fake_scene{fake_scene_path};
  fake_scene << "This is not a real glTF scene!\n";
  fake_scene.close();
  // This value is from: echo 'This is not a real glTF scene!' | sha256sum
  const auto fake_scene_sha256 =
      "8985d0a8aec5a091dce05fef24ee2f3daaa20b4c0dbcd521f0217632c9001a4c";

  /* Create some render camera instances to validate against.  Note that the
   atypical values chosen for e.g., clipping or depth ranges is to ensure there
   are no hard-coded or default values leaking in anywhere. */
  const ColorRenderCamera color_camera{
      {"proxy_render", {798, 247, M_PI_4}, {0.11, 111.111}, {}}, false};
  const DepthRenderCamera depth_camera{color_camera.core(), {0.12, 21.12}};

  {
    // Forgetting to include the depth_range for a depth render should raise.
    client.SetHttpService(std::make_unique<FailService>());
    EXPECT_THROW(
        client.RenderOnServer(depth_camera.core(),
                              RenderImageType::kDepthDepth32F, fake_scene_path),
        std::runtime_error);

    // Providing depth_range for non-depth should raise.
    EXPECT_THROW(client.RenderOnServer(
                     color_camera.core(), RenderImageType::kColorRgba8U,
                     fake_scene_path, std::nullopt, depth_camera.depth_range()),
                 std::runtime_error);
    EXPECT_THROW(client.RenderOnServer(
                     color_camera.core(), RenderImageType::kLabel16I,
                     fake_scene_path, std::nullopt, depth_camera.depth_range()),
                 std::runtime_error);
  }

  {
    /* Verify that the RenderClient::RenderOnServer adheres to the API contract
     by populating the correct data and file fields.  Tests in this section
     use DRAKE_EXPECTS_THROWS_MESSAGE all looking for the same error being
     raised, as no image response is provided.  However, the bulk of the test
     takes place in the assertions in FieldCheckService::PostForm. */
    const auto expected_message = fmt::format(
        "\\s*ERROR doing POST:\\s*"        // ERROR doing POST:
        "URL:\\s*{}\\s*"                   // URL:              {url}
        "Service Message:\\s*None\\.\\s*"  // Service Message:  None.
        "HTTP Code:\\s*500\\s*"            // Http Code:        {code}
        "Server Message:\\s*None\\.\\s*",  // Server Message:   None.
        url);

    // Check fields for a color render.
    client.SetHttpService(std::make_unique<FieldCheckService>(
        color_camera.core(), RenderImageType::kColorRgba8U, fake_scene_path,
        fake_scene_sha256));
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(),
                              RenderImageType::kColorRgba8U, fake_scene_path),
        expected_message);

    /* Check fields for a depth render.  This test also includes a verification
     that the provided mime_type is propagated correctly.  There is no special
     reason for this to be checked with the depth render, it is just convenient
     since a new HttpService is being set for the client. */
    client.SetHttpService(std::make_unique<FieldCheckService>(
        depth_camera.core(), RenderImageType::kDepthDepth32F, fake_scene_path,
        fake_scene_sha256, "test/mime_type", depth_camera.depth_range()));
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(depth_camera.core(),
                              RenderImageType::kDepthDepth32F, fake_scene_path,
                              "test/mime_type", depth_camera.depth_range()),
        expected_message);

    // Check fields for a label render.
    client.SetHttpService(std::make_unique<FieldCheckService>(
        color_camera.core(), RenderImageType::kLabel16I, fake_scene_path,
        fake_scene_sha256, std::nullopt, std::nullopt));
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(), RenderImageType::kLabel16I,
                              fake_scene_path),
        expected_message);
  }

  {
    /* Test bad HttpResponse's that have a server message provided.  Previous
     tests validating the FieldCheckService exception have already validated the
     path that the HttpResponse.data_path is not populated.  Edge cases should
     all produce 'None.' as the server message.
     NOTE: file extensions do not matter. */
    // NOTE: all tests using this must use http code 400.
    const auto message_template = fmt::format(
        "\\s*ERROR doing POST:\\s*"        // ERROR doing POST:
        "URL:\\s*{}\\s*"                   // URL:              {url}
        "Service Message:\\s*None\\.\\s*"  // Service Message:  None.
        "HTTP Code:\\s*400\\s*"            // Http Code:        {code}
        "Server Message:\\s*{{}}\\s*",     // Server Message:   {message}
        url);

    PostFormCallback callback{nullptr};
    // Case 1: edge case, service populated data_path but file is length 0.
    callback = [&]() {
      const auto response_path =
          (temp_dir_path / "zero_length.response").string();
      std::ofstream response{response_path};
      response.close();
      return HttpResponse{400, response_path};
    };
    client.SetHttpService(std::make_unique<ProxyService>(callback));
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(),
                              RenderImageType::kColorRgba8U, fake_scene_path),
        fmt::format(message_template, "None\\."));

    // Case 2: edge case, bad response but provided message "too long".
    callback = [&]() {
      const auto response_path = (temp_dir_path / "too_long.response").string();
      std::ofstream response{response_path};
      // NOTE: this value is hard-coded in RenderClient::RenderOnServer.
      for (int i = 0; i < 8192; ++i) response << '0';
      response.close();
      return HttpResponse{400, response_path};
    };
    client.SetHttpService(std::make_unique<ProxyService>(callback));
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(), RenderImageType::kLabel16I,
                              fake_scene_path),
        fmt::format(message_template, "None\\."));

    // Case 3: edge case, message provided of valid length but cannot be opened.
    const auto bad_permission_path =
        (temp_dir_path / "bad_permission.response").string();
    callback = [&]() {
      std::ofstream response{bad_permission_path};
      response << "If only you could read me!\n";
      response.close();
      fs::permissions(bad_permission_path, fs::perms::all,
                      fs::perm_options::remove);
      return HttpResponse{400, bad_permission_path};
    };
    client.SetHttpService(std::make_unique<ProxyService>(callback));
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(),
                              RenderImageType::kColorRgba8U, fake_scene_path),
        fmt::format("RenderOnServer: cannot open file {}",
                    bad_permission_path));

    // Case 4: server response that can be read.
    const auto response_text = "You are not a valid request :p";
    callback = [&]() {
      const auto response_path = (temp_dir_path / "can_read.response").string();
      std::ofstream response{response_path};
      response << response_text;
      response.close();
      return HttpResponse{400, response_path};
    };
    client.SetHttpService(std::make_unique<ProxyService>(callback));
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(), RenderImageType::kLabel16I,
                              fake_scene_path),
        fmt::format(message_template, response_text));
  }

  {
    // No file response from server should be reported correctly.
    PostFormCallback callback = [&]() { return HttpResponse{200}; };
    client.SetHttpService(std::make_unique<ProxyService>(callback));
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(),
                              RenderImageType::kColorRgba8U, fake_scene_path),
        fmt::format(
            "ERROR doing POST to {}, HTTP code=200: the server was supposed to"
            " respond with a file but did not.",
            url));
  }

  {
    // File response provided that does not exist should be reported correctly.
    const auto response_path =
        (temp_dir_path / "does_not_exist.response").string();
    PostFormCallback callback = [&]() {
      return HttpResponse{200, response_path};
    };
    client.SetHttpService(std::make_unique<ProxyService>(callback));
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(), RenderImageType::kLabel16I,
                              fake_scene_path),
        fmt::format(
            "ERROR doing POST to {}, HTTP code=200: the service responded with "
            "a file path '{}' but the file does not exist.",
            url, response_path));
  }

  {
    const auto response_path =
        (temp_dir_path / "not_an_image.response").string();
    // File response cannot be loaded as image should be reported correctly.
    PostFormCallback callback = [&]() {
      std::ofstream response{response_path};
      response << "I am not an image file!\n";
      response.close();
      return HttpResponse{200, response_path};
    };
    client.SetHttpService(std::make_unique<ProxyService>(callback));
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(),
                              RenderImageType::kColorRgba8U, fake_scene_path),
        fmt::format(
            "RenderClient: while trying to render the scene '{}' with a sha256 "
            "hash of '{}', the file returned by the server saved in '{}' is "
            "not understood as an image type that is supported, i.e., PNG or "
            "TIFF.",
            fake_scene_path, fake_scene_sha256, response_path));
  }

  {
    // Copy a "valid" PNG file and check that it is renamed.
    PostFormCallback callback = [&]() {
      const auto response_path =
          (temp_dir_path / "valid_png.response").string();
      fs::copy_file(kTestRgbaImagePath, response_path);
      return HttpResponse{200, response_path};
    };
    client.SetHttpService(std::make_unique<ProxyService>(callback));
    const auto expected_path =
        fs::path(fake_scene_path).replace_extension(".png").string();
    const auto response_png = client.RenderOnServer(
        color_camera.core(), RenderImageType::kColorRgba8U, fake_scene_path);
    EXPECT_EQ(response_png, expected_path);
  }

  {
    /* Manufacture a "valid" (not the same width and height) TIFF file and check
     that it is renamed. */
    PostFormCallback callback = [&]() {
      const auto response_path =
          (temp_dir_path / "valid_but_wrong_dims.response").string();
      fs::copy_file(kTestDepthImagePath, response_path);
      return HttpResponse{200, response_path};
    };
    client.SetHttpService(std::make_unique<ProxyService>(callback));
    const auto expected_path =
        fs::path(fake_scene_path).replace_extension(".tiff").string();
    const auto response_tiff = client.RenderOnServer(
        depth_camera.core(), RenderImageType::kDepthDepth32F, fake_scene_path,
        "test/mime_type", depth_camera.depth_range());
    EXPECT_EQ(response_tiff, expected_path);
  }

  fs::remove_all(temp_dir_path);
}

TEST_F(RenderClientTest, ComputeSha256Good) {
  // To obtain this magic number, use a bash command:
  //   sha256sum geometry/render_gltf_client/test/test_depth_32F.tiff
  EXPECT_EQ(
      RenderClient::ComputeSha256(kTestDepthImagePath),
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

  /* NOTE: files are created, not deleted, do not reuse names.
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
  const std::string scene = scratch_ / "scene.gltf";
  Touch(scene);

  // Failure case 1: file to rename does not exist.
  DRAKE_EXPECT_THROWS_MESSAGE(
      RenderClient::RenameHttpServiceResponse("/no/such/file", scene, ".png"),
      ".*[Nn]o such file.*/no/such/file.*");

  // Failure case 2: destination file already exists.
  DRAKE_EXPECT_THROWS_MESSAGE(
      RenderClient::RenameHttpServiceResponse(scene, scene, ".gltf"),
      ".*refusing to rename.*file already exists.*");
}

TEST_F(RenderClientTest, LoadColorImageGood) {
  // Loading a three channel (RGB) png file should work as expected.
  ImageRgba8U rgb(kTestImageWidth, kTestImageHeight, 0);
  RenderClient::LoadColorImage(kTestRgbImagePath, &rgb);
  EXPECT_EQ(rgb, CreateTestColorImage(true));

  // Loading a four channel (RGBA) png file should work as expected.
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
      RenderClient::LoadColorImage(kTestLabelImagePath, &ignored),
      ".*PNG image.*has 1 channel.*");
}

TEST_F(RenderClientTest, LoadDepthGood) {
  // Loading a single channel 32 bit tiff file should work as expected.
  ImageDepth32F depth(kTestImageWidth, kTestImageHeight, 0);
  RenderClient::LoadDepthImage(kTestDepthImagePath, &depth);
  EXPECT_EQ(depth, CreateTestDepthImage());
}

TEST_F(RenderClientTest, LoadDepthImageBad) {
  ImageDepth32F ignored(kTestImageWidth, kTestImageHeight, 0);

  // Failure case 1: no such file.
  DRAKE_EXPECT_THROWS_MESSAGE(
      RenderClient::LoadDepthImage("/no/such/file", &ignored),
      ".*cannot load.*/no/such/file.*");

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
        RenderClient::LoadDepthImage(kTestDepthImagePath, &wrong_size),
        ".*expected.*but got.*width=.*height=.*");
  }
}

TEST_F(RenderClientTest, LoadLabelImageGood) {
  // Loading a 16 bit label image file should work as expected.
  ImageLabel16I label(kTestImageWidth, kTestImageHeight, 0);
  RenderClient::LoadLabelImage(kTestLabelImagePath, &label);
  EXPECT_EQ(label, CreateTestLabelImage());
}

TEST_F(RenderClientTest, LoadLabelImageBad) {
  ImageLabel16I ignored(kTestImageWidth, kTestImageHeight, 0);

  // Failure case 1: no such file.
  DRAKE_EXPECT_THROWS_MESSAGE(
      RenderClient::LoadLabelImage("/no/such/file", &ignored),
      ".*cannot load.*/no/such/file.*");

  // Failure case 2: not a valid image file.
  DRAKE_EXPECT_THROWS_MESSAGE(
      RenderClient::LoadLabelImage(Touch(scratch_/"fake.png"), &ignored),
      ".*cannot load.*fake.*");

  // Failure case 3: wrong image dimensions (on each axis).
  for (const bool selector : {true, false}) {
    int width = kTestImageWidth;
    int height = kTestImageHeight;
    if (selector) { ++width; } else { ++height; }
    ImageLabel16I wrong_size(width, height, 0);
    DRAKE_EXPECT_THROWS_MESSAGE(
        RenderClient::LoadLabelImage(kTestLabelImagePath, &wrong_size),
        ".*expected.*but got.*width=.*height=.*");
  }

  // Failure case 4: wrong number of channels (== 4) instead of 1.
  DRAKE_EXPECT_THROWS_MESSAGE(
      RenderClient::LoadLabelImage(kTestRgbImagePath, &ignored),
      ".*PNG image.*has 3 channel.*");
}

}  // namespace
}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
