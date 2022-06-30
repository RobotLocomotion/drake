#include "drake/geometry/render/dev/render_gltf_client/internal_render_client.h"

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
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/render/dev/render_gltf_client/test/internal_sample_image_data.h"
#include "drake/geometry/render_gltf_client/internal_http_service.h"

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

class RenderClientTester {
 public:
  explicit RenderClientTester(const RenderClient* client) : client_(*client) {}

  void ValidateAttributes(const std::string& base_url,
                          const std::string& render_endpoint, bool verbose,
                          bool no_cleanup) const {
    EXPECT_EQ(client_.base_url(), base_url);
    EXPECT_EQ(client_.render_endpoint(), render_endpoint);
    EXPECT_EQ(client_.verbose(), verbose);
    EXPECT_EQ(client_.no_cleanup(), no_cleanup);
  }

  void CompareAttributes(const RenderClient& other) {
    ValidateAttributes(other.base_url(), other.render_endpoint(),
                       other.verbose(), other.no_cleanup());
  }

 private:
  const RenderClient& client_;
};

namespace {

// Constexpr dimensions for the actual testing images.
constexpr int kTestImageWidth = 3;
constexpr int kTestImageHeight = 2;

// The paths for the testing images used across different unit tests.
const auto kTestRgbImagePath = FindResourceOrThrow(
    "drake/geometry/render/dev/render_gltf_client/test/test_rgb_8U.png");
const auto kTestRgbaImagePath = FindResourceOrThrow(
    "drake/geometry/render/dev/render_gltf_client/test/test_rgba_8U.png");
const auto kTestDepthImagePath = FindResourceOrThrow(
    "drake/geometry/render/dev/render_gltf_client/test/test_depth_32F.tiff");
const auto kTestLabelImagePath = FindResourceOrThrow(
    "drake/geometry/render/dev/render_gltf_client/test/test_label_16I.png");

// Constructor / destructor ----------------------------------------------------
GTEST_TEST(RenderClient, Constructor) {
  const std::string base_url{"127.0.0.1:8000"};
  const std::string render_endpoint{"render"};
  const bool verbose = false;

  {
    // Verify attributes after valid construction.
    auto make_client_and_verify = [&](bool p_no_cleanup) {
      std::string temp_directory;
      {
        RenderClient client{Params{base_url, render_endpoint, std::nullopt,
                                   verbose, p_no_cleanup}};
        temp_directory = client.temp_directory();
        RenderClientTester tester{&client};
        EXPECT_TRUE(fs::is_directory(client.temp_directory()));
        tester.ValidateAttributes(base_url, render_endpoint, verbose,
                                  p_no_cleanup);
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
}

GTEST_TEST(RenderClient, Destructor) {
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
                          const std::string& /* base_url */,
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
                          const std::string& base_url,
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
    std::string unexpected_keys{""};
    for (const auto& pair : data_fields) {
      const auto& key = pair.first;
      if (data_fields_keys.find(key) == data_fields_keys.end()) {
        // no cover: these only execute if a test is failing.
        // LCOV_EXCL_START
        if (unexpected_keys.empty()) {
          unexpected_keys = "Unexpected key(s) in data_fields: " + key;
        } else {
          unexpected_keys += ", " + key;
        }
        // LCOV_EXCL_STOP
      }
    }
    EXPECT_EQ(unexpected_keys, "");

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

using PostFormCallback = typename std::function<HttpResponse(
    const std::string&, const DataFieldsMap&, const FileFieldsMap&)>;

/* A proxy HttpService that can be constructed with an std::function to modify
 the behavior of PostForm. */
class ProxyService : public HttpService {
 public:
  explicit ProxyService(const PostFormCallback& callback)
      : HttpService(), post_form_callback_{callback} {}

  HttpResponse DoPostForm(const std::string& /* temp_directory */,
                          const std::string& base_url,
                          const DataFieldsMap& data_fields,
                          const FileFieldsMap& file_fields,
                          bool /* verbose */ = false) override {
    return post_form_callback_(base_url, data_fields, file_fields);
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
GTEST_TEST(RenderClient, RenderOnServer) {
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
  const auto temp_dir_path = fs::path(client.temp_directory());

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
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(depth_camera.core(),
                              RenderImageType::kDepthDepth32F, fake_scene_path),
        "RenderOnServer: depth image render requested, but no depth_range was "
        "provided.");

    // Providing depth_range for non-depth should raise.
    const auto expected_message =
        "RenderOnServer: the depth_range parameter may only be provided when "
        "the image_type is a depth image.";
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(),
                              RenderImageType::kColorRgba8U, fake_scene_path,
                              std::nullopt, depth_camera.depth_range()),
        expected_message);
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(), RenderImageType::kLabel16I,
                              fake_scene_path, std::nullopt,
                              depth_camera.depth_range()),
        expected_message);
  }

  {
    /* Verify that the RenderClient::RenderOnServer adheres to the API contract
     by populating the correct data and file fields.  Tests in this section
     use DRAKE_EXPECTS_THROWS_MESSAGE all looking for the same error being
     raised, as no image response is provided.  However, the bulk of the test
     takes place in the assertions in FieldCheckService::PostForm. */
    const auto expected_message = fmt::format(
        "\\s*ERROR doing POST:\\s*/{0}\\s*"  // ERROR doing POST: /{endpoint}
        "Server Base URL:\\s*{1}\\s*"        // Server Base URL:  {base_url}
        "Service Message:\\s*None\\.\\s*"    // Service Message:  None.
        "HTTP Code:\\s*500\\s*"              // Http Code:        {code}
        "Server Message:\\s*None\\.\\s*",    // Server Message:   None.
        render_endpoint, base_url);

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

  // Trampoline helper to set the client HttpService.
  const auto response_path = (temp_dir_path / "response.file").string();
  auto set_proxy = [&](const PostFormCallback& callback) {
    // Delete the response file if it exists to start clean on each test.
    try {
      fs::remove(response_path);
      // no cover: this does not ever fail.
      // LCOV_EXCL_START
    } catch (...) {
    }
    // LCOV_EXCL_STOP
    // Set the service with the provided callback.
    client.SetHttpService(std::make_unique<ProxyService>(callback));
  };

  {
    /* Test bad HttpResponse's that have a server message provided.  Previous
     tests validating the FieldCheckService exception have already validated the
     path that the HttpResponse.data_path is not populated.  Edge cases should
     all produce 'None.' as the server message.
     NOTE: file extensions do not matter. */
    // NOTE: all tests using this must use http code 400.
    const auto message_template = fmt::format(
        "\\s*ERROR doing POST:\\s*/{0}\\s*"  // ERROR doing POST: /{endpoint}
        "Server Base URL:\\s*{1}\\s*"        // Server Base URL:  {base_url}
        "Service Message:\\s*None\\.\\s*"    // Service Message:  None.
        "HTTP Code:\\s*400\\s*"              // Http Code:        {code}
        "Server Message:\\s*{{}}\\s*",       // Server Message:   {message}
        render_endpoint, base_url);

    // Case 1: edge case, service populated data_path but file is length 0.
    set_proxy(
        [&](const std::string&, const DataFieldsMap&, const FileFieldsMap&) {
          std::ofstream response{response_path};
          response.close();
          HttpResponse ret;
          ret.http_code = 400;
          ret.data_path = response_path;
          return ret;
        });
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(),
                              RenderImageType::kColorRgba8U, fake_scene_path),
        fmt::format(message_template, "None\\."));

    // Case 2: edge case, bad response but provided message "too long".
    set_proxy(
        [&](const std::string&, const DataFieldsMap&, const FileFieldsMap&) {
          std::ofstream response{response_path};
          // NOTE: this value is hard-coded in RenderClient::RenderOnServer.
          for (int i = 0; i < 8192; ++i) response << '0';
          response.close();
          HttpResponse ret;
          ret.http_code = 400;
          ret.data_path = response_path;
          return ret;
        });
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(), RenderImageType::kLabel16I,
                              fake_scene_path),
        fmt::format(message_template, "None\\."));

    // Case 3: edge case, message provided of valid length but cannot be opened.
    set_proxy([&](const std::string&, const DataFieldsMap&,
                  const FileFieldsMap&) {
      std::ofstream response{response_path};
      response << "If only you could read me!\n";
      response.close();
      fs::permissions(response_path, fs::perms::all, fs::perm_options::remove);
      HttpResponse ret;
      ret.http_code = 400;
      ret.data_path = response_path;
      return ret;
    });
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(),
                              RenderImageType::kColorRgba8U, fake_scene_path),
        fmt::format(message_template, "None\\."));

    // Case 4: server response that can be read.
    const auto response_text = "You are not a valid request :p";
    set_proxy(
        [&](const std::string&, const DataFieldsMap&, const FileFieldsMap&) {
          std::ofstream response{response_path};
          response << response_text;
          response.close();
          HttpResponse ret;
          ret.http_code = 400;
          ret.data_path = response_path;
          return ret;
        });
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(), RenderImageType::kLabel16I,
                              fake_scene_path),
        fmt::format(message_template, response_text));
  }

  {
    // No file response from server should be reported correctly.
    set_proxy(
        [&](const std::string&, const DataFieldsMap&, const FileFieldsMap&) {
          HttpResponse ret;
          ret.http_code = 200;
          return ret;
        });
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(),
                              RenderImageType::kColorRgba8U, fake_scene_path),
        fmt::format(
            "ERROR with POST /{} response from server, base_url={}, HTTP "
            "code=200: the server was supposed to respond with a file but did "
            "not.",
            render_endpoint, base_url));
  }

  {
    // File response provided that does not exist should be reported correctly.
    set_proxy(
        [&](const std::string&, const DataFieldsMap&, const FileFieldsMap&) {
          // NOTE: set_proxy deletes the file.
          HttpResponse ret;
          ret.http_code = 200;
          ret.data_path = response_path;
          return ret;
        });
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(), RenderImageType::kLabel16I,
                              fake_scene_path),
        fmt::format(
            "ERROR with POST /{} response from service, base_url={}, HTTP "
            "code=200: the service responded with a file path '{}' but the "
            "file does not exist.",
            render_endpoint, base_url, response_path));
  }

  {
    // File response cannot be loaded as image should be reported correctly.
    set_proxy(
        [&](const std::string&, const DataFieldsMap&, const FileFieldsMap&) {
          std::ofstream response{response_path};
          response << "I am not an image file!\n";
          response.close();
          HttpResponse ret;
          ret.http_code = 200;
          ret.data_path = response_path;
          return ret;
        });
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(),
                              RenderImageType::kColorRgba8U, fake_scene_path),
        fmt::format(
            "RenderClient: while trying to render the scene '{}' with a sha256 "
            "hash of '{}', the file returned by the server saved in '{}' is "
            "not understood as an image type that is supported.  Image types "
            "attempted loading as: PNG, TIFF.",
            fake_scene_path, fake_scene_sha256, response_path));
  }

  {
    // Copy a "valid" PNG file and check that it is renamed.
    set_proxy(
        [&](const std::string&, const DataFieldsMap&, const FileFieldsMap&) {
          fs::copy_file(kTestRgbaImagePath, response_path);
          HttpResponse ret;
          ret.http_code = 200;
          ret.data_path = response_path;
          return ret;
        });
    const auto expected_path =
        fs::path(fake_scene_path).replace_extension(".png").string();
    const auto response_png = client.RenderOnServer(
        color_camera.core(), RenderImageType::kColorRgba8U, fake_scene_path);
    EXPECT_EQ(response_png, expected_path);
  }

  {
    /* Manufacture a "valid" (not the same width and height) TIFF file and check
     that it is renamed. */
    set_proxy(
        [&](const std::string&, const DataFieldsMap&, const FileFieldsMap&) {
          fs::copy_file(kTestDepthImagePath, response_path);
          HttpResponse ret;
          ret.http_code = 200;
          ret.data_path = response_path;
          return ret;
        });
    const auto expected_path =
        fs::path(fake_scene_path).replace_extension(".tiff").string();
    const auto response_tiff = client.RenderOnServer(
        depth_camera.core(), RenderImageType::kDepthDepth32F, fake_scene_path,
        "test/mime_type", depth_camera.depth_range());
    EXPECT_EQ(response_tiff, expected_path);
  }

  fs::remove_all(temp_dir_path);
}

GTEST_TEST(RenderClient, ComputeSha256) {
  const std::string base_url{"127.0.0.1:8000"};
  const std::string render_endpoint{"render"};
  const bool verbose = false;
  const bool no_cleanup = false;
  RenderClient client{
      Params{base_url, render_endpoint, std::nullopt, verbose, no_cleanup}};

  {
    // Failure case 1: provided input file does not exist.
    const auto unlikely = "/unlikely/to/be/a.file";
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.ComputeSha256(unlikely),
        fmt::format("ComputeSha256: input file '{}' does not exist.",
                    unlikely));
  }

  {
    // Failure case 2: sha256 cannot be computed.
    const std::string bad_path = fs::path(client.temp_directory()) / "bad.file";
    std::ofstream bad_file{bad_path};
    bad_file << "contents should not matter.\n";
    bad_file.close();
    fs::permissions(bad_path, fs::perms::all, fs::perm_options::remove);
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.ComputeSha256(bad_path),
        fmt::format(
            "ComputeSha256: unable to compute hash: cannot open file '{}'.",
            bad_path));
    fs::remove(bad_path);
  }

  {
    // Success case 1.
    const std::string f1_path = fs::path(client.temp_directory()) / "f1.bin";
    std::ofstream f1_file{f1_path};
    f1_file << "some valuable data!\n";
    f1_file.close();
    // echo 'some valuable data!' | sha256sum
    EXPECT_EQ(
        "03cbafcd18635120cee6c8d51ac3534a315649627d822651b3d2b587e81ab6e5",
        client.ComputeSha256(f1_path));
    fs::remove(f1_path);

    // Success case 2 (proof that different hashes returned).
    const std::string f2_path = fs::path(client.temp_directory()) / "f2.bin";
    std::ofstream f2_file{f2_path};
    f2_file << "additional data that is also valuable?";
    f2_file.close();
    // echo -n 'additional data that is also valuable?' | sha256sum
    EXPECT_EQ(
        "d40433c78b8f35adf78d25bcc374ea9dbf42f96d85ff29a8d726fe37178878fe",
        client.ComputeSha256(f2_path));
    fs::remove(f2_path);
  }
}

GTEST_TEST(RenderClient, RenameHttpServiceResponse) {
  const std::string base_url{"127.0.0.1:8000"};
  const std::string render_endpoint{"render"};
  // Keep verbose and no_cleanup `true` to get coverage on log() calls.
  const bool verbose = true;
  const bool no_cleanup = true;
  const RenderClient client{
      Params{base_url, render_endpoint, std::nullopt, verbose, no_cleanup}};
  const fs::path temp_dir = fs::path(client.temp_directory());
  const std::string scene = temp_dir / "scene.gltf";
  std::ofstream scene_file{scene};
  scene_file << "not a real glTF scene!\n";
  scene_file.close();
  const auto unlikely = "/unlikely/to/be/a.file";

  {
    // Failure case 1: file to rename does not exist.
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenameHttpServiceResponse(unlikely, scene, ".png"),
        fmt::format("RenderClient: cannot rename '{}', file does not exist.",
                    unlikely));

    // Failure case 2: file to rename based off does not exist.
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenameHttpServiceResponse(scene, unlikely, ".tiff"),
        fmt::format(
            "RenderClient: cannot rename '{0}' to '{1}' with extension '{2}': "
            "'{1}' does not exist.",
            scene, unlikely, ".tiff"));

    // Failure case 3: destination file already exists.
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenameHttpServiceResponse(scene, scene, ".gltf"),
        fmt::format(
            "RenderClient: refusing to rename '{}' to '{}', file already "
            "exists!",
            scene, scene));
  }

  {
    /* Creates input scene and path for easy enumeration of extension rename
     edge cases below. */
    struct RenameResult {
      RenameResult(const std::string response_data_path,
                   std::string reference_path, const std::string extension,
                   const std::string expected)
          : response_data_path_{response_data_path},
            reference_path_{reference_path},
            extension_{extension},
            expected_{expected} {
        std::ofstream reference_path_file{reference_path_};
        reference_path_file << "input_scene with contents.\n";
        reference_path_file.close();

        std::ofstream response_data_path_file{response_data_path_};
        response_data_path_file << "path with contents.\n";
        response_data_path_file.close();
      }

      const std::string response_data_path_;
      const std::string reference_path_;
      const std::string extension_;
      const std::string expected_;
    };

    /* NOTE: files are created, not deleted, do not reuse names.
     Additionally, do not test for no extension input_scene + ext="", this is
     already tested for above as an expected exception. */
    const fs::path sub_dir = temp_dir / "sub_directory";
    fs::create_directory(sub_dir);
    const std::vector<RenameResult> renames{
        // All components have extensions.
        {temp_dir / "path_0.bin", temp_dir / "input_0.gltf", ".png",
         temp_dir / "input_0.png"},
        // Input scene does not have a file extension.
        {temp_dir / "path_1.curl", temp_dir / "input_1", ".foo",
         temp_dir / "input_1.foo"},
        // Input path does not have an extension.
        {temp_dir / "path_2", temp_dir / "input_2.gltf", ".tiff",
         temp_dir / "input_2.tiff"},
        // Input and path do not have an extension.
        {temp_dir / "path_3", temp_dir / "input_3", ".zip",
         temp_dir / "input_3.zip"},
        // Empty extension is allowed.
        {temp_dir / "path_4.txt", temp_dir / "input_4.txt", "",
         temp_dir / "input_4"},
        // Source is in different directory.
        {sub_dir / "path_5.bin", temp_dir / "input_5.gltf", ".jpg",
         temp_dir / "input_5.jpg"}};
    for (const auto& r : renames) {
      /* Before renaming, response_data_path_ and reference_path_ exist, and
       expected_ does not. */
      EXPECT_TRUE(fs::is_regular_file(r.response_data_path_));
      EXPECT_TRUE(fs::is_regular_file(r.reference_path_));
      EXPECT_FALSE(fs::is_regular_file(r.expected_));

      // Renaming the file should result in our expected value.
      const auto result = client.RenameHttpServiceResponse(
          r.response_data_path_, r.reference_path_, r.extension_);
      EXPECT_EQ(result, r.expected_);
      EXPECT_EQ(fs::path(result).extension(), r.extension_);

      /* After renaming, response_data_path_ should no longer exist, and
       expected_ should. */
      EXPECT_TRUE(fs::is_regular_file(r.reference_path_));
      EXPECT_FALSE(fs::is_regular_file(r.response_data_path_));
      EXPECT_TRUE(fs::is_regular_file(r.expected_));
    }
  }

  fs::remove_all(temp_dir);
}

GTEST_TEST(RenderClient, LoadColorImage) {
  const std::string base_url{"127.0.0.1:8000"};
  const std::string render_endpoint{"render"};
  const bool verbose = false;
  const bool no_cleanup = false;
  const RenderClient client{
      Params{base_url, render_endpoint, std::nullopt, verbose, no_cleanup}};

  /* Create a Drake Image buffer with the same dimension as the testing images,
   i.e., test_{rgb, rgba}_8U.png, to exercise the image loading code.
   LoadColorImage() will attempt to load the PNG file to the buffer and throw
   exceptions if the dimension, channel number, or data type is incorrect. */
  ImageRgba8U drake_image(kTestImageWidth, kTestImageHeight, 0);
  {
    // Failure case 1: not a valid PNG file.
    const auto expected_message = "RenderClient: cannot load '{}' as PNG.";
    const auto unlikely = "/not/likely/a.png";
    DRAKE_EXPECT_THROWS_MESSAGE(client.LoadColorImage(unlikely, &drake_image),
                                fmt::format(expected_message, unlikely));

    const fs::path temp_dir = fs::path(client.temp_directory());
    const std::string fake_png_path = temp_dir / "fake.png";
    std::ofstream fake_png{fake_png_path};
    fake_png << "not a valid png file.\n";
    fake_png.close();
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.LoadColorImage(fake_png_path, &drake_image),
        fmt::format(expected_message, fake_png_path));
    fs::remove(fake_png_path);
  }

  {
    /* Failure case 2: different image dimensions between the loaded image and
     the Drake Image buffer.  `test_drake_image` should not be accessed. */
    const std::vector<std::pair<int, int>> width_height{
        {1, 1},
        {kTestImageWidth + 12, kTestImageHeight},
        {kTestImageWidth * 2, kTestImageHeight * 2}};
    for (const auto& [w, h] : width_height) {
      ImageRgba8U test_drake_image(w, h, 0);
      DRAKE_EXPECT_THROWS_MESSAGE(
          client.LoadColorImage(kTestRgbaImagePath, &test_drake_image),
          fmt::format("RenderClient: expected to import "
                      "\\(width={},height={}\\) from the "
                      "file '{}', but got \\(width={},height={}\\).",
                      w, h, kTestRgbaImagePath, kTestImageWidth,
                      kTestImageHeight));
    }
  }

  {
    /* Failure case 3: number of channels not equal to 3 or 4. Test this by
     loading a single-channel label png instead. */
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.LoadColorImage(kTestLabelImagePath, &drake_image),
        fmt::format(
            "RenderClient: loaded PNG image from '{}' has 1 channel\\(s\\), "
            "but either 3 \\(RGB\\) or 4 \\(RGBA\\) are required for color "
            "images.",
            kTestLabelImagePath));
  }

  {
    // Loading a three channel (RGB) png file should work as expected.
    DRAKE_EXPECT_NO_THROW(
        client.LoadColorImage(kTestRgbImagePath, &drake_image));
    EXPECT_EQ(drake_image, CreateTestColorImage(true));
  }

  {
    // Loading a four channel (RGBA) png file should work as expected.
    DRAKE_EXPECT_NO_THROW(
        client.LoadColorImage(kTestRgbaImagePath, &drake_image));
    EXPECT_EQ(drake_image, CreateTestColorImage(false));
  }
}

GTEST_TEST(RenderClient, LoadDepthImage) {
  const std::string base_url{"127.0.0.1:8000"};
  const std::string render_endpoint{"render"};
  const bool verbose = false;
  const bool no_cleanup = false;
  const RenderClient client{
      Params{base_url, render_endpoint, std::nullopt, verbose, no_cleanup}};

  /* Create a Drake Image buffer with the same dimension as the testing image,
   i.e., test_depth_32F.tiff, to exercise the image loading code.
   LoadDepthImage() will attempt to load the TIFF file to the buffer and throw
   exceptions if the dimension, channel number, or data type is incorrect. */
  ImageDepth32F drake_image(kTestImageWidth, kTestImageHeight, 0);
  {
    // Failure case 1: not a valid TIFF file.
    const auto expected_message = "RenderClient: cannot load '{}' as TIFF.";
    const auto unlikely = "/not/likely/a.tiff";
    DRAKE_EXPECT_THROWS_MESSAGE(client.LoadDepthImage(unlikely, &drake_image),
                                fmt::format(expected_message, unlikely));

    const fs::path temp_dir = fs::path(client.temp_directory());
    const std::string fake_tiff_path = temp_dir / "fake.tiff";
    std::ofstream fake_tiff{fake_tiff_path};
    fake_tiff << "not a valid tiff file.\n";
    fake_tiff.close();
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.LoadDepthImage(fake_tiff_path, &drake_image),
        fmt::format(expected_message, fake_tiff_path));
    fs::remove(fake_tiff_path);
  }

  {
    /* Failure case 2: different image dimensions between the loaded image and
     the drake image buffer.  `test_drake_image` should not be accessed. */
    const std::vector<std::pair<int, int>> width_height{
        {1, 1},
        {kTestImageWidth + 12, kTestImageHeight},
        {kTestImageWidth * 2, kTestImageHeight * 2}};
    for (const auto& [w, h] : width_height) {
      ImageDepth32F test_drake_image(w, h, 0);
      DRAKE_EXPECT_THROWS_MESSAGE(
          client.LoadDepthImage(kTestDepthImagePath, &test_drake_image),
          fmt::format("RenderClient: expected to import "
                      "\\(width={},height={}\\) from the "
                      "file '{}', but got \\(width={},height={}\\).",
                      w, h, kTestDepthImagePath, kTestImageWidth,
                      kTestImageHeight));
    }
  }

  {
    // Loading a single channel 32 bit tiff file should work as expected.
    DRAKE_EXPECT_NO_THROW(
        client.LoadDepthImage(kTestDepthImagePath, &drake_image));
    EXPECT_EQ(drake_image, CreateTestDepthImage());
  }
}

GTEST_TEST(RenderClient, LoadLabelImage) {
  const std::string base_url{"127.0.0.1:8000"};
  const std::string render_endpoint{"render"};
  const bool verbose = false;
  const bool no_cleanup = true;
  const RenderClient client{
      Params{base_url, render_endpoint, std::nullopt, verbose, no_cleanup}};

  /* Create a Drake Image buffer with the same dimension as the testing image,
   i.e., test_label_16U.png, to exercise the image loading code.
   LoadLabelImage() will attempt to load the PNG file to the buffer and throw
   exceptions if the dimension, channel number, or data type is incorrect. */
  ImageLabel16I drake_image(kTestImageWidth, kTestImageHeight, 0);
  {
    // Failure case 1: not a valid PNG file.
    const auto expected_message = "RenderClient: cannot load '{}' as PNG.";
    const auto unlikely = "/not/likely/a.png";
    DRAKE_EXPECT_THROWS_MESSAGE(client.LoadLabelImage(unlikely, &drake_image),
                                fmt::format(expected_message, unlikely));

    const fs::path temp_dir = fs::path(client.temp_directory());
    const std::string fake_png_path = temp_dir / "fake.png";
    std::ofstream fake_png{fake_png_path};
    fake_png << "not a valid png file.\n";
    fake_png.close();
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.LoadLabelImage(fake_png_path, &drake_image),
        fmt::format(expected_message, fake_png_path));
    fs::remove(fake_png_path);
  }

  {
    /* Failure case 2: different image dimensions between the loaded image and
     the drake image buffer.  `test_drake_image` should not be accessed. */
    const std::vector<std::pair<int, int>> width_height{
        {1, 1},
        {kTestImageWidth + 12, kTestImageHeight},
        {kTestImageWidth * 2, kTestImageHeight * 2}};
    for (const auto& [w, h] : width_height) {
      ImageLabel16I test_drake_image(w, h, 0);
      DRAKE_EXPECT_THROWS_MESSAGE(
          client.LoadLabelImage(kTestLabelImagePath, &test_drake_image),
          fmt::format("RenderClient: expected to import "
                      "\\(width={},height={}\\) from the "
                      "file '{}', but got \\(width={},height={}\\).",
                      w, h, kTestLabelImagePath, kTestImageWidth,
                      kTestImageHeight));
    }
  }

  {
    /* Failure case 3: number of channels not equal to 1. Test this by loading
     an RGB image instead. */
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.LoadLabelImage(kTestRgbImagePath, &drake_image),
        fmt::format(
            "RenderClient: loaded PNG image from '{}' has 3 channels, but only "
            "1 is allowed for label images.",
            kTestRgbImagePath));
  }

  {
    // Loading a 16 bit label image file should work as expected.
    DRAKE_EXPECT_NO_THROW(
        client.LoadLabelImage(kTestLabelImagePath, &drake_image));
    EXPECT_EQ(drake_image, CreateTestLabelImage());
  }
}

}  // namespace
}  // namespace internal
}  // namespace render_gltf_client
}  // namespace geometry
}  // namespace drake
