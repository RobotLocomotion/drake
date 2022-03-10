#include "drake/geometry/render/dev/render_client.h"

#include <cstdio>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <fmt/format.h>
#include <gtest/gtest.h>
#include <libpng/png.h>
#include <tiffio.h>

#include "drake/common/filesystem.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/unused.h"
#include "drake/geometry/render/dev/http_service.h"

namespace drake {
namespace geometry {
namespace render {
namespace internal {

using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

class RenderClientTester {
 public:
  explicit RenderClientTester(const RenderClient* client) : client_(*client) {}

  void ValidateAttributes(const std::string& url, int32_t port,
                          const std::string& render_endpoint, bool verbose,
                          bool no_cleanup) const {
    EXPECT_EQ(client_.url(), url);
    EXPECT_EQ(client_.port(), port);
    EXPECT_EQ(client_.render_endpoint(), render_endpoint);
    EXPECT_EQ(client_.verbose(), verbose);
    EXPECT_EQ(client_.no_cleanup(), no_cleanup);

    EXPECT_EQ(client_.temp_directory(),
              client_.http_service_->temp_directory());
    EXPECT_EQ(client_.http_service_->url(), url);
    EXPECT_EQ(client_.http_service_->port(), port);
    EXPECT_EQ(client_.http_service_->verbose(), verbose);
  }

  void CompareAttributes(const RenderClient& other) {
    ValidateAttributes(other.url(), other.port(), other.render_endpoint(),
                       other.verbose(), other.no_cleanup());
  }

  bool ClientWasCloned() const { return client_.this_was_cloned_; }

 private:
  const RenderClient& client_;
};

namespace {

namespace fs = drake::filesystem;

// Constructor / destructor ----------------------------------------------------
GTEST_TEST(RenderClient, Constructor) {
  const std::string url{"127.0.0.1"};
  const int32_t port{8000};
  const std::string render_endpoint{"render"};
  const bool verbose = false;
  const bool no_cleanup = false;

  {
    // Provided url may not end with slash.
    DRAKE_EXPECT_THROWS_MESSAGE(
        RenderClient(url + "/", port, render_endpoint, verbose, no_cleanup),
        "HttpService: url may not end with '/'\\.");
    // Provided url may not be empty.
    DRAKE_EXPECT_THROWS_MESSAGE(
        RenderClient("", port, render_endpoint, verbose, no_cleanup),
        "HttpService: url parameter may not be empty\\.");
    // Provided endpoint may not start with a slash.
    DRAKE_EXPECT_THROWS_MESSAGE(
        RenderClient(url, port, "/" + render_endpoint, verbose, no_cleanup),
        "Provided endpoint='/render' is not valid, it may not start or end "
        "with a '/'\\.");
    // Provided endpoint may not end with a slash.
    DRAKE_EXPECT_THROWS_MESSAGE(
        RenderClient(url, port, render_endpoint + "/", verbose, no_cleanup),
        "Provided endpoint='render/' is not valid, it may not start or end "
        "with a '/'\\.");
  }

  {
    // Verify attributes after valid construction.
    auto make_client_and_verify = [&](bool p_no_cleanup) {
      std::string temp_directory;
      {
        RenderClient client{url, port, render_endpoint, verbose, p_no_cleanup};
        temp_directory = client.temp_directory();
        RenderClientTester tester{&client};
        EXPECT_TRUE(fs::is_directory(client.temp_directory()));
        tester.ValidateAttributes(url, port, render_endpoint, verbose,
                                  p_no_cleanup);
        EXPECT_FALSE(tester.ClientWasCloned());
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
  const std::string url{"127.0.0.1"};
  const int32_t port{8000};
  const std::string render_endpoint = "render";
  const bool verbose = false;

  std::string temp_dir_path;
  // Construction with no_cleanup=false: temp_directory should be gone.
  {
    RenderClient client{url, port, render_endpoint, verbose, false};
    temp_dir_path = client.temp_directory();
    EXPECT_TRUE(fs::is_directory(temp_dir_path));
  }  // Client is deleted.
  EXPECT_FALSE(fs::is_directory(temp_dir_path));

  // Construction with no_cleanup=true: temp_directory should remain.
  {
    RenderClient client{url, port, render_endpoint, verbose, true};
    temp_dir_path = client.temp_directory();
    EXPECT_TRUE(fs::is_directory(temp_dir_path));
  }  // Client is deleted.
  EXPECT_TRUE(fs::is_directory(temp_dir_path));
  fs::remove(temp_dir_path);
}

// Cloning ---------------------------------------------------------------------
/* RenderClient is a concrete implementation, but it also supports inheritance.
 Derived classes must override DoClone to be considered valid. */
class RenderClientGoodClone : public RenderClient {
 public:
  RenderClientGoodClone(const std::string& url, int32_t port,
                        const std::string& render_endpoint, bool verbose,
                        bool no_cleanup)
      : RenderClient(url, port, render_endpoint, verbose, no_cleanup) {}
  RenderClientGoodClone(const RenderClientGoodClone& other)
      : RenderClient(other) {}
  std::unique_ptr<RenderClient> DoClone() const override {
    return std::unique_ptr<RenderClientGoodClone>(
        new RenderClientGoodClone(*this));
  }
};

// Derived-derived class that forgets to implement DoClone() will raise.
class RenderClientBadClone : public RenderClientGoodClone {
 public:
  RenderClientBadClone(const std::string& url, int32_t port,
                       const std::string& render_endpoint, bool verbose,
                       bool no_cleanup)
      : RenderClientGoodClone(url, port, render_endpoint, verbose, no_cleanup) {
  }
  RenderClientBadClone(const RenderClientBadClone& other)
      : RenderClientGoodClone(other) {}
};

GTEST_TEST(RenderClient, Clone) {
  const std::string url{"localhost"};
  const int32_t port{8888};
  const std::string render_endpoint{"speed_render"};
  const bool verbose{false};
  const bool no_cleanup{false};

  /* Clone verification requires making sure that after deletion, if a
   RenderClient was cloned it does not delete it's temporary directory when
   no_cleanup=true -- the last lived clone will delete it. */
  std::string client_temp_directory;
  {
    std::unique_ptr<RenderClient> clone = nullptr;
    {
      // Clone of a RenderClient should work as expected.
      RenderClient client{url, port, render_endpoint, verbose, no_cleanup};
      RenderClientTester client_tester{&client};
      client_temp_directory = client.temp_directory();
      clone = client.Clone();
      RenderClientTester clone_tester{clone.get()};

      client_tester.CompareAttributes(*clone);
      EXPECT_TRUE(client_tester.ClientWasCloned());
      EXPECT_FALSE(clone_tester.ClientWasCloned());
    }  // Client deleted, clone is still in scope.
    EXPECT_TRUE(fs::is_directory(client_temp_directory));
  }  // All instances are deleted, temp directory removed.
  EXPECT_FALSE(fs::is_directory(client_temp_directory));

  {
    // Verify that Clone() works the same for derived classes.
    std::unique_ptr<RenderClient> clone = nullptr;
    {
      // Clone of a RenderClientGoodClone should work as expected.
      RenderClientGoodClone client{url, port, render_endpoint, verbose,
                                   no_cleanup};
      RenderClientTester client_tester{&client};
      client_temp_directory = client.temp_directory();
      clone = client.Clone();
      RenderClientTester clone_tester{clone.get()};

      client_tester.CompareAttributes(*clone);
      EXPECT_TRUE(client_tester.ClientWasCloned());
      EXPECT_FALSE(clone_tester.ClientWasCloned());
    }  // Client deleted, clone is still in scope.
    EXPECT_TRUE(fs::is_directory(client_temp_directory));
  }  // All instances are deleted, temp directory removed.
  EXPECT_FALSE(fs::is_directory(client_temp_directory));

  {
    // Clone() on a class that did not override DoClone() should raise.
    RenderClientBadClone client{url, port, render_endpoint, verbose,
                                no_cleanup};
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.Clone(),
        "Error in cloning RenderClient class of type.*RenderClientBadClone; "
        "the clone returns type .*RenderClientGoodClone\\..*"
        "RenderClientBadClone::DoClone\\(\\) was probably not implemented");
  }
}

// RenderOnServer --------------------------------------------------------------
// Convenience definitions for interacting with HttpService.
using data_map_t = std::map<std::string, std::string>;
using file_map_t =
    std::map<std::string, std::pair<std::string, std::optional<std::string>>>;

// A simple HttpService that always fails.
class FailService : public HttpService {
 public:
  FailService(const std::string& temp_directory, const std::string& url,
              int32_t port, bool verbose)
      : HttpService(temp_directory, url, port, verbose) {}

  FailService(const FailService& other) : HttpService(other) {}

  std::unique_ptr<HttpService> DoClone() const override {
    return std::unique_ptr<FailService>(new FailService(*this));
  }

  HttpResponse PostForm(const std::string& /* endpoint */,
                        const data_map_t& /* data_fields */,
                        const file_map_t& /* file_fields */) override {
    HttpResponse ret;
    ret.http_code = 500;
    ret.service_error = true;
    ret.service_error_message = "FailService always fails.";
    return ret;
  }
};

/* Verifies the contract fullfilled by RenderClient::RenderOnServer, checking
 that all fields are exactly as expected (and where min_depth / max_depth are
 concerned, they are included / excluded as expected).  The code checks in
 PostForm are more or less a duplication of RenderClient::RenderOnServer,
 meaning any changes to the code populating the <form> will break this test case
 (intentionally).

 A test using a FieldCheckService will construct with the exact same parameters
 that RenderClient::RenderOnServer is going to use, so that when PostForm is
 called behind the scenes we have all the information to cross-check against. */
class FieldCheckService : public HttpService {
 public:
  /* All parameters for HttpService, followed by RenderClient::RenderOnServer
   with the addition of the sha256. */
  FieldCheckService(const std::string& temp_directory, const std::string& url,
                    int32_t port, bool verbose,
                    const RenderCameraCore& camera_core,
                    RenderImageType image_type, const std::string& scene_path,
                    const std::string& scene_sha256,
                    const std::optional<std::string>& mime_type,
                    double min_depth = -1.0, double max_depth = -1.0)
      : HttpService(temp_directory, url, port, verbose),
        camera_core_{camera_core},
        image_type_{image_type},
        scene_path_{scene_path},
        scene_sha256_{scene_sha256},
        mime_type_{mime_type},
        min_depth_{min_depth},
        max_depth_{max_depth} {}

  FieldCheckService(const FieldCheckService& other)
      : HttpService(other),
        camera_core_{other.camera_core_},
        image_type_{other.image_type_},
        scene_path_{other.scene_path_},
        scene_sha256_{other.scene_sha256_},
        mime_type_{other.mime_type_},
        min_depth_{other.min_depth_},
        max_depth_{other.max_depth_} {}

  std::unique_ptr<HttpService> DoClone() const override {
    return std::unique_ptr<FieldCheckService>(new FieldCheckService(*this));
  }

  /* Check all of the <form> fields.  Always respond with failure (http 500). */
  HttpResponse PostForm(const std::string& endpoint,
                        const data_map_t& data_fields,
                        const file_map_t& file_fields) override {
    ThrowIfEndpointInvalid(endpoint);
    ThrowIfFilesMissing(file_fields);

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
      EXPECT_EQ(data_fields.at("min_depth"), std::to_string(min_depth_));
      EXPECT_EQ(data_fields.at("max_depth"), std::to_string(max_depth_));
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
        if (unexpected_keys.empty()) {
          unexpected_keys = "Unexpected key(s) in data_fields: " + key;
        } else {
          unexpected_keys += ", " + key;
        }
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
  const double min_depth_;
  const double max_depth_;
};

using PostFormCallback_t = typename std::function<HttpResponse(
    const std::string&, const data_map_t&, const file_map_t&)>;

/* A proxy HttpService that can be cunstructed with an std::function to modify
 the behavior of PostForm. */
class ProxyService : public HttpService {
 public:
  ProxyService(const std::string& temp_directory, const std::string& url,
               int32_t port, bool verbose, const PostFormCallback_t& callback)
      : HttpService(temp_directory, url, port, verbose),
        post_form_callback_{callback} {}

  ProxyService(const ProxyService& other) : HttpService(other) {}

  std::unique_ptr<HttpService> DoClone() const override {
    return std::unique_ptr<ProxyService>(new ProxyService(*this));
  }

  HttpResponse PostForm(
      const std::string& endpoint,
      const std::map<std::string, std::string>& data_fields,
      const std::map<std::string,
                     std::pair<std::string, std::optional<std::string>>>&
          file_fields) override {
    ThrowIfEndpointInvalid(endpoint);
    ThrowIfFilesMissing(file_fields);
    return post_form_callback_(endpoint, data_fields, file_fields);
  }

  PostFormCallback_t post_form_callback_;
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
  const std::string url{"notarealserver"};
  const int32_t port{8192};
  const std::string render_endpoint{"no_render"};
  const bool verbose{false};
  const bool no_cleanup{false};

  // Create a client and proxy HttpService creation helper.
  RenderClient client{url, port, render_endpoint, verbose, no_cleanup};
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
    /* Invalid depth range should raise an exception.  Exhaustive testing of
     ValidDepthRangeOrThrow happens elsewhere. */
    client.SetHttpService(
        std::make_unique<FailService>(temp_dir_path, url, port, verbose));
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(depth_camera.core(),
                              RenderImageType::kDepthDepth32F, fake_scene_path,
                              std::nullopt, /* min_depth */ 10.0,
                              /* max_depth */ 1.0),
        "max_depth cannot be less than or equal to min_depth\\.");

    // Providing min_depth / max_depth for non-depth should raise.
    const auto expected_message =
        "min_depth and max_depth are only allowed with a depth "
        "RenderImageType.";
    const std::vector<std::pair<double, double>> min_max{
        {1.0, -1.0}, {1.0, 10.0}, {-1.0, 10.0}};
    for (const auto& [min_depth, max_depth] : min_max) {
      DRAKE_EXPECT_THROWS_MESSAGE(
          client.RenderOnServer(color_camera.core(),
                                RenderImageType::kColorRgba8U, fake_scene_path,
                                std::nullopt, min_depth, max_depth),
          expected_message);
      DRAKE_EXPECT_THROWS_MESSAGE(
          client.RenderOnServer(color_camera.core(), RenderImageType::kLabel16I,
                                fake_scene_path, std::nullopt, min_depth,
                                max_depth),
          expected_message);
    }
  }

  {
    /* Verify that the RenderClient::RenderOnServer adheres to the API contract
     by populating the correct data and file fields.  Tests in this section
     use DRAKE_EXPECTS_THROWS_MESSAGE all looking for the same error being
     raised, as no image response is provided.  However, the bulk of the test
     takes place in the assertions in FieldCheckService::PostForm. */
    const auto expected_message = fmt::format(
        "\\s*ERROR doing POST:\\s*/{0}\\s*"  // ERROR doing POST: /{endpoint}
        "Server URL:\\s*{1}:{2}\\s*"         // Server URL:       {url}:{port}
        "Service Message:\\s*None\\.\\s*"    // Service Message:  None.
        "HTTP Code:\\s*500\\s*"              // Http Code:        {code}
        "Server Message:\\s*None\\.\\s*",    // Server Message:   None.
        render_endpoint, url, port);

    // Check fields for a color render.
    client.SetHttpService(std::make_unique<FieldCheckService>(
        temp_dir_path, url, port, verbose, color_camera.core(),
        RenderImageType::kColorRgba8U, fake_scene_path, fake_scene_sha256,
        std::nullopt));
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(),
                              RenderImageType::kColorRgba8U, fake_scene_path,
                              std::nullopt),
        expected_message);

    /* Check fields for a depth render.  This test also includes a verification
     that the provided mime_type is propagated correctly.  There is no special
     reason for this to be checked with the depth render, it is just convenient
     since a new HttpService is being set for the client. */
    const auto& depth_range = depth_camera.depth_range();
    client.SetHttpService(std::make_unique<FieldCheckService>(
        temp_dir_path, url, port, verbose, depth_camera.core(),
        RenderImageType::kDepthDepth32F, fake_scene_path, fake_scene_sha256,
        "test/mime_type", depth_range.min_depth(), depth_range.max_depth()));
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(depth_camera.core(),
                              RenderImageType::kDepthDepth32F, fake_scene_path,
                              "test/mime_type", depth_range.min_depth(),
                              depth_range.max_depth()),
        expected_message);

    // Check fields for a label render.
    client.SetHttpService(std::make_unique<FieldCheckService>(
        temp_dir_path, url, port, verbose, color_camera.core(),
        RenderImageType::kLabel16I, fake_scene_path, fake_scene_sha256,
        std::nullopt));
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(), RenderImageType::kLabel16I,
                              fake_scene_path, std::nullopt),
        expected_message);
  }

  // Trampoline helper to set the client HttpService.
  const auto response_path = (temp_dir_path / "response.file").string();
  auto set_proxy = [&](const PostFormCallback_t& callback) {
    // Delete the response file if it exists to start clean on each test.
    try {
      fs::remove(response_path);
    } catch (...) {
    }
    // Set the service with the provided callback.
    client.SetHttpService(std::make_unique<ProxyService>(
        temp_dir_path, url, port, verbose, callback));
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
        "Server URL:\\s*{1}:{2}\\s*"         // Server URL:       {url}:{port}
        "Service Message:\\s*None\\.\\s*"    // Service Message:  None.
        "HTTP Code:\\s*400\\s*"              // Http Code:        {code}
        "Server Message:\\s*{{}}\\s*",       // Server Message:   {message}
        render_endpoint, url, port);

    // Case 1: edge case, service populated data_path but file is length 0.
    set_proxy([&](const std::string&, const data_map_t&, const file_map_t&) {
      std::ofstream response{response_path};
      response.close();
      HttpResponse ret;
      ret.http_code = 400;
      ret.data_path = response_path;
      return ret;
    });
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(),
                              RenderImageType::kColorRgba8U, fake_scene_path,
                              std::nullopt),
        fmt::format(message_template, "None\\."));

    // Case 2: edge case, bad response but provided message "too long".
    set_proxy([&](const std::string&, const data_map_t&, const file_map_t&) {
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
                              fake_scene_path, std::nullopt),
        fmt::format(message_template, "None\\."));

    // Case 3: edge case, message provided of valid length but cannot be opened.
    set_proxy([&](const std::string&, const data_map_t&, const file_map_t&) {
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
                              RenderImageType::kColorRgba8U, fake_scene_path,
                              std::nullopt),
        fmt::format(message_template, "None\\."));

    // Case 4: server response that can be read.
    const auto response_text = "You are not a valid request :p";
    set_proxy([&](const std::string&, const data_map_t&, const file_map_t&) {
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
                              fake_scene_path, std::nullopt),
        fmt::format(message_template, response_text));
  }

  {
    // No file response from server should be reported correctly.
    set_proxy([&](const std::string&, const data_map_t&, const file_map_t&) {
      HttpResponse ret;
      ret.http_code = 200;
      return ret;
    });
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(),
                              RenderImageType::kColorRgba8U, fake_scene_path,
                              std::nullopt);
        ,
        fmt::format(
            "ERROR with POST /{} response from server, url={}:{}, HTTP "
            "code=200: the server was supposed to respond with a file but did "
            "not.",
            render_endpoint, url, port));
  }

  {
    // File response provided that does not exist should be reported correctly.
    set_proxy([&](const std::string&, const data_map_t&, const file_map_t&) {
      // NOTE: set_proxy deletes the file.
      HttpResponse ret;
      ret.http_code = 200;
      ret.data_path = response_path;
      return ret;
    });
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenderOnServer(color_camera.core(), RenderImageType::kLabel16I,
                              fake_scene_path, std::nullopt),
        fmt::format(
            "ERROR with POST /{} response from service, url={}:{}, HTTP "
            "code=200: the service responded with a file path '{}' but the "
            "file does not exist.",
            render_endpoint, url, port, response_path));
  }

  {
    // File response cannot be loaded as image should be reported correctly.
    set_proxy([&](const std::string&, const data_map_t&, const file_map_t&) {
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
                              RenderImageType::kColorRgba8U, fake_scene_path,
                              std::nullopt),
        fmt::format(
            "RenderClient: while trying to render the scene '{}' with a sha256 "
            "hash of '{}', the file returned by the server saved in '{}' is "
            "not understood as an image type that is supported.  Image types "
            "attempted loading as: PNG, TIFF.",
            fake_scene_path, fake_scene_sha256, response_path));
  }

  {
    // Copy a "valid" PNG file and check that it is renamed.
    set_proxy([&](const std::string&, const data_map_t&, const file_map_t&) {
      auto box_png =
          FindResourceOrThrow("drake/geometry/render/test/meshes/box.png");
      fs::copy_file(box_png, response_path);
      HttpResponse ret;
      ret.http_code = 200;
      ret.data_path = response_path;
      return ret;
    });
    const auto expected_path =
        fs::path(fake_scene_path).replace_extension(".png").string();
    const auto response_png = client.RenderOnServer(
        color_camera.core(), RenderImageType::kColorRgba8U, fake_scene_path,
        std::nullopt);
    EXPECT_EQ(response_png, expected_path);
  }

  {
    // Manufacture a "valid" TIFF file and check that it is renamed.
    set_proxy([&](const std::string&, const data_map_t&, const file_map_t&) {
      TIFF* tiff = TIFFOpen(response_path.c_str(), "w");
      DRAKE_DEMAND(tiff != nullptr);
      constexpr int width = 4;
      constexpr int height = 1;
      // The data being used is irrelevant, just need to dump 32 bits to file.
      float data[width] = {1.0f, 2.0f, 3.0f, 4.0f};
      /* NOTE: if this assertion ever becomes false, this test can be
       conditionally compiled rather than asserted, however libtiff is being
       promised that it is getting 32 bit data. */
      static_assert(sizeof(float) == 4, "Expected 32 bit floats...");

      TIFFSetField(tiff, TIFFTAG_IMAGEWIDTH, width);
      TIFFSetField(tiff, TIFFTAG_IMAGELENGTH, height);
      TIFFSetField(tiff, TIFFTAG_SAMPLESPERPIXEL, 1);
      TIFFSetField(tiff, TIFFTAG_BITSPERSAMPLE, 32);

      TIFFWriteScanline(tiff, static_cast<void*>(data), 0, 0);
      TIFFClose(tiff);

      HttpResponse ret;
      ret.http_code = 200;
      ret.data_path = response_path;
      return ret;
    });
    const auto expected_path =
        fs::path(fake_scene_path).replace_extension(".tiff").string();
    const auto response_tiff = client.RenderOnServer(
        color_camera.core(), RenderImageType::kColorRgba8U, fake_scene_path,
        std::nullopt);
    EXPECT_EQ(response_tiff, expected_path);
  }

  fs::remove_all(temp_dir_path);
}

GTEST_TEST(RenderClient, ComputeSha256) {
  const std::string url{"127.0.0.1"};
  const int32_t port{8000};
  const std::string render_endpoint{"render"};
  const bool verbose = false;
  const bool no_cleanup = false;
  RenderClient client{url, port, render_endpoint, verbose, no_cleanup};

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

GTEST_TEST(RenderClient, ValidDepthRangeOrThrow) {
  const std::string url{"127.0.0.1"};
  const int32_t port{8000};
  const std::string render_endpoint{"render"};
  const bool verbose = false;
  const bool no_cleanup = false;
  RenderClient client{url, port, render_endpoint, verbose, no_cleanup};

  {
    // Valid depth ranges should not throw.  0.0 is allowed for min_depth.
    DRAKE_EXPECT_NO_THROW(client.ValidDepthRangeOrThrow(0.0, 1.0));
    DRAKE_EXPECT_NO_THROW(client.ValidDepthRangeOrThrow(10.0, 1000.0));
  }

  {
    // Case 1: min_depth or max_depth are less than 0.
    const auto expected_message =
        "min_depth and max_depth must be provided for depth images, and be "
        "positive.";
    const std::vector<std::pair<double, double>> min_max{
        {-1.0, 1.0}, {1.0, -1.0}, {-1.0, -1.0}};
    for (const auto& [min, max] : min_max) {
      DRAKE_EXPECT_THROWS_MESSAGE(client.ValidDepthRangeOrThrow(min, max),
                                  expected_message);
    }
  }

  {
    // Case 2: max_depth <= min_depth.
    const auto expected_message =
        "max_depth cannot be less than or equal to min_depth.";
    DRAKE_EXPECT_THROWS_MESSAGE(client.ValidDepthRangeOrThrow(10.0, 1.0),
                                expected_message);
    DRAKE_EXPECT_THROWS_MESSAGE(client.ValidDepthRangeOrThrow(10.0, 10.0),
                                expected_message);
  }
}

GTEST_TEST(RenderClient, RenameToSceneWithExtension) {
  const std::string url{"127.0.0.1"};
  const int32_t port{8000};
  const std::string render_endpoint{"render"};
  const bool verbose = false;
  const bool no_cleanup = false;
  const RenderClient client{url, port, render_endpoint, verbose, no_cleanup};
  const fs::path temp_dir = fs::path(client.temp_directory());
  const std::string scene = temp_dir / "scene.gltf";
  std::ofstream scene_file{scene};
  scene_file << "not a real glTF scene!\n";
  scene_file.close();
  const auto unlikely = "/unlikely/to/be/a.file";

  {
    // Failure case 1: file to rename does not exist.
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenameToSceneWithExtension(scene, unlikely, ".png"),
        fmt::format("RenderClient: cannot rename '{}', file does not exist.",
                    unlikely));

    // Failure case 2: file to rename based off does not exist.
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenameToSceneWithExtension(unlikely, scene, ".tiff"),
        fmt::format(
            "RenderClient: cannot rename '{0}' to '{1}' with extension '{2}': "
            "'{1}' does not exist.",
            scene, unlikely, ".tiff"));

    // Failure case 3: destination file already exists.
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.RenameToSceneWithExtension(scene, scene, ".gltf"),
        fmt::format(
            "RenderClient: refusing to rename '{}' to '{}', file already "
            "exists!",
            scene, scene));
  }

  {
    /* Creates input scene and path for easy enumeration of extension rename
     edge cases below. */
    struct RenameResult {
      RenameResult(const std::string input_scene, std::string path,
                   const std::string ext, const std::string expected)
          : input_scene_{input_scene},
            path_{path},
            ext_{ext},
            expected_{expected} {
        std::ofstream input_scene_file{input_scene_};
        input_scene_file << "input_scene with contents.\n";
        input_scene_file.close();

        std::ofstream path_file{path_};
        path_file << "path with contents.\n";
        path_file.close();
      }

      const std::string input_scene_;
      const std::string path_;
      const std::string ext_;
      const std::string expected_;
    };

    /* NOTE: files are created, not deleted, do not reuse names.
     Additionally, do not test for no extension input_scene + ext="", this is
     already tested for above as an expected exception. */
    const fs::path sub_dir = temp_dir / "sub_directory";
    fs::create_directory(sub_dir);
    const std::vector<RenameResult> renames{
        // All components have extensions.
        {temp_dir / "input_0.gltf", temp_dir / "path_0.bin", ".png",
         temp_dir / "input_0.png"},
        // Input scene does not have a file extension.
        {temp_dir / "input_1", temp_dir / "path_1.curl", ".foo",
         temp_dir / "input_1.foo"},
        // Input path does not have an extension.
        {temp_dir / "input_2.gltf", temp_dir / "path_2", ".tiff",
         temp_dir / "input_2.tiff"},
        // Input and path do not have an extension.
        {temp_dir / "input_3", temp_dir / "path_3", ".zip",
         temp_dir / "input_3.zip"},
        // Empty extension is allowed.
        {temp_dir / "input_4.txt", temp_dir / "path_4.txt", "",
         temp_dir / "input_4"},
        // Source is in different directory.
        {temp_dir / "input_5.gltf", sub_dir / "path_5.bin", ".jpg",
         temp_dir / "input_5.jpg"}};
    for (const auto& r : renames) {
      // Before renaming, input_scene_ and path_ exist, expected_ does not.
      EXPECT_TRUE(fs::is_regular_file(r.input_scene_));
      EXPECT_TRUE(fs::is_regular_file(r.path_));
      EXPECT_FALSE(fs::is_regular_file(r.expected_));

      // Renaming the file should result in our expected value.
      const auto result =
          client.RenameToSceneWithExtension(r.input_scene_, r.path_, r.ext_);
      EXPECT_EQ(result, r.expected_);
      EXPECT_EQ(fs::path(result).extension(), r.ext_);

      // After renaming, path_ should no longer exist, expected_ should.
      EXPECT_TRUE(fs::is_regular_file(r.input_scene_));
      EXPECT_FALSE(fs::is_regular_file(r.path_));
      EXPECT_TRUE(fs::is_regular_file(r.expected_));
    }
  }

  fs::remove_all(temp_dir);
}

// LoadColorImage, LoadDepthImage, LoadLabelImage ------------------------------
// Type trait for Png image data underlying type, similar to ImageTraits.
template <int bit_depth>
struct PngTraits {
  // Compile error for unspecialized use.
};

template <>
struct PngTraits<8> {
  using ChannelType = uint8_t;
};

template <>
struct PngTraits<16> {
  using ChannelType = uint16_t;
};

/* Returns `current` value, but also increments `current` by 1.  If increasing
 by 1 would put `current` out of bounds for the numeric limits of ChannelType,
 current will be reset to 0.

 Convenience method for loop bodies so that the increase and bounds check do not
 need to be repeated in every case (e.g., red, green, blue, alpha). */
template <typename ChannelType>
ChannelType PixelValue(ChannelType* current) {
  static_assert(std::is_same_v<ChannelType, uint8_t> ||
                    std::is_same_v<ChannelType, uint16_t> ||
                    std::is_same_v<ChannelType, int16_t>,
                "Only uint8_t, uint16_t, and int16_t are supported.");
  ChannelType ret = *current;
  *current = static_cast<ChannelType>(
      (static_cast<uint32_t>(*current) + static_cast<uint32_t>(1)) %
      static_cast<uint32_t>(std::numeric_limits<ChannelType>::max()));
  return ret;
}

/* On construction, create a PNG with some arbitrary but non-zero values.  Only
 supports creating single channel, three channel (RGB), or 4 channel (RGBA) PNG
 images.  To help validate that the image coordinate system is being set
 correctly (which corner is "lower left") we create an "image" that has
 "increasing values" (modulo the maximum value for ChannelType) by pixel
 channel.  For example, with an RGB image, the pixels start as:

 - (0, 0): [kRedStart, kGreenStart, kBlueStart]
 - (0, 1): [(kRedStart+1) % 255, (kGreenStart+1) % 255, (kBlueStart+1) % 255]

 For a 16 bit gray image,

 - (0, 0): kGrayStart
 - (0, 1): (kGrayStart+1) % 65535

 This enables testing below to always know what the first pixel value should be
 at coordinate (0, 0).  The tests for image load verification rely on the
 pattern created in the constructor here, changing the pattern requires
 updating the tests as well.

 Tip: if a test image is desired to be viewed, run tests with --sandbox_debug
 so that the temporary files are not deleted.  Note that `no_cleanup` needs to
 be set to `true` for a test case, and each test usually removes the image
 with `fs::remove` so these will need to be updated to keep the image. */
template <int channels, int bit_depth>
class Png {
 public:
  static_assert(channels == 1 || channels == 3 || channels == 4,
                "Png <channels> must be 1, 3, or 4");
  static_assert(bit_depth == 8 || bit_depth == 16,
                "Png <bit_depth> must be 8 or 16");
  static_assert((channels == 1 && bit_depth == 16) ||      // 16 bit gray
                    (channels == 3 && bit_depth == 8) ||   // 8 bit RGB
                    (channels == 4 && bit_depth == 8) ||   // 8 bit RGBA
                    (channels == 3 && bit_depth == 16) ||  // 16 bit RGB
                    (channels == 1 && bit_depth == 8),     // 8 bit gray
                "Png: unsupported <channels, bit_depth> combination.");

  // Data type to allocate for memory, either uint8_t or uint16_t.
  using ChannelType = typename PngTraits<bit_depth>::ChannelType;
  static_assert(std::is_same_v<ChannelType, uint8_t> ||
                    std::is_same_v<ChannelType, uint16_t>,
                "Png: only uint8_t and uint16_t channel types are supported.");

  // For when writing a color image.
  static constexpr ChannelType kRedStart = static_cast<ChannelType>(50);
  static constexpr ChannelType kGreenStart = static_cast<ChannelType>(100);
  static constexpr ChannelType kBlueStart = static_cast<ChannelType>(150);
  static constexpr ChannelType kAlphaStart = static_cast<ChannelType>(0);

  // For when writing a single channel image.
  static constexpr ChannelType kGrayStart = static_cast<ChannelType>(0);

  Png(const std::string& path, int width, int height)
      : path_{path}, width_{width}, height_{height} {
    /* See the docs:
     http://www.libpng.org/pub/png/libpng-1.0.3-manual.html

     If something goes wrong, throw an exception to fail the test. */
    FILE* fp = fopen(path.c_str(), "wb");
    if (fp == nullptr) {
      throw std::runtime_error("Could not create FILE* for png.");
    }
    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING,
                                                  nullptr, nullptr, nullptr);
    if (png_ptr == nullptr) {
      fclose(fp);
      throw std::runtime_error("Could not create png_ptr for png.");
    }
    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
      png_destroy_write_struct(&png_ptr, static_cast<png_infopp>(nullptr));
      fclose(fp);
      throw std::runtime_error("Error creating info_ptr for png.");
    }

    // Error callback for libpng.
    if (setjmp(png_jmpbuf(png_ptr))) {
      png_destroy_write_struct(&png_ptr, &info_ptr);
      fclose(fp);
      throw std::runtime_error("Critical error trying to write png.");
    }

    // Setup and write out the header information.
    png_init_io(png_ptr, fp);
    png_set_IHDR(png_ptr, info_ptr, width_, height_, bit_depth, ColorType(),
                 PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png_ptr, info_ptr);

    /* Populate the pixel data.  See constructor comments.
     NOTE: do not use png_set_filler! */
    if (bit_depth > 8) png_set_swap(png_ptr);
    ChannelType* row_data[height_];
    ChannelType gray = static_cast<ChannelType>(kGrayStart);
    ChannelType red = static_cast<ChannelType>(kRedStart);
    ChannelType green = static_cast<ChannelType>(kGreenStart);
    ChannelType blue = static_cast<ChannelType>(kBlueStart);
    ChannelType alpha = static_cast<ChannelType>(kAlphaStart);
    /* Due to the template, and this method supporting gray, RGB, and RGBA, we
     must mark them as "unused" since depending on the template a given variable
     may not be used in the loop. */
    unused(gray);
    unused(red);
    unused(green);
    unused(blue);
    unused(alpha);
    for (int j = 0; j < height_; ++j) {
      row_data[j] = static_cast<ChannelType*>(
          png_malloc(png_ptr, png_get_rowbytes(png_ptr, info_ptr)));
      ChannelType* row_ptr = row_data[j];
      for (int i = 0; i < width_; ++i) {
        for (int k = 0; k < channels; ++k) {
          if constexpr (channels == 1) {
            *row_ptr++ = PixelValue(&gray);
          } else if constexpr (channels == 3 || channels == 4) {
            if (k == 0) {
              *row_ptr++ = PixelValue(&red);
            } else if (k == 1) {
              *row_ptr++ = PixelValue(&green);
            } else if (k == 2) {
              *row_ptr++ = PixelValue(&blue);
            } else if (k == 3) {
              *row_ptr++ = PixelValue(&alpha);
            }
          }
        }
      }
    }
    // NOTE: reinterpret_cast is needed for uint16_t data.
    png_write_rows(png_ptr, reinterpret_cast<png_bytepp>(row_data), height);

    // Finalize the png image.
    png_write_end(png_ptr, info_ptr);
    for (int j = 0; j < height_; ++j) {
      png_free(png_ptr, row_data[j]);
    }
    png_destroy_write_struct(&png_ptr, &info_ptr);
    fclose(fp);
  }

  // Return the `color_type` for `png_set_IHDR`.
  constexpr int ColorType() const {
    if constexpr (channels == 1) {
      return PNG_COLOR_TYPE_GRAY;
    } else if constexpr (channels == 3) {
      return PNG_COLOR_TYPE_RGB;
    } else {  // channels := 4
      return PNG_COLOR_TYPE_RGB_ALPHA;
    }
  }

  const std::string path_;
  const int width_;
  const int height_;
};

// Convenience class definitions for the tests below.
using PngRgb = Png<3, 8>;
using PngRgba = Png<4, 8>;
using PngGray = Png<1, 16>;
/* NOTE: these image types do not generate the same expected patterns that can
 be validated using EvaluatePng.  They are used to check that RenderClient
 loading functions reject png images that have the wrong channel width. */
using PngRgb16 = Png<3, 16>;  // LoadColorImage is 8 bit only.
using PngGray8 = Png<1, 8>;   // LoadLabelImage is 16 bit only.

/* Used to evaluate color PNG images loaded from a file match the expected
 pattern created in the constructor of the Png class.  The source_had_alpha
 template parameter distinguishes whether the test should be expecting to read
 actual alpha values (test loaded RGBA) or if the source image should have had
 its alpha padded with 255 (RGB). */
template <bool source_had_alpha>
struct EvaluatePng {
  // Convenience definitions.
  using ChannelType = typename ImageRgba8U::T;
  static constexpr ChannelType kRedStart = PngRgba::kRedStart;
  static constexpr ChannelType kGreenStart = PngRgba::kGreenStart;
  static constexpr ChannelType kBlueStart = PngRgba::kBlueStart;
  static constexpr ChannelType kAlphaStart = PngRgba::kAlphaStart;
  static constexpr ChannelType kAlphaPad = static_cast<ChannelType>(255);

  /* If the image coordinates are wrong for loading, this test will fail with
   actual test information being reported in the failure. */
  static void CornerCheck(const ImageRgba8U& image) {
    EXPECT_EQ(image.at(0, 0)[0], kRedStart);
    EXPECT_EQ(image.at(0, 0)[1], kGreenStart);
    EXPECT_EQ(image.at(0, 0)[2], kBlueStart);
    // For RGBA, use the expected value.  RGB, alpha is padded with 255.
    if constexpr (source_had_alpha) {
      EXPECT_EQ(image.at(0, 0)[3], kAlphaStart);
    } else {
      EXPECT_EQ(image.at(0, 0)[3], static_cast<ChannelType>(255));
    }
  }

  /* Tests all the pixels in the test image, but error reporting is not helpful
   in identifying what was wrong where. */
  static void FullImageCheck(const ImageRgba8U& image) {
    int n_good{0};
    const int n_expected = image.width() * image.height() * 4;
    ChannelType red = PngRgba::kRedStart;
    ChannelType green = PngRgba::kGreenStart;
    ChannelType blue = PngRgba::kBlueStart;
    ChannelType alpha = PngRgba::kAlphaStart;
    for (int j = 0; j < image.height(); ++j) {
      for (int i = 0; i < image.width(); ++i) {
        n_good += static_cast<int>(image.at(i, j)[0] == PixelValue(&red));
        n_good += static_cast<int>(image.at(i, j)[1] == PixelValue(&green));
        n_good += static_cast<int>(image.at(i, j)[2] == PixelValue(&blue));
        // For RGBA, use the expected value.  RGB, alpha is padded with 255.
        if (source_had_alpha) {
          n_good += static_cast<int>(image.at(i, j)[3] == PixelValue(&alpha));
        } else {
          n_good += static_cast<int>(image.at(i, j)[3] ==
                                     static_cast<ChannelType>(255));
        }
      }
    }
    EXPECT_EQ(n_good, n_expected);
  }
};

// Readability definitions for `source_had_alpha` template parameter.
constexpr bool SourceRgba = true;
constexpr bool SourceRgb = false;

GTEST_TEST(RenderClient, LoadColorImage) {
  const std::string url{"127.0.0.1"};
  const int32_t port{8000};
  const std::string render_endpoint{"render"};
  const bool verbose = false;
  const bool no_cleanup = true;
  const RenderClient client{url, port, render_endpoint, verbose, no_cleanup};
  const fs::path temp_dir = fs::path(client.temp_directory());

  // NOTE: keep the images small to reduce test overhead.
  constexpr int width = 222;
  constexpr int height = 111;
  ImageRgba8U drake_image{width, height};
  auto zero_drake_image = [&drake_image]() {
    for (int j = 0; j < height; ++j) {
      for (int i = 0; i < width; ++i) {
        drake_image.at(i, j)[0] = 0;  // r
        drake_image.at(i, j)[1] = 0;  // g
        drake_image.at(i, j)[2] = 0;  // b
        drake_image.at(i, j)[3] = 0;  // a
      }
    }
  };

  {
    // Failure case 1: not a valid PNG file.
    const auto expected_message = "RenderClient: cannot load '{}' as PNG.";
    const auto unlikely = "/not/likely/a.png";
    DRAKE_EXPECT_THROWS_MESSAGE(client.LoadColorImage(unlikely, &drake_image),
                                fmt::format(expected_message, unlikely));

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
    /* Failure case 2: wrong image dimensions on file.  drake_image should not
     be accessed especially when the file dimensions are bigger. */
    const std::vector<std::pair<int, int>> width_height{
        {width, height / 2}, {width + 12, height}, {width * 2, height * 2}};
    for (const auto& [w, h] : width_height) {
      const std::string path = temp_dir / fmt::format("rgb_{}_{}.png", w, h);
      PngRgb rgb{path, w, h};
      DRAKE_EXPECT_THROWS_MESSAGE(
          client.LoadColorImage(path, &drake_image),
          fmt::format("RenderClient: expected to import "
                      "\\(width={},height={}\\) from the "
                      "file '{}', but got \\(width={},height={}\\).",
                      width, height, path, w, h));
      fs::remove(path);
    }
  }

  {
    // Failure case 3: number of channels not equal to 3 or 4.
    const std::string path = temp_dir / "gray.png";
    PngGray gray{path, width, height};
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.LoadColorImage(path, &drake_image),
        fmt::format(
            "RenderClient: loaded PNG image from '{}' has 1 channel\\(s\\), "
            "but either 3 \\(RGB\\) or 4 \\(RGBA\\) are required for color "
            "images.",
            path));
    fs::remove(path);
  }

  {
    // Failure case 4: right number of channels, but 16 bit color.
    const std::string path = temp_dir / "rgb_16_bit.png";
    PngRgb16 rgb_16{path, width, height};
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.LoadColorImage(path, &drake_image),
        fmt::format(
            "RenderClient: loaded PNG image from '{}' has a channel size in "
            "bytes of 2, but only RGB and RGBA uchar \\(channel size=1\\) "
            "images are supported.",
            path));
    fs::remove(path);
  }

  {
    // Loading a three channel (RGB) png file should work as expected.
    zero_drake_image();
    const auto rgb_png_path = temp_dir / "rgb.png";
    PngRgb rgb{rgb_png_path, width, height};
    client.LoadColorImage(rgb_png_path, &drake_image);
    const auto drake_rgb_png_path = temp_dir / "drake_rgb.png";
    EvaluatePng<SourceRgb>::CornerCheck(drake_image);
    EvaluatePng<SourceRgb>::FullImageCheck(drake_image);
    // fs::remove(rgb_png_path);
  }

  {
    // Loading a four channel (RGBA) png file should work as expected.
    zero_drake_image();
    const auto rgba_png_path = temp_dir / "rgba.png";
    PngRgba rgba{rgba_png_path, width, height};
    client.LoadColorImage(rgba_png_path, &drake_image);
    const auto drake_rgba_png_path = temp_dir / "drake_rgba.png";
    EvaluatePng<SourceRgba>::CornerCheck(drake_image);
    EvaluatePng<SourceRgba>::FullImageCheck(drake_image);
    // fs::remove(rgba_png_path);
  }
}

// TODO(svenevs): LoadDepthImage

GTEST_TEST(RenderClient, LoadLabelImage) {
  const std::string url{"127.0.0.1"};
  const int32_t port{8000};
  const std::string render_endpoint{"render"};
  const bool verbose = false;
  const bool no_cleanup = true;
  const RenderClient client{url, port, render_endpoint, verbose, no_cleanup};
  const fs::path temp_dir = fs::path(client.temp_directory());

  // NOTE: keep the images small to reduce test overhead.
  constexpr int width = 222;
  constexpr int height = 111;
  ImageLabel16I drake_image{width, height};
  auto zero_drake_image = [&drake_image]() {
    for (int j = 0; j < height; ++j) {
      for (int i = 0; i < width; ++i) {
        drake_image.at(i, j)[0] = 0;
      }
    }
  };

  {
    // Failure case 1: not a valid PNG file.
    const auto expected_message = "RenderClient: cannot load '{}' as PNG.";
    const auto unlikely = "/not/likely/a.png";
    DRAKE_EXPECT_THROWS_MESSAGE(client.LoadLabelImage(unlikely, &drake_image),
                                fmt::format(expected_message, unlikely));

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
    /* Failure case 2: wrong image dimensions on file.  drake_image should not
     be accessed especially when the file dimensions are bigger. */
    const std::vector<std::pair<int, int>> width_height{
        {width, height / 2}, {width + 12, height}, {width * 2, height * 2}};
    for (const auto& [w, h] : width_height) {
      const std::string path = temp_dir / fmt::format("label_{}_{}.png", w, h);
      PngGray rgb{path, w, h};
      DRAKE_EXPECT_THROWS_MESSAGE(
          client.LoadLabelImage(path, &drake_image),
          fmt::format("RenderClient: expected to import "
                      "\\(width={},height={}\\) from the "
                      "file '{}', but got \\(width={},height={}\\).",
                      width, height, path, w, h));
      fs::remove(path);
    }
  }

  {
    // Failure case 3: number of channels not equal to 1.
    const std::string path = temp_dir / "rgb.png";
    PngRgb rgb{path, width, height};
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.LoadLabelImage(path, &drake_image),
        fmt::format(
            "RenderClient: loaded PNG image from '{}' has 3 channels, but only "
            "1 is allowed for label images.",
            path));
    fs::remove(path);
  }

  {
    // Failure case 4: right number of channels, but 16 bit color.
    const std::string path = temp_dir / "gray_8_bit.png";
    PngGray8 gray_8{path, width, height};
    DRAKE_EXPECT_THROWS_MESSAGE(
        client.LoadLabelImage(path, &drake_image),
        fmt::format(
            "RenderClient: loaded PNG image from '{}' did not have ushort "
            "data, but single channel ushort PNG is required for label images.",
            path));
    fs::remove(path);
  }

  {
    // Loading a 16 bit label image file should work as expected.
    zero_drake_image();
    const auto label_path = temp_dir / "label.png";
    PngGray label_image{label_path, width, height};
    client.LoadLabelImage(label_path, &drake_image);
    /* PngGray will generate an image that starts at 0 and increases each pixel
     by 1, so we check that the loaded image matches this pattern. */
    using ChannelType = typename ImageLabel16I::T;
    ChannelType label = static_cast<ChannelType>(PngGray::kGrayStart);
    int n_good = 0;
    const int n_expected = width * height;
    for (int j = 0; j < height; ++j) {
      for (int i = 0; i < width; ++i) {
        n_good +=
            static_cast<int>(drake_image.at(i, j)[0] == PixelValue(&label));
      }
    }
    EXPECT_EQ(n_good, n_expected);
  }
}

}  // namespace
}  // namespace internal
}  // namespace render
}  // namespace geometry
}  // namespace drake
