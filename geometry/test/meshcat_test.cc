#include "drake/geometry/meshcat.h"

#include <cstdlib>
#include <thread>

#include <drake_vendor/msgpack.hpp>
#include <fmt/format.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/meshcat_types.h"

namespace drake {
namespace geometry {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;
using ::testing::HasSubstr;

// A small wrapper around std::system to ensure correct argument passing.
int SystemCall(const std::vector<std::string>& argv) {
  std::string command;
  for (const std::string& arg : argv) {
    // Note: Can't use ASSERT_THAT inside this subroutine.
    EXPECT_THAT(arg, ::testing::Not(HasSubstr("'")));
    command += fmt::format("'{}' ", arg);
  }
  return std::system(command.c_str());
}

// Calls a python helper program to send and receive websocket messages(s)
// to/from the given Meshcat instance.
//
// @param send_json Message to send, as a json string.
// @param expect_num_messages Expected number of messages to receive.
// @param expect_json Expected content of the final message, as a json string.
// @param expect_success Whether to insist that the python helper finished and
//     the expected_json (if given) was actually received.
void CheckWebsocketCommand(
    const Meshcat& meshcat,
    std::optional<std::string> send_json,
    std::optional<int> expect_num_messages,
    std::optional<std::string> expect_json,
    bool expect_success = true) {
  std::vector<std::string> argv;
  argv.push_back(FindResourceOrThrow(
      "drake/geometry/meshcat_websocket_client"));
  // Even when this unit test is itself running under valgrind, we don't want to
  // instrument the helper process. Our valgrind configuration recognizes this
  // argument and skips instrumentation of the child process.
  argv.push_back("--disable-drake-valgrind-tracing");
  argv.push_back(fmt::format("--ws_url={}", meshcat.ws_url()));
  if (send_json) {
    DRAKE_DEMAND(!send_json->empty());
    argv.push_back(fmt::format("--send_message={}", std::move(*send_json)));
  }
  if (expect_num_messages) {
    argv.push_back(fmt::format("--expect_num_messages={}",
        *expect_num_messages));
  }
  if (expect_json) {
    DRAKE_DEMAND(!expect_json->empty());
    argv.push_back(fmt::format("--expect_message={}", std::move(*expect_json)));
  }
  argv.push_back(fmt::format("--expect_success={}",
      expect_success ? "1" : "0"));
  const int exit_code = SystemCall(argv);
  if (expect_success) {
    EXPECT_EQ(exit_code, 0);
  }
}

GTEST_TEST(MeshcatTest, TestHttp) {
  Meshcat meshcat;
  // Note: The server doesn't respect all requests; unfortunately we can't use
  // curl --head and wget --spider nor curl --range to avoid downloading the
  // full file.
  EXPECT_EQ(SystemCall({"/usr/bin/curl", "-o", "/dev/null", "--silent",
                        meshcat.web_url() + "/index.html"}),
            0);
  EXPECT_EQ(SystemCall({"/usr/bin/curl", "-o", "/dev/null", "--silent",
                        meshcat.web_url() + "/meshcat.js"}),
            0);
  EXPECT_EQ(SystemCall({"/usr/bin/curl", "-o", "/dev/null", "--silent",
                        meshcat.web_url() + "/favicon.ico"}),
            0);
  EXPECT_EQ(SystemCall({"/usr/bin/curl", "-o", "/dev/null", "--silent",
                        meshcat.web_url() + "/no-such-file"}),
            0);
}

GTEST_TEST(MeshcatTest, ConstructMultiple) {
  Meshcat meshcat;
  Meshcat meshcat2;

  EXPECT_THAT(meshcat.web_url(), HasSubstr("http://localhost:"));
  EXPECT_THAT(meshcat.ws_url(), HasSubstr("ws://localhost:"));
  EXPECT_THAT(meshcat2.web_url(), HasSubstr("http://localhost:"));
  EXPECT_THAT(meshcat2.ws_url(), HasSubstr("ws://localhost:"));
  EXPECT_NE(meshcat.web_url(), meshcat2.web_url());
}

GTEST_TEST(MeshcatTest, Ports) {
  Meshcat meshcat(7050);
  EXPECT_EQ(meshcat.port(), 7050);

  // Can't open the same port twice.
  DRAKE_EXPECT_THROWS_MESSAGE(Meshcat(7050),
                              "Meshcat failed to open a websocket port.");

  // The default constructor gets a default port.
  Meshcat m3;
  EXPECT_GE(m3.port(), 7000);
  EXPECT_LE(m3.port(), 7099);
}

// Use a basic web_url_pattern to affect web_url() and ws_url(). The pattern
// parameter only affects those URLs getters, not the server's bind behavior.
GTEST_TEST(MeshcatTest, CustomHttp) {
  const std::string pattern = "http://127.0.0.254:{port}";
  const Meshcat meshcat({"", std::nullopt, pattern});
  const std::string port = std::to_string(meshcat.port());
  EXPECT_EQ(meshcat.web_url(), "http://127.0.0.254:" + port);
  EXPECT_EQ(meshcat.ws_url(), "ws://127.0.0.254:" + port);
}

// Check a web_url_pattern that does not use any substitutions.
GTEST_TEST(MeshcatTest, CustomNoPort) {
  const std::string pattern = "http://example.ngrok.io";
  const Meshcat meshcat({"", std::nullopt, pattern});
  EXPECT_EQ(meshcat.web_url(), "http://example.ngrok.io");
  EXPECT_EQ(meshcat.ws_url(), "ws://example.ngrok.io");
}

// Check a web_url_pattern that uses https instead of http.
GTEST_TEST(MeshcatTest, CustomHttps) {
  const std::string pattern = "https://localhost:{port}";
  const Meshcat meshcat({"", std::nullopt, pattern});
  const std::string port = std::to_string(meshcat.port());
  EXPECT_EQ(meshcat.web_url(), "https://localhost:" + port);
  EXPECT_EQ(meshcat.ws_url(), "wss://localhost:" + port);
}

// Check that binding to the don't-care host "" does not crash.
// It should display as "localhost".
GTEST_TEST(MeshcatTest, CustomDefaultInterface) {
  const Meshcat meshcat({""});
  const std::string port = std::to_string(meshcat.port());
  EXPECT_EQ(meshcat.web_url(), "http://localhost:" + port);
}

// Check that binding to "*" (as mentioned in Params docs) does not crash.
// It should display as "localhost".
GTEST_TEST(MeshcatTest, CustomAllInterfaces) {
  const Meshcat meshcat({"*"});
  const std::string port = std::to_string(meshcat.port());
  EXPECT_EQ(meshcat.web_url(), "http://localhost:" + port);
}

// Check that binding to an IP does not crash.
GTEST_TEST(MeshcatTest, CustomNumericInterface) {
  const Meshcat meshcat({"127.0.0.1"});
  const std::string port = std::to_string(meshcat.port());
  EXPECT_EQ(meshcat.web_url(), "http://127.0.0.1:" + port);
}

// Check that binding to a malformed value does crash.
GTEST_TEST(MeshcatTest, BadCustomInterface) {
  DRAKE_EXPECT_THROWS_MESSAGE(Meshcat({"----"}), ".*failed to open.*");
}

GTEST_TEST(MeshcatTest, MalformedCustom) {
  // Using a non-existent substitution is detected immediately.
  DRAKE_EXPECT_THROWS_MESSAGE(
      Meshcat({"", std::nullopt, "http://localhost:{portnum}"}),
      ".*argument.*");
  // Only http or https are allowed.
  DRAKE_EXPECT_THROWS_MESSAGE(
      Meshcat({"", std::nullopt, "file:///tmp"}),
      ".*web_url_pattern.*http.*");
}

// Checks that unparsable messages are ignored.
GTEST_TEST(MeshcatTest, UnparseableMessageIgnored) {
  auto dut = std::make_unique<Meshcat>();

  // Send an unparsable message; don't expect a reply.
  const char* const message = "0";
  const bool expect_success = false;
  CheckWebsocketCommand(*dut, message, {}, {}, expect_success);

  // Pause to allow the websocket thread to run.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // The object can be destroyed with neither errors nor sanitizer leaks.
  EXPECT_NO_THROW(dut.reset());
}

// Checks that parseable messages with unknown semantics are ignored.
GTEST_TEST(MeshcatTest, UnknownEventIgnored) {
  auto dut = std::make_unique<Meshcat>();

  // Send a syntactically well-formed UserInterfaceEvent to tickle the
  // stack, but don't expect a reply.
  const char* const message = R"""({
    "type": "no_such_type",
    "name": "no_such_name"
  })""";
  const bool expect_success = false;
  CheckWebsocketCommand(*dut, message, {}, {}, expect_success);

  // Pause to allow the websocket thread to run.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // The object can be destroyed with neither errors nor sanitizer leaks.
  EXPECT_NO_THROW(dut.reset());
}

class MeshcatFaultTest : public testing::TestWithParam<int> {};

// Checks that a problem with the worker thread eventually ends up as an
// exception on the main thread.
TEST_P(MeshcatFaultTest, WorkerThreadFault) {
  const int fault_number = GetParam();

  auto dut = std::make_unique<Meshcat>();

  // Cause the websocket thread to fail.
  EXPECT_NO_THROW(dut->InjectWebsocketThreadFault(fault_number));

  // Keep checking an accessor function until the websocket fault is detected
  // and is converted into an exception on the main thread. Here we should be
  // able to call *any* function and have it report the fault; we use web_url
  // out of simplicity, and rely the impl() function in the cc file to prove
  // that every public function is preceded by a ThrowIfWebsocketThreadExited.
  auto checker = [&dut]() {
    for (int i = 0; i < 10; ++i) {
      // Send a syntactically well-formed UserInterfaceEvent to tickle the
      // stack, but don't expect a reply.
      const char* const message = R"""({
        "type": "no_such_type",
        "name": "no_such_name"
      })""";
      const bool expect_success = false;
      CheckWebsocketCommand(*dut, message, {}, {}, expect_success);

      // Poll the accessor function.
      dut->web_url();

      // Pause to allow the websocket thread to run.
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  };
  DRAKE_EXPECT_THROWS_MESSAGE(checker(), ".*thread exited.*");

  // The object can be destroyed with neither errors nor sanitizer leaks.
  EXPECT_NO_THROW(dut.reset());
}

INSTANTIATE_TEST_SUITE_P(AllFaults, MeshcatFaultTest,
    testing::Range(0, Meshcat::kMaxFaultNumber + 1));

GTEST_TEST(MeshcatTest, NumActive) {
  Meshcat meshcat;
  EXPECT_EQ(meshcat.GetNumActiveConnections(), 0);
}

// The correctness of this is established with meshcat_manual_test.  Here we
// simply aim to provide code coverage for CI (e.g., no segfaults).
GTEST_TEST(MeshcatTest, SetObjectWithShape) {
  Meshcat meshcat;
  EXPECT_TRUE(meshcat.GetPackedObject("sphere").empty());
  meshcat.SetObject("sphere", Sphere(.25), Rgba(1.0, 0, 0, 1));
  EXPECT_FALSE(meshcat.GetPackedObject("sphere").empty());
  meshcat.SetObject("cylinder", Cylinder(.25, .5), Rgba(0.0, 1.0, 0, 1));
  EXPECT_FALSE(meshcat.GetPackedObject("cylinder").empty());
  // HalfSpaces are not supported yet; this should only log a warning.
  meshcat.SetObject("halfspace", HalfSpace());
  EXPECT_TRUE(meshcat.GetPackedObject("halfspace").empty());
  meshcat.SetObject("box", Box(.25, .25, .5), Rgba(0, 0, 1, 1));
  EXPECT_FALSE(meshcat.GetPackedObject("box").empty());
  meshcat.SetObject("ellipsoid", Ellipsoid(.25, .25, .5), Rgba(1., 0, 1, 1));
  EXPECT_FALSE(meshcat.GetPackedObject("ellipsoid").empty());
  meshcat.SetObject("capsule", Capsule(.25, .5));
  EXPECT_FALSE(meshcat.GetPackedObject("capsule").empty());
  meshcat.SetObject(
      "mesh", Mesh(FindResourceOrThrow(
                       "drake/geometry/render/test/meshes/box.obj"),
                   .25));
  EXPECT_FALSE(meshcat.GetPackedObject("mesh").empty());
  meshcat.SetObject(
      "convex", Convex(FindResourceOrThrow(
                           "drake/geometry/render/test/meshes/box.obj"),
                       .25));
  EXPECT_FALSE(meshcat.GetPackedObject("convex").empty());
  // Bad filename (no extension).  Should only log a warning.
  meshcat.SetObject("bad", Mesh("test"));
  EXPECT_TRUE(meshcat.GetPackedObject("bad").empty());
  // Bad filename (file doesn't exist).  Should only log a warning.
  meshcat.SetObject("bad", Mesh("test.obj"));
  EXPECT_TRUE(meshcat.GetPackedObject("bad").empty());
}

GTEST_TEST(MeshcatTest, SetObjectWithPointCloud) {
  Meshcat meshcat;

  perception::PointCloud cloud(5);
  // clang-format off
  cloud.mutable_xyzs().transpose() <<
    1, 2, 3,
    10, 20, 30,
    100, 200, 300,
    4, 5, 6,
    40, 50, 60;
  // clang-format on
  meshcat.SetObject("cloud", cloud);
  EXPECT_FALSE(meshcat.GetPackedObject("cloud").empty());

  perception::PointCloud rgb_cloud(
      5, perception::pc_flags::kXYZs | perception::pc_flags::kRGBs);
  rgb_cloud.mutable_xyzs() = cloud.xyzs();
  // clang-format off
  rgb_cloud.mutable_rgbs() <<
    1, 2, 3,
    10, 20, 30,
    100, 200, 255,
    4, 5, 6,
    40, 50, 60;
  // clang-format on
  meshcat.SetObject("rgb_cloud", rgb_cloud);
  EXPECT_FALSE(meshcat.GetPackedObject("rgb_cloud").empty());
}

GTEST_TEST(MeshcatTest, SetObjectWithTriangleSurfaceMesh) {
  Meshcat meshcat;

  const int face_data[2][3] = {{0, 1, 2}, {2, 3, 0}};
  std::vector<SurfaceTriangle> faces;
  for (int f = 0; f < 2; ++f) faces.emplace_back(face_data[f]);
  const Eigen::Vector3d vertex_data[4] = {
      {0, 0, 0}, {0.5, 0, 0}, {0.5, 0.5, 0}, {0, 0.5, 0.5}};
  std::vector<Eigen::Vector3d> vertices;
  for (int v = 0; v < 4; ++v) vertices.emplace_back(vertex_data[v]);
  TriangleSurfaceMesh<double> surface_mesh(
      std::move(faces), std::move(vertices));
  meshcat.SetObject("triangle_mesh", surface_mesh, Rgba(.9, 0, .9, 1.0));
  EXPECT_FALSE(meshcat.GetPackedObject("triangle_mesh").empty());

  meshcat.SetObject("triangle_mesh_wireframe", surface_mesh,
                    Rgba(.9, 0, .9, 1.0), true, 5.0);
  EXPECT_FALSE(meshcat.GetPackedObject("triangle_mesh_wireframe").empty());
}

GTEST_TEST(MeshcatTest, SetLine) {
  Meshcat meshcat;

  Eigen::Matrix3Xd vertices(3, 200);
  Eigen::RowVectorXd t = Eigen::RowVectorXd::LinSpaced(200, 0, 10 * M_PI);
  vertices << .25 * t.array().sin(), .25 * t.array().cos(), t / (10 * M_PI);
  meshcat.SetLine("line", vertices, 3.0, Rgba(0, 0, 1, 1));
  EXPECT_FALSE(meshcat.GetPackedObject("line").empty());

  Eigen::Matrix3Xd start(3, 4), end(3, 4);
  // clang-format off
  start << -.1, -.1,  .1, .1,
           -.1,  .1, -.1, .1,
           0, 0, 0, 0;
  // clang-format on
  end = start;
  end.row(2) = Eigen::RowVector4d::Ones();
  meshcat.SetLineSegments("line_segments", start, end, 5.0, Rgba(0, 1, 0, 1));
  EXPECT_FALSE(meshcat.GetPackedObject("line_segments").empty());

  // Throws if start.cols() != end.cols().
  EXPECT_THROW(
      meshcat.SetLineSegments("bad_segments", Eigen::Matrix3Xd::Identity(3, 4),
                              Eigen::Matrix3Xd::Identity(3, 3)),
      std::exception);
}

GTEST_TEST(MeshcatTest, SetTriangleMesh) {
  Meshcat meshcat;

  // Populate the vertices/faces transposed, for easier Eigen initialization.
  Eigen::MatrixXd vertices(4, 3);
  Eigen::MatrixXi faces(2, 3);
  // clang-format off
  vertices << 0, 0, 0,
              1, 0, 0,
              1, 0, 1,
              0, 0, 1;
  faces << 0, 1, 2,
           3, 0, 2;
  // clang-format on

  meshcat.SetTriangleMesh("triangle_mesh", vertices.transpose(),
                         faces.transpose(), Rgba(1, 0, 0, 1), true, 5.0);
  EXPECT_FALSE(meshcat.GetPackedObject("triangle_mesh").empty());
}

GTEST_TEST(MeshcatTest, SetTransform) {
  Meshcat meshcat;
  EXPECT_FALSE(meshcat.HasPath("frame"));
  EXPECT_TRUE(meshcat.GetPackedTransform("frame").empty());
  const RigidTransformd X_ParentPath{math::RollPitchYawd(.5, .26, -3),
                                     Vector3d{.9, -2., .12}};
  meshcat.SetTransform("frame", X_ParentPath);

  std::string transform = meshcat.GetPackedTransform("frame");
  msgpack::object_handle oh =
      msgpack::unpack(transform.data(), transform.size());
  auto data = oh.get().as<internal::SetTransformData>();
  EXPECT_EQ(data.type, "set_transform");
  EXPECT_EQ(data.path, "/drake/frame");
  Eigen::Map<Eigen::Matrix4d> matrix(data.matrix);
  EXPECT_TRUE(CompareMatrices(matrix, X_ParentPath.GetAsMatrix4()));
}

GTEST_TEST(MeshcatTest, SetTransformWithMatrix) {
  Meshcat meshcat;
  EXPECT_FALSE(meshcat.HasPath("frame"));
  EXPECT_TRUE(meshcat.GetPackedTransform("frame").empty());
  Eigen::Matrix4d matrix;
  // clang-format off
  matrix <<  1,  2,  3,  4,
             5,  6,  7,  8,
            -1, -2, -3, -4,
            -5, -6, -7, -8;
  // clang-format on
  meshcat.SetTransform("frame", matrix);

  std::string transform = meshcat.GetPackedTransform("frame");
  msgpack::object_handle oh =
      msgpack::unpack(transform.data(), transform.size());
  auto data = oh.get().as<internal::SetTransformData>();
  EXPECT_EQ(data.type, "set_transform");
  EXPECT_EQ(data.path, "/drake/frame");
  Eigen::Map<Eigen::Matrix4d> actual(data.matrix);
  EXPECT_TRUE(CompareMatrices(matrix, actual));
}

GTEST_TEST(MeshcatTest, Delete) {
  Meshcat meshcat;
  // Ok to delete an empty tree.
  meshcat.Delete();
  EXPECT_FALSE(meshcat.HasPath(""));
  EXPECT_FALSE(meshcat.HasPath("frame"));
  meshcat.SetTransform("frame", RigidTransformd{});
  EXPECT_TRUE(meshcat.HasPath(""));
  EXPECT_TRUE(meshcat.HasPath("frame"));
  EXPECT_TRUE(meshcat.HasPath("/drake/frame"));
  // Deleting a random string does nothing.
  meshcat.Delete("bad");
  EXPECT_TRUE(meshcat.HasPath("frame"));
  meshcat.Delete("frame");
  EXPECT_FALSE(meshcat.HasPath("frame"));

  // Deleting a parent directory deletes all children.
  meshcat.SetTransform("test/frame", RigidTransformd{});
  meshcat.SetTransform("test/frame2", RigidTransformd{});
  meshcat.SetTransform("test/another/frame", RigidTransformd{});
  EXPECT_TRUE(meshcat.HasPath("test/frame"));
  EXPECT_TRUE(meshcat.HasPath("test/frame2"));
  EXPECT_TRUE(meshcat.HasPath("test/another/frame"));
  meshcat.Delete("test");
  EXPECT_FALSE(meshcat.HasPath("test/frame"));
  EXPECT_FALSE(meshcat.HasPath("test/frame2"));
  EXPECT_FALSE(meshcat.HasPath("test/another/frame"));
  EXPECT_TRUE(meshcat.HasPath("/drake"));

  // Deleting the empty string deletes the prefix.
  meshcat.SetTransform("test/frame", RigidTransformd{});
  meshcat.SetTransform("test/frame2", RigidTransformd{});
  meshcat.SetTransform("test/another/frame", RigidTransformd{});
  EXPECT_TRUE(meshcat.HasPath("test/frame"));
  EXPECT_TRUE(meshcat.HasPath("test/frame2"));
  EXPECT_TRUE(meshcat.HasPath("test/another/frame"));
  meshcat.Delete();
  EXPECT_FALSE(meshcat.HasPath("test/frame"));
  EXPECT_FALSE(meshcat.HasPath("test/frame2"));
  EXPECT_FALSE(meshcat.HasPath("test/another/frame"));
  EXPECT_FALSE(meshcat.HasPath("/drake"));
}

// Tests three methods of SceneTreeElement:
// - SceneTreeElement::operator[]() is used in Meshcat::Set*().  We'll use
// SetTransform() here.
// - SceneTreeElement::Find() is used in Meshcat::HasPath() and
// Meshcat::GetPacked*().  We'll use HasPath() to test.
// - SceneTreeElement::Delete() is used in Meshat::Delete().
// All of them also run through WebSocketPublisher::FullPath().
GTEST_TEST(MeshcatTest, Paths) {
  Meshcat meshcat;
  // Absolute paths.
  meshcat.SetTransform("/foo/frame", RigidTransformd{});
  EXPECT_TRUE(meshcat.HasPath("/foo/frame"));
  meshcat.Delete("/foo/frame");
  EXPECT_FALSE(meshcat.HasPath("/foo/frame"));

  // Absolute paths with strange spellings.
  meshcat.SetTransform("///bar///frame///", RigidTransformd{});
  EXPECT_TRUE(meshcat.HasPath("//bar//frame//"));
  EXPECT_TRUE(meshcat.HasPath("/bar/frame"));
  meshcat.Delete("////bar//frame///");
  EXPECT_FALSE(meshcat.HasPath("/bar/frame"));

  // Relative paths.
  meshcat.SetTransform("frame", RigidTransformd{});
  EXPECT_TRUE(meshcat.HasPath("frame"));
  EXPECT_TRUE(meshcat.HasPath("/drake/frame"));

  // Relative paths with strange spellings.
  meshcat.SetTransform("bar///frame///", RigidTransformd{});
  EXPECT_TRUE(meshcat.HasPath("bar//frame//"));
  EXPECT_TRUE(meshcat.HasPath("/drake/bar/frame"));
  meshcat.Delete("bar//frame//");
  EXPECT_FALSE(meshcat.HasPath("bar/frame"));
  EXPECT_FALSE(meshcat.HasPath("/drake/bar/frame"));
}

GTEST_TEST(MeshcatTest, SetPropertyBool) {
  Meshcat meshcat;
  EXPECT_FALSE(meshcat.HasPath("/Grid"));
  EXPECT_TRUE(meshcat.GetPackedProperty("/Grid", "visible").empty());
  meshcat.SetProperty("/Grid", "visible", false);
  EXPECT_TRUE(meshcat.HasPath("/Grid"));

  std::string property = meshcat.GetPackedProperty("/Grid", "visible");
  msgpack::object_handle oh = msgpack::unpack(property.data(), property.size());
  auto data = oh.get().as<internal::SetPropertyData<bool>>();
  EXPECT_EQ(data.type, "set_property");
  EXPECT_EQ(data.path, "/Grid");
  EXPECT_EQ(data.property, "visible");
  EXPECT_FALSE(data.value);
}

GTEST_TEST(MeshcatTest, SetPropertyDouble) {
  Meshcat meshcat;
  EXPECT_FALSE(meshcat.HasPath("/Cameras/default/rotated/<object>"));
  EXPECT_TRUE(
      meshcat.GetPackedProperty("/Cameras/default/rotated/<object>", "zoom")
          .empty());
  meshcat.SetProperty("/Cameras/default/rotated/<object>", "zoom", 2.0);
  EXPECT_TRUE(meshcat.HasPath("/Cameras/default/rotated/<object>"));

  std::string property =
      meshcat.GetPackedProperty("/Cameras/default/rotated/<object>", "zoom");
  msgpack::object_handle oh = msgpack::unpack(property.data(), property.size());
  auto data = oh.get().as<internal::SetPropertyData<double>>();
  EXPECT_EQ(data.type, "set_property");
  EXPECT_EQ(data.path, "/Cameras/default/rotated/<object>");
  EXPECT_EQ(data.property, "zoom");
  EXPECT_EQ(data.value, 2.0);
}

GTEST_TEST(MeshcatTest, Buttons) {
  Meshcat meshcat;

  // Asking for clicks prior to adding is an error.
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.GetButtonClicks("alice"),
      "Meshcat does not have any button named alice.");

  // A new button starts out unclicked.
  meshcat.AddButton("alice");
  EXPECT_EQ(meshcat.GetButtonClicks("alice"), 0);

  // Clicking the button increases the count.
  CheckWebsocketCommand(meshcat, R"""({
      "type": "button",
      "name": "alice"
    })""", {}, {});
  EXPECT_EQ(meshcat.GetButtonClicks("alice"), 1);

  // Adding using an existing button name resets its count.
  meshcat.AddButton("alice");
  EXPECT_EQ(meshcat.GetButtonClicks("alice"), 0);

  // Clicking the button increases the count again.
  CheckWebsocketCommand(meshcat, R"""({
      "type": "button",
      "name": "alice"
    })""", {}, {});
  EXPECT_EQ(meshcat.GetButtonClicks("alice"), 1);

  // Removing the button then asking for clicks is an error.
  meshcat.DeleteButton("alice");
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.GetButtonClicks("alice"),
      "Meshcat does not have any button named alice.");

  // Removing a non-existent button is an error.
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.DeleteButton("alice"),
      "Meshcat does not have any button named alice.");

  // Adding the button anew starts with a zero count again.
  meshcat.AddButton("alice");
  EXPECT_EQ(meshcat.GetButtonClicks("alice"), 0);

  // Buttons are removed when deleting all controls.
  meshcat.AddButton("bob");
  meshcat.DeleteAddedControls();
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.GetButtonClicks("alice"),
      "Meshcat does not have any button named alice.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.GetButtonClicks("bob"),
      "Meshcat does not have any button named bob.");

  // Adding a button with the keycode.
  meshcat.AddButton("alice", "KeyT");
  CheckWebsocketCommand(meshcat, R"""({
      "type": "button",
      "name": "alice"
    })""", {}, {});
  EXPECT_EQ(meshcat.GetButtonClicks("alice"), 1);
  // Adding with the same keycode still resets.
  meshcat.AddButton("alice", "KeyT");
  EXPECT_EQ(meshcat.GetButtonClicks("alice"), 0);
  // Adding the same button with an empty keycode throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.AddButton("alice"),
      ".*does not match the current keycode.*");
  // Adding the same button with a different keycode throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.AddButton("alice", "KeyR"),
      ".*does not match the current keycode.*");
  meshcat.DeleteButton("alice");

  // Adding a button with the keycode empty, then populated works.
  meshcat.AddButton("alice");
  meshcat.AddButton("alice", "KeyT");
}

GTEST_TEST(MeshcatTest, Sliders) {
  Meshcat meshcat;

  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.GetSliderValue("slider"),
      "Meshcat does not have any slider named slider.");

  meshcat.AddSlider("slider", 0.2, 1.5, 0.1, 0.5);
  EXPECT_NEAR(meshcat.GetSliderValue("slider"), 0.5, 1e-14);
  meshcat.SetSliderValue("slider", 0.7);
  EXPECT_NEAR(meshcat.GetSliderValue("slider"), 0.7, 1e-14);
  meshcat.SetSliderValue("slider", -2.0);
  EXPECT_NEAR(meshcat.GetSliderValue("slider"), .2, 1e-14);
  meshcat.SetSliderValue("slider", 2.0);
  EXPECT_NEAR(meshcat.GetSliderValue("slider"), 1.5, 1e-14);
  meshcat.SetSliderValue("slider", 1.245);
  EXPECT_NEAR(meshcat.GetSliderValue("slider"), 1.2, 1e-14);
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.AddSlider("slider", 0.2, 1.5, 0.1, 0.5),
      "Meshcat already has a slider named slider.");

  meshcat.DeleteSlider("slider");

  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.GetSliderValue("slider"),
      "Meshcat does not have any slider named slider.");

  meshcat.AddSlider("slider1", 2, 3, 0.01, 2.35);
  meshcat.AddSlider("slider2", 4, 5, 0.01, 4.56);

  auto slider_names = meshcat.GetSliderNames();
  EXPECT_EQ(slider_names.size(), 2);
  EXPECT_EQ(slider_names[0], "slider1");
  EXPECT_EQ(slider_names[1], "slider2");

  meshcat.DeleteAddedControls();
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.GetSliderValue("slider1"),
      "Meshcat does not have any slider named slider1.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.GetSliderValue("slider2"),
      "Meshcat does not have any slider named slider2.");

  slider_names = meshcat.GetSliderNames();
  EXPECT_EQ(slider_names.size(), 0);
}

GTEST_TEST(MeshcatTest, DuplicateMixedControls) {
  Meshcat meshcat;

  meshcat.AddButton("button");
  meshcat.AddSlider("slider", 0.2, 1.5, 0.1, 0.5);

  // We cannot use AddButton nor AddSlider to change the type of an existing
  // control by attempting to re-use its name.
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.AddButton("slider"),
      "Meshcat already has a slider named slider.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.AddButton("slider", "KeyR"),
      "Meshcat already has a slider named slider.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.AddSlider("button", 0.2, 1.5, 0.1, 0.5),
      "Meshcat already has a button named button.");
}

// Properly testing Meshcat's limited support for gamepads requires human
// input, and is done in meshcat_manual_test. This test simply ensures the
// entry point forwards along the Javascript messages.
GTEST_TEST(MeshcatTest, Gamepad) {
  Meshcat meshcat;

  Meshcat::Gamepad gamepad = meshcat.GetGamepad();
  // Check the default status assuming no messages have been received:
  EXPECT_FALSE(gamepad.index);
  EXPECT_TRUE(gamepad.button_values.empty());
  EXPECT_TRUE(gamepad.axes.empty());

  // Clicking the button increases the count.
  CheckWebsocketCommand(meshcat, R"""({
      "type": "gamepad",
      "name": "",
      "gamepad": {
        "index": 1, 
        "button_values": [0, 0.5],
        "axes": [0.1, 0.2, 0.3, 0.4]
      }
    })""", {}, {});

  gamepad = meshcat.GetGamepad();
  EXPECT_TRUE(gamepad.index);
  EXPECT_EQ(gamepad.index, 1);
  std::vector<double> expected_button_values{0, 0.5};
  std::vector<double> expected_axes{0.1, 0.2, 0.3, 0.4};
  EXPECT_EQ(gamepad.button_values, expected_button_values);
  EXPECT_EQ(gamepad.axes, expected_axes);
}

GTEST_TEST(MeshcatTest, SetPropertyWebSocket) {
  Meshcat meshcat;
  meshcat.SetProperty("/Background", "visible", false);
  CheckWebsocketCommand(meshcat, {}, 1, R"""({
      "type": "set_property",
      "path": "/Background",
      "property": "visible",
      "value": false
    })""");
  meshcat.SetProperty("/Grid", "visible", false);
  // Note: The order of the messages is due to "/Background" < "/Grid" in the
  // std::map, not due to the order that SetProperty was called.
  CheckWebsocketCommand(meshcat, {}, 1, R"""({
      "type": "set_property",
      "path": "/Background",
      "property": "visible",
      "value": false
    })""");
  CheckWebsocketCommand(meshcat, {}, 2, R"""({
      "type": "set_property",
      "path": "/Grid",
      "property": "visible",
      "value": false
    })""");

  // Confirm that meshcat.Flush() doesn't crash even when we've had multiple
  // clients connect, received data, and disconnect.
  meshcat.Flush();
}

GTEST_TEST(MeshcatTest, SetPerspectiveCamera) {
  Meshcat meshcat;
  Meshcat::PerspectiveCamera perspective;
  perspective.fov = 82;
  perspective.aspect = 1.5;
  meshcat.SetCamera(perspective, "/my/camera");
  CheckWebsocketCommand(meshcat, {}, 1, R"""({
      "type": "set_object",
      "path": "/my/camera",
      "object": {
        "object": {
          "type": "PerspectiveCamera",
          "fov": 82.0,
          "aspect": 1.5,
          "near": 0.01,
          "far": 100,
          "zoom": 1.0
        }
      }
    })""");
}

GTEST_TEST(MeshcatTest, SetOrthographicCamera) {
  Meshcat meshcat;
  Meshcat::OrthographicCamera ortho;
  ortho.left = -1.23;
  ortho.bottom = .84;
  meshcat.SetCamera(ortho, "/my/camera");
  CheckWebsocketCommand(meshcat, {}, 1, R"""({
      "type": "set_object",
      "path": "/my/camera",
      "object": {
        "object": {
          "type": "OrthographicCamera",
          "left": -1.23,
          "right": 1.0,
          "top": -1.0,
          "bottom": 0.84,
          "near": -1000.0,
          "far": 1000.0,
          "zoom": 1.0
        }
      }
    })""");
}

GTEST_TEST(MeshcatTest, SetAnimation) {
  Meshcat meshcat;
  MeshcatAnimation animation;

  animation.SetTransform(0, "sphere", RigidTransformd(Vector3d{0, 0, 0}));
  animation.SetTransform(20, "sphere", RigidTransformd(Vector3d{0, 0, 1}));
  animation.SetTransform(40, "sphere", RigidTransformd(Vector3d{0, 0, 0}));

  animation.SetProperty(0, "cylinder", "visible", true);
  animation.SetProperty(20, "cylinder", "visible", false);
  animation.SetProperty(40, "cylinder", "visible", true);

  animation.SetProperty(0, "ellipsoid/<object>", "material.opacity", 0.0);
  animation.SetProperty(20, "ellipsoid/<object>", "material.opacity", 1.0);
  animation.SetProperty(40, "ellipsoid/<object>", "material.opacity", 0.0);

  animation.set_loop_mode(MeshcatAnimation::kLoopRepeat);
  animation.set_repetitions(4);
  animation.set_autoplay(true);
  animation.set_clamp_when_finished(true);

  meshcat.SetAnimation(animation);

  // The animations will be in lexographical order by path since we're using a
  // std::map with the path strings as the (sorted) keys.
  CheckWebsocketCommand(meshcat, {}, 1, R"""({
      "type": "set_animation",
      "animations": [{
          "path": "/drake/cylinder",
          "clip": {
              "fps": 32.0,
              "name": "default",
              "tracks": [{
                  "name": ".visible",
                  "type": "boolean",
                  "keys": [{
                      "time": 0,
                      "value": true
                    },{
                      "time": 20,
                      "value": false
                    },{
                      "time": 40,
                      "value": true
                  }]
              }]
          }
      }, {
          "path": "/drake/ellipsoid/<object>",
          "clip": {
              "fps": 32.0,
              "name": "default",
              "tracks": [{
                  "name": ".material.opacity",
                  "type": "number",
                  "keys": [{
                      "time": 0,
                      "value": 0.0
                    },{
                      "time": 20,
                      "value": 1.0
                  },{
                      "time": 40,
                      "value": 0.0
                  }]
              }]
          }
      }, {
          "path": "/drake/sphere",
          "clip": {
              "fps": 32.0,
              "name": "default",
              "tracks": [{
                  "name": ".position",
                  "type": "vector3",
                  "keys": [{
                      "time": 0,
                      "value": [0.0, 0.0, 0.0]
                    },{
                      "time": 20,
                      "value": [0.0, 0.0, 1.0]
                    },{
                      "time": 40,
                      "value": [0.0, 0.0, 0.0]
                  }]
              }, {
                  "name": ".quaternion",
                  "type": "quaternion",
                  "keys": [{
                      "time": 0,
                      "value": [0.0, 0.0, 0.0, 1.0]
                    },{
                      "time": 20,
                      "value": [0.0, 0.0, 0.0, 1.0]
                    },{
                      "time": 40,
                      "value": [0.0, 0.0, 0.0, 1.0]
                  }]
              }]
          }
      }],
      "options": {
          "play": true,
          "loopMode": 2201,
          "repetitions": 4,
          "clampWhenFinished": true
      }
  })""");
}

GTEST_TEST(MeshcatTest, Set2dRenderMode) {
  Meshcat meshcat;
  meshcat.Set2dRenderMode();
  // We simply confirm that all of the objects have been set, and use
  // meshcat_manual_test to check that the visualizer updates as we expect.
  EXPECT_FALSE(
      meshcat.GetPackedObject("/Cameras/default/rotated").empty());
  EXPECT_FALSE(meshcat.GetPackedTransform("/Cameras/default").empty());
  EXPECT_FALSE(
      meshcat.GetPackedProperty("/Cameras/default/rotated/<object>", "position")
          .empty());
  EXPECT_FALSE(meshcat.GetPackedProperty("/Background", "visible").empty());
  EXPECT_FALSE(meshcat.GetPackedProperty("/Grid", "visible").empty());
  EXPECT_FALSE(meshcat.GetPackedProperty("/Axes", "visible").empty());
}

GTEST_TEST(MeshcatTest, ResetRenderMode) {
  Meshcat meshcat;
  meshcat.ResetRenderMode();
  // We simply confirm that all of the objects have been set, and use
  // meshcat_manual_test to check that the visualizer updates as we expect.
  EXPECT_FALSE(
      meshcat.GetPackedObject("/Cameras/default/rotated").empty());
  EXPECT_FALSE(meshcat.GetPackedTransform("/Cameras/default").empty());
  EXPECT_FALSE(
      meshcat.GetPackedProperty("/Cameras/default/rotated/<object>", "position")
          .empty());
  EXPECT_FALSE(meshcat.GetPackedProperty("/Background", "visible").empty());
  EXPECT_FALSE(meshcat.GetPackedProperty("/Grid", "visible").empty());
  EXPECT_FALSE(meshcat.GetPackedProperty("/Axes", "visible").empty());
}

GTEST_TEST(MeshcatTest, StaticHtml) {
  Meshcat meshcat;

  // Call each command that will be saved (at least) once.
  meshcat.SetObject("box", Box(.25, .25, .5), Rgba(0, 0, 1, 1));
  meshcat.SetTransform("box", RigidTransformd(Vector3d{0, 0, 0}));
  meshcat.SetProperty("/Background", "visible", false);

  MeshcatAnimation animation;
  animation.SetTransform(0, "box", RigidTransformd());
  animation.SetTransform(20, "box",
                         RigidTransformd(RotationMatrixd::MakeZRotation(M_PI)));

  const std::string html = meshcat.StaticHtml();

  // Confirm that the js source links were replaced.
  EXPECT_THAT(html, ::testing::Not(HasSubstr("meshcat.js")));
  EXPECT_THAT(html, ::testing::Not(HasSubstr("stats.min.js")));
  // The static html replaces the javascript web socket connection code with
  // direct invocation of MeshCat with all of the data. We'll confirm that
  // this appears to have happened by testing for the presence of the injected
  // tree (base64 content) and the absence of what is *believed* to be the
  // delimiting text of the connection block.
  EXPECT_THAT(html, HasSubstr("data:application/octet-binary;base64"));
  EXPECT_THAT(html, ::testing::Not(HasSubstr("CONNECTION BLOCK")));
}

// Check MeshcatParams.hide_stats_plot sends a hide_realtime_rate message
GTEST_TEST(MeshcatTest, RealtimeRatePlot) {
  MeshcatParams params;
  params.show_stats_plot = true;
  Meshcat meshcat(params);
  CheckWebsocketCommand(meshcat, {}, 1, R"""({
      "type": "show_realtime_rate",
      "show": true
    })""");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
