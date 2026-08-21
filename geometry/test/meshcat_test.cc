#include "drake/geometry/meshcat.h"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <msgpack.hpp>

#include "drake/common/find_resource.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/timer.h"
#include "drake/geometry/meshcat_types_internal.h"

namespace drake {
namespace geometry {
namespace {

namespace fs = std::filesystem;

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;
using testing::ElementsAre;
using ::testing::HasSubstr;

// N.B. the bindings/pydrake/geometry/test/visualizers_test.py covers testing
// of our Meshcat http operations. It's too awkward to try to do that here.

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
void CheckWebsocketCommand(const Meshcat& meshcat,
                           std::optional<std::string> send_json,
                           std::optional<int> expect_num_messages,
                           std::optional<std::string> expect_json,
                           bool expect_success = true) {
  if (expect_num_messages) {
    meshcat.Flush();
  }
  std::vector<std::string> argv;
  argv.push_back(
      FindResourceOrThrow("drake/geometry/meshcat_websocket_client"));
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
    argv.push_back(
        fmt::format("--expect_num_messages={}", *expect_num_messages));
  }
  if (expect_json) {
    DRAKE_DEMAND(!expect_json->empty());
    argv.push_back(fmt::format("--expect_message={}", std::move(*expect_json)));
  }
  argv.push_back(
      fmt::format("--expect_success={}", expect_success ? "1" : "0"));
  const int exit_code = SystemCall(argv);
  if (expect_success) {
    EXPECT_EQ(exit_code, 0);
  }
}

/* Decodes the most recent SetTransform message for Meshcat on the given path
and returns the message's decoded path and transform data. If no message has
ever been sent, returns a default-constructed value. */
std::pair<std::string, Eigen::Matrix4d> GetDecodedTransform(
    const Meshcat& meshcat, std::string_view path) {
  std::string message = meshcat.GetPackedTransform(path);
  if (message.empty()) {
    return {};
  }

  msgpack::object_handle oh = msgpack::unpack(message.data(), message.size());
  internal::SetTransformData decoded;
  EXPECT_NO_THROW(decoded = oh.get().as<internal::SetTransformData>());
  EXPECT_EQ(decoded.type, "set_transform");
  Eigen::Map<Eigen::Matrix4d> matrix(decoded.matrix);
  return {decoded.path, Eigen::Matrix4d{matrix}};
}

/* Decodes the most recent SetProperty message for Meshcat on the given path
and returns the message's decoded path and property value. If no message has
ever been sent, returns a default-constructed value. */
template <typename T>
std::pair<std::string, T> GetDecodedProperty(const Meshcat& meshcat,
                                             std::string_view path,
                                             std::string_view property) {
  std::string message = meshcat.GetPackedProperty(path, std::string{property});
  if (message.empty()) {
    return {};
  }
  msgpack::object_handle oh = msgpack::unpack(message.data(), message.size());
  internal::SetPropertyData<T> decoded;
  EXPECT_NO_THROW(decoded = oh.get().as<internal::SetPropertyData<T>>());
  EXPECT_EQ(decoded.type, "set_property");
  EXPECT_EQ(decoded.property, property);
  return {decoded.path, decoded.value};
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

// We'll assume that nothing else on the machine is using this port.
// This will have false positives if something else was using the port.
GTEST_TEST(MeshcatTest, HardCodedPort) {
  // Use a port greater than 8000, to make sure the user can step outside our
  // default range of [7000, 7999].
  const int specific_port = 8422;
  Meshcat meshcat(specific_port);
  EXPECT_EQ(meshcat.port(), specific_port);
}

GTEST_TEST(MeshcatTest, DefaultPort) {
  // The default constructor gets a default port.
  Meshcat meshcat;
  const int port = meshcat.port();
  EXPECT_GE(port, 7000);
  EXPECT_LE(port, 7999);

  // Can't open the same port twice.
  DRAKE_EXPECT_THROWS_MESSAGE(Meshcat(port),
                              "Meshcat failed to open a websocket port.");
}

GTEST_TEST(MeshcatTest, EphemeralPort) {
  // Use port 0 to choose an ephemeral port:
  //  https://en.wikipedia.org/wiki/Ephemeral_port
  Meshcat meshcat(0);
  EXPECT_GE(meshcat.port(), 32768);

  // Try clicking a button to make sure the number was correct. This also serves
  // as an end-to-end test of button handling over websockets.
  meshcat.AddButton("button");
  CheckWebsocketCommand(meshcat, R"""({
      "type": "button",
      "name": "button"
    })""",
                        {}, {});
  EXPECT_EQ(meshcat.GetButtonClicks("button"), 1);
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
  DRAKE_EXPECT_THROWS_MESSAGE(Meshcat({"", std::nullopt, "file:///tmp"}),
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

// Simple utility struct that will allow us to determine the msgpack encoding
// of an arbitrary 4x4 transform matrix (see e.g., MeshData.matrix or
// MeshfileObjectData.matrix). The default value is the zero matrix.
struct JustMatrix {
  double matrix[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  MSGPACK_DEFINE_MAP(matrix);
};

// Note: The Mesh shape is special. It can reference on-disk or in-memory data
// and they all need to be handled. This test runs through the various
// permutations.
GTEST_TEST(MeshcatTest, SetObjectWithMesh) {
  Meshcat meshcat;

  using testing::HasSubstr;
  using testing::Not;

  // Apply an arbitrary, non-uniform scale so we can detect its inclusion.
  const Vector3d non_uniform_scale{2, 3, 4};
  // A .obj file with material library and image. The packed message should
  // encode the .obj, the .mtl file, and the image.
  const fs::path obj_path =
      FindResourceOrThrow("drake/geometry/render/test/meshes/rainbow_box.obj");
  const Mesh disk_obj(obj_path, non_uniform_scale);
  const auto obj_file = MemoryFile::Make(obj_path);
  const auto mtl_file = MemoryFile::Make(
      FindResourceOrThrow("drake/geometry/render/test/meshes/rainbow_box.mtl"));
  const auto png_file = MemoryFile::Make(FindResourceOrThrow(
      "drake/geometry/render/test/meshes/rainbow_stripes.png"));
  const Mesh memory_obj(
      InMemoryMesh{MemoryFile(obj_file.contents(), obj_file.extension(),
                              "a hint; *not* a path"),
                   {{"rainbow_box.mtl", MemoryFile(mtl_file)},
                    {"rainbow_stripes.png", MemoryFile(png_file)}}},
      non_uniform_scale);
  // The "hetero" objs mix up the supporting files so that, in turn, one is
  // in memory, and one is on-disk.
  const Mesh hetero_obj1(
      InMemoryMesh{
          MemoryFile(obj_file.contents(), obj_file.extension(), "hetero1_obj"),
          {{"rainbow_box.mtl", fs::path(mtl_file.filename_hint())},
           {"rainbow_stripes.png", MemoryFile(png_file)}}},
      non_uniform_scale);
  const Mesh hetero_obj2(
      InMemoryMesh{
          MemoryFile(obj_file.contents(), obj_file.extension(), "hetero2_obj"),
          {{"rainbow_box.mtl", MemoryFile(mtl_file)},
           {"rainbow_stripes.png", fs::path(png_file.filename_hint())}}},
      non_uniform_scale);

  // Get the msgpack encoding of the matrix field (leaving out all prefixes).
  // We'll match the string with the contents below.
  const std::string matrix_string = [&non_uniform_scale]() {
    JustMatrix data;
    data.matrix[0] = non_uniform_scale.x();
    data.matrix[5] = non_uniform_scale.y();
    data.matrix[10] = non_uniform_scale.z();
    data.matrix[15] = 1;  // Homogeneous matrix.
    std::stringstream matrix_stream;
    msgpack::pack(matrix_stream, data);
    const std::string full_string = matrix_stream.str();
    return full_string.substr(full_string.find("matrix"));
  }();
  DRAKE_DEMAND(!matrix_string.empty());

  for (const auto* mesh_ptr :
       {&disk_obj, &memory_obj, &hetero_obj1, &hetero_obj2}) {
    const MeshSource& source = mesh_ptr->source();
    const bool is_disk = source.is_path();
    SCOPED_TRACE(fmt::format("Full obj from {} - {}",
                             is_disk ? "disk" : "memory",
                             is_disk ? std::string() : source.description()));
    DRAKE_DEMAND(meshcat.GetPackedObject("obj_path").empty());

    meshcat.SetObject("obj_path", *mesh_ptr);
    const std::string packed_obj = meshcat.GetPackedObject("obj_path");
    EXPECT_FALSE(packed_obj.empty());
    // Evidence that the image got loaded.
    EXPECT_THAT(packed_obj, testing::HasSubstr("data:image/png;base64"));
    // Evidence that the material library got loaded.
    EXPECT_THAT(packed_obj, testing::HasSubstr("newmtl Rainbow_Stripes"));
    // Evidence that the non-uniform scale was propagated.
    EXPECT_THAT(packed_obj, testing::HasSubstr(matrix_string));
    meshcat.Delete("obj_path");
    ASSERT_TRUE(meshcat.GetPackedObject("obj_path").empty());
  }

  // Missing elements from the in-memory mesh should proceed (but with missing
  // resources). Warnings are also spewed, but we can't test for those.

  // Missing the mtl file (whether the png is present or not), means no mtl and
  // no png.
  for (const InMemoryMesh& mem_mesh :
       {InMemoryMesh{obj_file},
        InMemoryMesh{obj_file,
                     {{"rainbow_stripes.png", MemoryFile(png_file)}}}}) {
    SCOPED_TRACE(fmt::format("Partial OBJ with {} supporting files",
                             mem_mesh.supporting_files.size()));
    DRAKE_DEMAND(meshcat.GetPackedObject("obj_path").empty());
    meshcat.SetObject("obj_path", Mesh(mem_mesh));
    const std::string packed_obj = meshcat.GetPackedObject("obj_path");
    EXPECT_FALSE(packed_obj.empty());
    EXPECT_THAT(packed_obj, Not(HasSubstr("data:image/png;base64")));
    EXPECT_THAT(packed_obj, Not(HasSubstr("newmtl Rainbow_Stripes")));
    meshcat.Delete("obj_path");
  }

  // If only the texture is missing, we still have "success" - materials are
  // loaded but the image is not.
  {
    DRAKE_DEMAND(meshcat.GetPackedObject("obj_path").empty());
    meshcat.SetObject(
        "obj_path",
        Mesh(InMemoryMesh{obj_file,
                          {{"rainbow_box.mtl", MemoryFile(mtl_file)}}}));
    const std::string packed_obj = meshcat.GetPackedObject("obj_path");
    EXPECT_FALSE(packed_obj.empty());
    EXPECT_THAT(packed_obj, Not(HasSubstr("data:image/png;base64")));
    EXPECT_THAT(packed_obj, HasSubstr("newmtl Rainbow_Stripes"));
    meshcat.Delete("obj_path");
  }

  // Meshcat defers to `meshcat_internal` logic for handling glTF files so
  // it's enough to show that good things happen. Tests on the internal
  // implementations are responsible for confirming it's the *right* thing.
  meshcat.SetObject(
      "gltf",
      Mesh(FindResourceOrThrow("drake/geometry/render/test/meshes/cube1.gltf"),
           0.25));
  EXPECT_FALSE(meshcat.GetPackedObject("gltf").empty());
}

// The correctness of this is established with meshcat_manual_test.  Here we
// simply aim to provide code coverage for CI (e.g., no segfaults).
// Meshes are treated in SetObjectWithMesh.
GTEST_TEST(MeshcatTest, SetObjectWithShape) {
  Meshcat meshcat;
  EXPECT_TRUE(meshcat.GetPackedObject("sphere").empty());
  meshcat.SetObject("sphere", Sphere(0.25), Rgba(1.0, 0, 0, 1));
  EXPECT_FALSE(meshcat.GetPackedObject("sphere").empty());
  meshcat.SetObject("cylinder", Cylinder(0.25, 0.5), Rgba(0.0, 1.0, 0, 1));
  EXPECT_FALSE(meshcat.GetPackedObject("cylinder").empty());
  // HalfSpaces are not supported yet; this should only log a warning.
  meshcat.SetObject("halfspace", HalfSpace());
  EXPECT_TRUE(meshcat.GetPackedObject("halfspace").empty());
  meshcat.SetObject("box", Box(0.25, 0.25, 0.5), Rgba(0, 0, 1, 1));
  EXPECT_FALSE(meshcat.GetPackedObject("box").empty());
  meshcat.SetObject("ellipsoid", Ellipsoid(0.25, 0.25, 0.5),
                    Rgba(1.0, 0, 1, 1));
  EXPECT_FALSE(meshcat.GetPackedObject("ellipsoid").empty());
  meshcat.SetObject("capsule", Capsule(0.25, 0.5));
  EXPECT_FALSE(meshcat.GetPackedObject("capsule").empty());
  meshcat.SetObject(
      "convex",
      Convex(FindResourceOrThrow("drake/geometry/render/test/meshes/box.obj"),
             0.25));
  EXPECT_FALSE(meshcat.GetPackedObject("convex").empty());
  // Bad filename (no extension).  Should only log a warning.
  meshcat.SetObject("bad", Mesh("test"));
  EXPECT_TRUE(meshcat.GetPackedObject("bad").empty());
  // Bad filename (file doesn't exist).  Should only log a warning.
  meshcat.SetObject("bad", Mesh("test.obj"));
  EXPECT_TRUE(meshcat.GetPackedObject("bad").empty());
}

// Confirms that an OBJ with a missing material becomes a MeshData instead of a
// MeshfileObjectData.
GTEST_TEST(MeshcatTest, ObjWithMissingMtl) {
  Meshcat meshcat;

  // The baseline .obj with .mtl file.
  const std::string obj_source =
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.obj");
  meshcat.SetObject("with_mtl", Mesh(obj_source), Rgba(1, 0, 1));
  const std::string with_mtl_packed = meshcat.GetPackedObject("with_mtl");
  EXPECT_THAT(with_mtl_packed, testing::HasSubstr("_meshfile_object"));
  EXPECT_THAT(with_mtl_packed, testing::HasSubstr("mtl_library"));

  // Copy the .obj into a directory, without its .mtl file.
  const fs::path dir = temp_directory();
  const fs::path missing_mtl_path = dir / "box.obj";
  fs::copy_file(obj_source, missing_mtl_path);
  meshcat.SetObject("missing_mtl", Mesh(missing_mtl_path.string()),
                    Rgba(1, 0.75, 0.5));

  const std::string missing_mtl_packed = meshcat.GetPackedObject("missing_mtl");
  EXPECT_THAT(missing_mtl_packed, testing::HasSubstr("_meshfile_geometry"));
  EXPECT_THAT(missing_mtl_packed, testing::HasSubstr("MeshPhongMaterial"));
  // The Rgba value above gets hex encoded as follows:
  EXPECT_THAT(missing_mtl_packed, testing::HasSubstr("\0\xFF\xBF\x7F"));
}

// When an .mtl file ends with a map_* line, and doesn't have a newline, we
// still need to extract the image name.
GTEST_TEST(MeshcatTest, MtlMapAtEOF) {
  Meshcat meshcat;

  // The baseline .obj with .mtl file.
  const std::string obj_source =
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.obj");
  meshcat.SetObject("original_mtl", Mesh(obj_source), Rgba(1, 0, 1));
  const std::string original_mtl_packed =
      meshcat.GetPackedObject("original_mtl");
  EXPECT_THAT(original_mtl_packed, testing::HasSubstr("_meshfile_object"));
  EXPECT_THAT(original_mtl_packed, testing::HasSubstr("mtl_library"));
  EXPECT_THAT(original_mtl_packed, testing::HasSubstr("data:image/png;base64"));

  // Copy the .obj and .png into a directory, write a single line .mtl (with no
  // newline).
  const fs::path dir = temp_directory();
  const fs::path eof_mtl_path = dir / "box.obj";
  fs::copy_file(obj_source, eof_mtl_path);
  // Copy the texture file that the .mtl will reference.
  const std::string png_source =
      FindResourceOrThrow("drake/geometry/render/test/meshes/box.png");
  fs::copy_file(png_source, dir / "box.png");

  std::ofstream eof_mtl_file(eof_mtl_path.string() + ".mtl");
  eof_mtl_file << "map_Kd -s 1 1 1 box.png";
  eof_mtl_file.close();
  meshcat.SetObject("eof_mtl", Mesh(eof_mtl_path.string()), Rgba(1, 0.75, 0.5));

  const std::string eof_mtl_packed = meshcat.GetPackedObject("eof_mtl");
  EXPECT_THAT(eof_mtl_packed, testing::HasSubstr("_meshfile_object"));
  EXPECT_THAT(eof_mtl_packed, testing::HasSubstr("mtl_library"));
  EXPECT_THAT(eof_mtl_packed, testing::HasSubstr("data:image/png;base64"));
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
  TriangleSurfaceMesh<double> surface_mesh(std::move(faces),
                                           std::move(vertices));
  meshcat.SetObject("triangle_mesh", surface_mesh, Rgba(0.9, 0, 0.9, 1.0));
  EXPECT_FALSE(meshcat.GetPackedObject("triangle_mesh").empty());

  meshcat.SetObject("triangle_mesh_wireframe", surface_mesh,
                    Rgba(0.9, 0, 0.9, 1.0), true, 5.0);
  EXPECT_FALSE(meshcat.GetPackedObject("triangle_mesh_wireframe").empty());
}

GTEST_TEST(MeshcatTest, PlotSurface) {
  Meshcat meshcat;

  constexpr int nx = 15, ny = 11;
  Eigen::MatrixXd X = RowVector<double, nx>::LinSpaced(0, 1).replicate<ny, 1>();
  Eigen::MatrixXd Y = Vector<double, ny>::LinSpaced(0, 1).replicate<1, nx>();
  // z = y*sin(5*x)
  Eigen::MatrixXd Z = (Y.array() * (5 * X.array()).sin()).matrix();

  // Wireframe = false.
  meshcat.PlotSurface("plot_surface", X, Y, Z, Rgba(0, 0, 0.9, 1.0), false);
  EXPECT_FALSE(meshcat.GetPackedObject("plot_surface").empty());

  // Wireframe = true.
  meshcat.PlotSurface("plot_surface_wireframe", X, Y, Z, Rgba(0, 0, 0.9, 1.0),
                      true);
  EXPECT_FALSE(meshcat.GetPackedObject("plot_surface_wireframe").empty());
}

GTEST_TEST(MeshcatTest, SetLine) {
  Meshcat meshcat;

  Eigen::Matrix3Xd vertices(3, 200);
  Eigen::RowVectorXd t = Eigen::RowVectorXd::LinSpaced(200, 0, 10 * M_PI);
  vertices << 0.25 * t.array().sin(), 0.25 * t.array().cos(), t / (10 * M_PI);
  meshcat.SetLine("line", vertices, 3.0, Rgba(0, 0, 1, 1));
  EXPECT_FALSE(meshcat.GetPackedObject("line").empty());

  Eigen::Matrix3Xd start(3, 4), end(3, 4);
  // clang-format off
  start << -0.1, -0.1,  0.1,  0.1,
           -0.1,  0.1, -0.1,  0.1,
              0,    0,    0,    0;
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
  const RigidTransformd X_ParentPath{RollPitchYawd(0.5, 0.26, -3),
                                     Vector3d{0.9, -2.0, 0.12}};
  meshcat.SetTransform("frame", X_ParentPath);

  const auto [actual_path, actual_value] =
      GetDecodedTransform(meshcat, "frame");
  EXPECT_EQ(actual_path, "/drake/frame");
  EXPECT_TRUE(CompareMatrices(actual_value, X_ParentPath.GetAsMatrix4()));
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

  const auto [actual_path, actual_value] =
      GetDecodedTransform(meshcat, "frame");
  EXPECT_EQ(actual_path, "/drake/frame");
  EXPECT_TRUE(CompareMatrices(actual_value, matrix));
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
  const std::string path{"/Grid"};
  const std::string property{"visible"};
  EXPECT_FALSE(meshcat.HasPath(path));
  EXPECT_TRUE(meshcat.GetPackedProperty(path, property).empty());

  meshcat.SetProperty(path, property, false);
  EXPECT_TRUE(meshcat.HasPath(path));
  const auto [actual_path, actual_value] =
      GetDecodedProperty<bool>(meshcat, path, property);
  EXPECT_EQ(actual_path, path);
  EXPECT_FALSE(actual_value);
}

GTEST_TEST(MeshcatTest, SetPropertyDouble) {
  Meshcat meshcat;
  const std::string path{"/Cameras/default/rotated/<object>"};
  const std::string property{"zoom"};
  EXPECT_FALSE(meshcat.HasPath(path));
  EXPECT_TRUE(meshcat.GetPackedProperty(path, property).empty());

  meshcat.SetProperty(path, property, 2.0);
  EXPECT_TRUE(meshcat.HasPath(path));
  const auto [actual_path, actual_value] =
      GetDecodedProperty<double>(meshcat, path, property);
  EXPECT_EQ(actual_path, path);
  EXPECT_EQ(actual_value, 2.0);
}

GTEST_TEST(MeshcatTest, SetEnvironmentMap) {
  Meshcat meshcat;
  const std::string path{"/Background/<object>"};
  const std::string property{"environment_map"};
  EXPECT_FALSE(meshcat.HasPath(path));
  auto [actual_path, actual_value] =
      GetDecodedProperty<std::string>(meshcat, path, property);
  EXPECT_EQ(actual_path, "");
  EXPECT_EQ(actual_value, "");

  // Set the map to a valid image.
  const fs::path env_map(
      FindResourceOrThrow("drake/geometry/test/env_256_cornell_box.png"));
  EXPECT_NO_THROW(meshcat.SetEnvironmentMap(env_map));
  EXPECT_TRUE(meshcat.HasPath(path));
  std::tie(actual_path, actual_value) =
      GetDecodedProperty<std::string>(meshcat, path, property);
  EXPECT_EQ(actual_path, path);
  EXPECT_THAT(actual_value, testing::StartsWith("cas-"));

  // Clear the map with an empty string.
  EXPECT_NO_THROW(meshcat.SetEnvironmentMap(""));
  EXPECT_TRUE(meshcat.HasPath(path));
  std::tie(actual_path, actual_value) =
      GetDecodedProperty<std::string>(meshcat, path, property);
  EXPECT_EQ(actual_path, path);
  EXPECT_EQ(actual_value, "");

  // An invalid map throws.
  DRAKE_EXPECT_THROWS_MESSAGE(meshcat.SetEnvironmentMap("invalid_file.png"),
                              ".*invalid_file.png.*environment_map.*");
}

GTEST_TEST(MeshcatTest, InitialPropeties) {
  // Construct Meshcat with some extra initial properties.
  const std::vector<double> some_vector{1.0, 2.0};
  const std::string some_string{"hello"};
  const bool some_bool{true};
  const double some_double{22.2};
  const Meshcat meshcat{MeshcatParams{
      .initial_properties =
          {
              {.path = "/a", .property = "p1", .value = some_vector},
              {.path = "/b", .property = "p2", .value = some_string},
              {.path = "/c", .property = "p3", .value = some_bool},
              {.path = "/d", .property = "p4", .value = some_double},
          },
  }};

  // Check that they all showed up.
  auto check_property = [&meshcat](auto path, auto property, auto value) {
    SCOPED_TRACE(fmt::format("property = {}", property));
    using T = decltype(value);
    const auto [actual_path, actual_value] =
        GetDecodedProperty<T>(meshcat, path, property);
    EXPECT_EQ(actual_path, path);
    EXPECT_EQ(actual_value, value);
  };
  check_property("/a", "p1", some_vector);
  check_property("/b", "p2", some_string);
  check_property("/c", "p3", some_bool);
  check_property("/d", "p4", some_double);
}

// Tests the functional logic of button handling, without actually creating any
// websocket connections. (The EphemeralPort case tests a button using an actual
// connection.)
GTEST_TEST(MeshcatTest, Buttons) {
  Meshcat meshcat;

  // Asking for clicks prior to adding is an error.
  DRAKE_EXPECT_THROWS_MESSAGE(meshcat.GetButtonClicks("alice"),
                              "Meshcat does not have any button named alice.*");

  // A new button starts out unclicked.
  meshcat.AddButton("alice");
  EXPECT_THAT(meshcat.GetButtonNames(), ElementsAre("alice"));
  EXPECT_EQ(meshcat.GetButtonClicks("alice"), 0);

  auto click = [&meshcat]() {
    internal::UserInterfaceEvent data;
    data.type = "button";
    data.name = "alice";
    std::stringstream message_stream;
    msgpack::pack(message_stream, data);
    meshcat.InjectWebsocketMessage(message_stream.str());
  };

  // Clicking the button increases the count.
  click();
  EXPECT_EQ(meshcat.GetButtonClicks("alice"), 1);

  // Adding using an existing button name resets its count.
  meshcat.AddButton("alice");
  EXPECT_EQ(meshcat.GetButtonClicks("alice"), 0);

  // Clicking the button increases the count again.
  click();
  EXPECT_EQ(meshcat.GetButtonClicks("alice"), 1);

  // Removing the button then asking for clicks is an error.
  EXPECT_TRUE(meshcat.DeleteButton("alice"));
  EXPECT_EQ(meshcat.GetButtonNames().size(), 0);
  DRAKE_EXPECT_THROWS_MESSAGE(meshcat.GetButtonClicks("alice"),
                              "Meshcat does not have any button named alice.*");

  // Strictly (the default) removing a missing button throws.
  DRAKE_EXPECT_THROWS_MESSAGE(meshcat.DeleteButton("alice"),
                              "Meshcat does not have any button named alice.*");
  EXPECT_FALSE(meshcat.DeleteButton("alice", /*strict = */ false));

  // Adding the button anew starts with a zero count again.
  meshcat.AddButton("alice");
  EXPECT_EQ(meshcat.GetButtonClicks("alice"), 0);

  // Buttons are removed when deleting all controls.
  meshcat.AddButton("bob");
  EXPECT_THAT(meshcat.GetButtonNames(), ElementsAre("alice", "bob"));
  meshcat.DeleteAddedControls();
  DRAKE_EXPECT_THROWS_MESSAGE(meshcat.GetButtonClicks("alice"),
                              "Meshcat does not have any button named alice.*");
  DRAKE_EXPECT_THROWS_MESSAGE(meshcat.GetButtonClicks("bob"),
                              "Meshcat does not have any button named bob.*");
  EXPECT_EQ(meshcat.GetButtonNames().size(), 0);

  // Adding a button with the keycode.
  meshcat.AddButton("alice", "KeyT");
  click();
  EXPECT_EQ(meshcat.GetButtonClicks("alice"), 1);
  // Adding with the same keycode still resets.
  meshcat.AddButton("alice", "KeyT");
  EXPECT_EQ(meshcat.GetButtonClicks("alice"), 0);
  // Adding the same button with an empty keycode throws.
  DRAKE_EXPECT_THROWS_MESSAGE(meshcat.AddButton("alice"),
                              ".*does not match the current keycode.*");
  // Adding the same button with a different keycode throws.
  DRAKE_EXPECT_THROWS_MESSAGE(meshcat.AddButton("alice", "KeyR"),
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
      "Meshcat does not have any slider named slider.*");

  meshcat.AddSlider("slider", 0.2, 1.5, 0.1, 0.5);
  EXPECT_NEAR(meshcat.GetSliderValue("slider"), 0.5, 1e-14);
  meshcat.SetSliderValue("slider", 0.7);
  EXPECT_NEAR(meshcat.GetSliderValue("slider"), 0.7, 1e-14);
  meshcat.SetSliderValue("slider", -2.0);
  EXPECT_NEAR(meshcat.GetSliderValue("slider"), 0.2, 1e-14);
  meshcat.SetSliderValue("slider", 2.0);
  EXPECT_NEAR(meshcat.GetSliderValue("slider"), 1.5, 1e-14);
  meshcat.SetSliderValue("slider", 1.245);
  EXPECT_NEAR(meshcat.GetSliderValue("slider"), 1.2, 1e-14);
  DRAKE_EXPECT_THROWS_MESSAGE(meshcat.AddSlider("slider", 0.2, 1.5, 0.1, 0.5),
                              "Meshcat already has a slider named slider.");

  EXPECT_TRUE(meshcat.DeleteSlider("slider"));

  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.GetSliderValue("slider"),
      "Meshcat does not have any slider named slider.*");

  meshcat.AddSlider("slider1", 2, 3, 0.01, 2.35);
  meshcat.AddSlider("slider2", 4, 5, 0.01, 4.56);

  auto slider_names = meshcat.GetSliderNames();
  EXPECT_THAT(slider_names, ElementsAre("slider1", "slider2"));

  meshcat.DeleteAddedControls();
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.GetSliderValue("slider1"),
      "Meshcat does not have any slider named slider1.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.GetSliderValue("slider2"),
      "Meshcat does not have any slider named slider2.*");

  // Strictly (the default) removing a missing slider throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.DeleteSlider("slider1"),
      "Meshcat does not have any slider named slider1.*");
  EXPECT_FALSE(meshcat.DeleteSlider("slider1", /*strict = */ false));

  slider_names = meshcat.GetSliderNames();
  EXPECT_EQ(slider_names.size(), 0);

  // AddSlider and SetSliderValue return the rounded/truncated values.
  EXPECT_NEAR(meshcat.AddSlider("slider_rounded1", 0.2, 1.5, 0.1, 0.512), 0.5,
              1e-14);
  EXPECT_NEAR(meshcat.AddSlider("slider_rounded2", 0.2, 1.5, 0.1, 0.1), 0.2,
              1e-14);
  EXPECT_NEAR(meshcat.SetSliderValue("slider_rounded1", 1.7), 1.5, 1e-14);
  EXPECT_NEAR(meshcat.SetSliderValue("slider_rounded2", 1.234), 1.2, 1e-14);
}

GTEST_TEST(MeshcatTest, DuplicateMixedControls) {
  Meshcat meshcat;

  meshcat.AddButton("button");
  meshcat.AddSlider("slider", 0.2, 1.5, 0.1, 0.5);

  // We cannot use AddButton nor AddSlider to change the type of an existing
  // control by attempting to re-use its name.
  DRAKE_EXPECT_THROWS_MESSAGE(meshcat.AddButton("slider"),
                              "Meshcat already has a slider named slider.");
  DRAKE_EXPECT_THROWS_MESSAGE(meshcat.AddButton("slider", "KeyR"),
                              "Meshcat already has a slider named slider.");
  DRAKE_EXPECT_THROWS_MESSAGE(meshcat.AddSlider("button", 0.2, 1.5, 0.1, 0.5),
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
    })""",
                        {}, {});

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
  ortho.bottom = 0.84;
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
              "fps": 64.0,
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
              "fps": 64.0,
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
              "fps": 64.0,
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

GTEST_TEST(MeshcatTest, RecordingFps) {
  // The hard-coded default values for Meshcat's default recording and a newly-
  // started recording should match the MeshcatAnimation constructor's default,
  // and retain that value throughout the whole lifecycle.
  const double default_fps = MeshcatAnimation().frames_per_second();
  Meshcat meshcat;
  EXPECT_EQ(meshcat.get_recording().frames_per_second(), default_fps);
  meshcat.StartRecording();
  EXPECT_EQ(meshcat.get_recording().frames_per_second(), default_fps);
  meshcat.StopRecording();
  EXPECT_EQ(meshcat.get_recording().frames_per_second(), default_fps);
  meshcat.PublishRecording();
  EXPECT_EQ(meshcat.get_recording().frames_per_second(), default_fps);
  meshcat.DeleteRecording();
  EXPECT_EQ(meshcat.get_recording().frames_per_second(), default_fps);

  // When the user provides a specific fps, it's likewise retained.
  const double new_fps = 22.22;
  meshcat.StartRecording(new_fps);
  EXPECT_EQ(meshcat.get_recording().frames_per_second(), new_fps);
  meshcat.StopRecording();
  EXPECT_EQ(meshcat.get_recording().frames_per_second(), new_fps);
  meshcat.PublishRecording();
  EXPECT_EQ(meshcat.get_recording().frames_per_second(), new_fps);
  meshcat.DeleteRecording();
  EXPECT_EQ(meshcat.get_recording().frames_per_second(), new_fps);
}

GTEST_TEST(MeshcatTest, RecordingSafety) {
  // It's safe to call recording-related functions without starting recording.
  Meshcat meshcat;
  EXPECT_NO_THROW(meshcat.get_recording());
  EXPECT_NO_THROW(meshcat.get_mutable_recording());
  EXPECT_NO_THROW(meshcat.DeleteRecording());
  EXPECT_NO_THROW(meshcat.PublishRecording());
  EXPECT_NO_THROW(meshcat.StopRecording());
}

template <typename T>
bool has_animation_property(const Meshcat& meshcat, std::string_view property,
                            int frame) {
  return meshcat.get_recording()
      .get_key_frame<T>(frame, "foo", property)
      .has_value();
}

// The unit test for MeshcatRecording covers the details of recording logic, so
// we can rely on it for most of the test coverage. Here, we only need to cover
// the few implementation details that sit within meshcat.cc using some basic
// acceptance tests.
GTEST_TEST(MeshcatTest, Recording) {
  // We'll use two devices under test, with set_visualizations_while_recording
  // configured differently in each. This helps us highlight & verify what's the
  // same vs different under that setting.
  Meshcat dut_live;  // Will use `true` for set_visualizations_while_recording.
  Meshcat dut_mute;  // Will use `false` for set_visualizations_while_recording.

  // Cycle through various Meshcat operations while in a sequence of different
  // recording states, to see what happens in each. We'll stop testing when we
  // hit the first error, because they tend to cascade so future errors aren't
  // that interesting.
  for (auto* dut : {&dut_live, &dut_mute}) {
    const bool is_live = dut == &dut_live;
    for (int sequence = 0; sequence <= 5; ++sequence) {
      SCOPED_TRACE(fmt::format("live = {}, sequence = {}", is_live, sequence));
      const double kFps = 64.0;
      if (sequence == 0) {
        // Prior to starting a recording.
      } else if (sequence == 1) {
        dut->StartRecording(kFps, is_live);
      } else if (sequence == 2) {
        // No change; keep recording.
      } else if (sequence == 3) {
        dut->StopRecording();
      } else if (sequence == 4) {
        dut->PublishRecording();
      } else if (sequence == 5) {
        dut->DeleteRecording();
      }

      // Set one of each type of property on the current frame.
      const double time = sequence / kFps;
      using Vec = std::vector<double>;
      dut->SetProperty("foo", "bravo", (sequence % 2) == 1, time);
      dut->SetProperty("foo", "delta", time, time);
      dut->SetProperty("foo", "victor", Vec{time, time, time}, time);
      dut->SetTransform("foo", RigidTransformd{Vector3d::Constant(time)}, time);

      // Check exactly which properties and frames have been recorded. (Note
      // that nothing here is affected by `is_live`; the recording should be the
      // same whether live or not.)
      for (int i = 0; i <= 5; ++i) {
        SCOPED_TRACE(fmt::format("i = {}", i));
        const bool has_reached = (sequence >= i);
        const bool should_record = (i >= 1) && (i <= 2);
        const bool not_deleted = (sequence < 5);
        const bool expected = has_reached && should_record && not_deleted;
        ASSERT_EQ(has_animation_property<bool>(*dut, "bravo", i), expected);
        ASSERT_EQ(has_animation_property<double>(*dut, "delta", i), expected);
        ASSERT_EQ(has_animation_property<Vec>(*dut, "victor", i), expected);
        ASSERT_EQ(has_animation_property<Vec>(*dut, "position", i), expected);
      }

      // Check the live property values. The dut_live will update all the time,
      // but the dut_mute will not update when the sequence number is 1 or 2.
      const bool bravo = GetDecodedProperty<bool>(*dut, "foo", "bravo").second;
      const double delta =
          GetDecodedProperty<double>(*dut, "foo", "delta").second;
      const Vec victor = GetDecodedProperty<Vec>(*dut, "foo", "victor").second;
      const Eigen::Matrix4d transform = GetDecodedTransform(*dut, "foo").second;
      const int live_sequence = (is_live || sequence >= 3) ? sequence : 0;
      const double live_time = live_sequence / kFps;
      EXPECT_EQ(bravo, (live_sequence % 2) == 1);
      EXPECT_EQ(delta, live_time);
      EXPECT_EQ(victor.at(0), live_time);
      EXPECT_EQ(transform(0, 3), live_time);
    }
  }
}

GTEST_TEST(MeshcatTest, Set2dRenderMode) {
  Meshcat meshcat;
  meshcat.Set2dRenderMode();
  // We simply confirm that all of the objects have been set, and use
  // meshcat_manual_test to check that the visualizer updates as we expect.
  EXPECT_FALSE(meshcat.GetPackedObject("/Cameras/default/rotated").empty());
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
  EXPECT_FALSE(meshcat.GetPackedObject("/Cameras/default/rotated").empty());
  EXPECT_FALSE(meshcat.GetPackedTransform("/Cameras/default").empty());
  EXPECT_FALSE(
      meshcat.GetPackedProperty("/Cameras/default/rotated/<object>", "position")
          .empty());
  EXPECT_FALSE(meshcat.GetPackedProperty("/Background", "visible").empty());
  EXPECT_FALSE(meshcat.GetPackedProperty("/Grid", "visible").empty());
  EXPECT_FALSE(meshcat.GetPackedProperty("/Axes", "visible").empty());
}

GTEST_TEST(MeshcatTest, SetCameraTarget) {
  Meshcat meshcat;
  meshcat.SetCameraTarget({1, 2, 3});
  // The actual meshcat message has the p_WT re-expressed in a y-up world frame.
  CheckWebsocketCommand(meshcat, {}, 1, R"""({
      "type": "set_target",
      "value": [1, 3, -2]
    })""");
}

GTEST_TEST(MeshcatTest, SetCameraPose) {
  Meshcat meshcat;
  meshcat.SetCameraPose({1, 2, 3}, {-2, -3, -4});
  // The actual meshcat messages have the p_WT re-expressed in a y-up world
  // frame.

  // SetCameraTarget() called on <-2, -3, -4>.
  CheckWebsocketCommand(meshcat, {}, 3, R"""({
      "type": "set_target",
      "value": [-2, -4, 3]
    })""");

  // /Cameras/default should have an identity transform.
  {
    const auto [actual_path, actual_value] =
        GetDecodedTransform(meshcat, "/Cameras/default");
    EXPECT_EQ(actual_path, "/Cameras/default");
    EXPECT_TRUE(CompareMatrices(actual_value, Eigen::Matrix4d::Identity()));
  }

  // /Cameras/default/rotated/<object> should have a position value equal to
  // <1, 3, -2>.
  {
    const auto [actual_path, actual_value] =
        GetDecodedProperty<std::vector<double>>(
            meshcat, "/Cameras/default/rotated/<object>", "position");
    EXPECT_EQ(actual_path, "/Cameras/default/rotated/<object>");
    EXPECT_EQ(actual_value, std::vector({1.0, 3.0, -2.0}));
  }
}

GTEST_TEST(MeshcatTest, CameraTracking) {
  Meshcat meshcat;

  auto inject = [&meshcat](const auto& message) {
    std::stringstream message_stream;
    msgpack::pack(message_stream, message);
    meshcat.InjectWebsocketMessage(message_stream.str());
  };

  // When no message has been received, no pose is available.
  EXPECT_EQ(meshcat.GetTrackedCameraPose(), std::nullopt);

  // A message with a valid transform (16 floats and is perspective is True).
  internal::UserInterfaceEvent valid_pose_message;
  valid_pose_message.type = "camera_pose";
  // clang-format off
  valid_pose_message.camera_pose = {1, 0, 0, 0,
                                    0, 1, 0, 0,
                                    0, 0, 1, 0,
                                    1, 2, 3, 1};
  // clang-format on
  valid_pose_message.is_perspective = true;

  // Transform y-up to z-up, and from facing in the +z direction to the -z
  // direction (with concomitant flip of the y-axis).
  const RigidTransformd X_WC_expected(
      RotationMatrixd(RollPitchYawd(M_PI / 2, M_PI, M_PI)), {1.0, -3.0, 2.0});

  // The message sent when the camera has an orthographic projection.
  internal::UserInterfaceEvent invalid_pose_message = valid_pose_message;
  invalid_pose_message.is_perspective = false;

  // Send valid meshcat pose - pose is available.
  inject(valid_pose_message);
  std::optional<RigidTransformd> X_WC = meshcat.GetTrackedCameraPose();
  ASSERT_TRUE(X_WC.has_value());

  // The pose has been transformed.
  EXPECT_TRUE(CompareMatrices(X_WC->GetAsMatrix34(),
                              X_WC_expected.GetAsMatrix34(), 1e-15));

  // Send invalid meshcat pose - pose is cleared.
  inject(invalid_pose_message);
  X_WC = meshcat.GetTrackedCameraPose();
  EXPECT_FALSE(X_WC.has_value());
}

// The tracked camera pose is discarded when the websocket disconnects.
GTEST_TEST(MeshcatTest, CameraTrackingDisconnect) {
  Meshcat meshcat;
  CheckWebsocketCommand(meshcat, R"""({
      "type": "camera_pose",
      "camera_pose": [
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 0
      ],
      "is_perspective": true
    })""",
                        {}, {});
  EXPECT_FALSE(meshcat.GetTrackedCameraPose().has_value());
}

GTEST_TEST(MeshcatTest, StaticHtml) {
  Meshcat meshcat;

  // Call each command that will be saved (at least) once.
  meshcat.SetObject("box", Box(0.25, 0.25, 0.5), Rgba(0, 0, 1, 1));
  meshcat.SetTransform("box", RigidTransformd(Vector3d{0, 0, 0}));
  meshcat.SetProperty("/Background", "visible", false);

  MeshcatAnimation animation;
  animation.SetTransform(0, "box", RigidTransformd());
  animation.SetTransform(20, "box",
                         RigidTransformd(RotationMatrixd::MakeZRotation(M_PI)));

  const std::string html = meshcat.StaticHtml();

  // Confirm that the js link was replaced.
  EXPECT_THAT(html, ::testing::Not(HasSubstr("meshcat.js")));
  // The static html replaces the javascript web socket connection code with
  // direct invocation of MeshCat with all of the data. We'll confirm that
  // this appears to have happened by testing for the presence of the injected
  // tree (base64 content) and the absence of what is *believed* to be the
  // delimiting text of the connection block.
  EXPECT_THAT(html, HasSubstr("data:application/octet-binary;base64"));
  EXPECT_THAT(html, ::testing::Not(HasSubstr("CONNECTION BLOCK")));
}

GTEST_TEST(MeshcatTest, StaticZip) {
  Meshcat meshcat;
  const std::string zip = meshcat.StaticZip();
  EXPECT_EQ(zip.substr(0, 4), "PK\x03\x04");
}

// Check that MeshcatParams.show_stats_plot sends a show_realtime_rate message.
GTEST_TEST(MeshcatTest, ShowStatsPlot) {
  MeshcatParams params;
  params.show_stats_plot = true;
  Meshcat meshcat(params);
  CheckWebsocketCommand(meshcat, {}, 1, R"""({
      "type": "show_realtime_rate",
      "show": true
    })""");
}

// Tests the logic of time advancement.
//
// We don't test for the realtime rate message broadcast directly. As part of
// the broadcast, the stored realtime rate gets updated and can be read from
// Meshcat::GetRealtimeRate(). We'll assume if the expected value is available
// there, then it got broadcast correctly.
//
// Most of the compuation logic is contained (and tested) in
// realtime_rate_calculator.*. This test just needs to confirm that things are
// correct:
//
//   1. Initialization doesn't broadcast a rate.
//      - In fact, initialization clears previously broadcast rate.
//   2. One message is broadcast for each period boundary passed.
//      Without explicitly consuming the messages, this is impossible to test.
//      Instead, we rely on inspection of behavior during code review to
//      validate. To test it, try running a simulation with a significantly
//      slowed realtime rate such that the visualizer's publish period is
//      longer than Meshcat's realtime_rate_period.
GTEST_TEST(MeshcatTest, SetSimulationTime) {
  // Arbitrary, non-default parameter value to confirm that it's getting used.
  const double wall_period = 0.625;
  MeshcatParams params{.realtime_rate_period = wall_period};
  Meshcat meshcat(params);

  // Initial realtime rate is always zero.
  ASSERT_EQ(meshcat.GetRealtimeRate(), 0.0);

  // Replace the wall clock timer with one we explicitly control.
  auto timer_ptr = std::make_unique<ManualTimer>();
  ManualTimer& timer = *timer_ptr;
  meshcat.InjectMockTimer(std::move(timer_ptr));

  // Initialize meshcat's simulation with the first invocation where sim and
  // wall times are both zero.
  meshcat.SetSimulationTime(0);

  // A list of test triples (wall_timer, sim_time, expected_realtime_rate) to
  // step through.
  const std::vector<std::array<double, 3>> tests{
      // Rate remains at zero prior to the wall clock reaching the first period.
      {wall_period * 0.7, 1.0, 0.0},
      {wall_period * 0.8, 1.2, 0.0},
      {wall_period * 0.9, 1.4, 0.0},
      // After completing the first period, we have a non-zero value.
      {wall_period * 1.5, 2.0, 2.0 / (wall_period * 1.5)},
      // Wall time advances but sim time backtracks => re-initialize.
      // This should (internally) tare the timer back to zero by calling
      // Start().
      {wall_period * 999, 0.5, 0.0},
      // Sim time advances.
      {wall_period * 1.5, 0.7, (0.7 - 0.5) / (wall_period * 1.5)},
  };

  // Step through the tests.
  for (int i = 0; i < ssize(tests); ++i) {
    const double wall_timer = tests[i][0];
    const double sim_time = tests[i][1];
    const double expected_realtime_rate = tests[i][2];
    SCOPED_TRACE(
        fmt::format("tests[{}] with wall_timer={}, sim_time={}, rtr={}", i,
                    wall_timer, sim_time, expected_realtime_rate));

    timer.set_tick(wall_timer);
    EXPECT_NO_THROW(meshcat.SetSimulationTime(sim_time));
    EXPECT_EQ(meshcat.GetRealtimeRate(), expected_realtime_rate);
    // Repeat calls are always a no-op.
    EXPECT_NO_THROW(meshcat.SetSimulationTime(sim_time));
    EXPECT_EQ(meshcat.GetRealtimeRate(), expected_realtime_rate);
  }
}

// Tests that the call to immediately broadcast a provided realtime rate value
// works. We're not actually testing that the message got sent (we'll rely on
// reviewer inspection). When we broadcast the rate, we also store the rate.
// We look at the stored value as proxy.
GTEST_TEST(MeshcatTest, RealtimeRate) {
  Meshcat meshcat;
  meshcat.SetRealtimeRate(2.2);
  EXPECT_EQ(meshcat.GetRealtimeRate(), 2.2);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
