#include "drake/geometry/meshcat.h"

#include <cstdlib>

#include <fmt/format.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <msgpack.hpp>

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
    command = std::move(command) + "'" + arg + "' ";
  }
  return std::system(command.c_str());
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
                        meshcat.web_url() + "/main.min.js"}),
            0);
  EXPECT_EQ(SystemCall({"/usr/bin/curl", "-o", "/dev/null", "--silent",
                        meshcat.web_url() + "/favicon.ico"}),
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
  DRAKE_EXPECT_THROWS_MESSAGE(Meshcat m2(7050),
                              "Meshcat failed to open a websocket port.");

  // The default constructor gets a default port.
  Meshcat m3;
  EXPECT_GE(m3.port(), 7000);
  EXPECT_LE(m3.port(), 7099);
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
  // Capsules are not supported yet; this should only log a warning.
  meshcat.SetObject("capsule", Capsule(.25, .5));
  EXPECT_TRUE(meshcat.GetPackedObject("capsule").empty());
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

  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.GetButtonClicks("button"),
      "Meshcat does not have any button named button.");

  meshcat.AddButton("button");
  EXPECT_EQ(meshcat.GetButtonClicks("button"), 0);
  meshcat.DeleteButton("button");

  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.GetButtonClicks("button"),
      "Meshcat does not have any button named button.");

  meshcat.AddButton("button1");
  meshcat.AddButton("button2");
  meshcat.DeleteAddedControls();
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.GetButtonClicks("button1"),
      "Meshcat does not have any button named button1.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.GetButtonClicks("button2"),
      "Meshcat does not have any button named button2.");
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

  meshcat.DeleteSlider("slider");

  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.GetSliderValue("slider"),
      "Meshcat does not have any slider named slider.");

  meshcat.AddSlider("slider1", 2, 3, 0.01, 2.35);
  meshcat.AddSlider("slider2", 4, 5, 0.01, 4.56);
  meshcat.DeleteAddedControls();
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.GetSliderValue("slider1"),
      "Meshcat does not have any slider named slider1.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      meshcat.GetSliderValue("slider2"),
      "Meshcat does not have any slider named slider2.");
}

void CheckWebsocketCommand(const drake::geometry::Meshcat& meshcat,
                           int message_num,
                           const std::string& desired_command_json) {
  EXPECT_EQ(SystemCall(
                {FindResourceOrThrow("drake/geometry/meshcat_websocket_client"),
                 meshcat.ws_url(), std::to_string(message_num),
                 desired_command_json}),
            0);
}

GTEST_TEST(MeshcatTest, SetPropertyWebSocket) {
  Meshcat meshcat;
  meshcat.SetProperty("/Background", "visible", false);
  CheckWebsocketCommand(meshcat, 1, R"""({
      "type": "set_property",
      "path": "/Background",
      "property": "visible",
      "value": false
    })""");
  meshcat.SetProperty("/Grid", "visible", false);
  // Note: The order of the messages is due to "/Background" < "/Grid" in the
  // std::map, not due to the order that SetProperty was called.
  CheckWebsocketCommand(meshcat, 1, R"""({
      "type": "set_property",
      "path": "/Background",
      "property": "visible",
      "value": false
    })""");
  CheckWebsocketCommand(meshcat, 2, R"""({
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
  CheckWebsocketCommand(meshcat, 1, R"""({
      "type": "set_object",
      "path": "/my/camera",
      "object": {
        "object": {
          "type": "PerspectiveCamera",
          "fov": 82.0,
          "aspect": 1.5,
          "near": 0.01,
          "far": 100
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
  CheckWebsocketCommand(meshcat, 1, R"""({
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
  CheckWebsocketCommand(meshcat, 1, R"""({
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
  // Confirm that I have some base64 content.
  EXPECT_THAT(html, HasSubstr("data:application/octet-binary;base64"));

  // Confirm that the meshcat.js link was replaced.
  EXPECT_THAT(html, ::testing::Not(HasSubstr("meshcat.js")));
}

}  // namespace
}  // namespace geometry
}  // namespace drake
