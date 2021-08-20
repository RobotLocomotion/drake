#include "drake/geometry/meshcat.h"

#include <cstdlib>

#include <fmt/format.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <msgpack.hpp>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/meshcat_types.h"

namespace drake {
namespace geometry {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using ::testing::HasSubstr;

// A small wrapper around std::system to ensure correct argument passing.
int SystemCall(const std::vector<std::string>& argv) {
  std::string command;
  for (const std::string& arg : argv) {
    // Note: Can't use ASSERT_THAT inside this subroutine.
    EXPECT_THAT(arg, ::testing::Not(::testing::HasSubstr("'")));
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

// The correctness of this is established with meshcat_manual_test.  Here we
// simply aim to provide code coverage for CI (e.g., no segfaults).
GTEST_TEST(MeshcatTest, SetObjectWithShape) {
  Meshcat meshcat;
  EXPECT_TRUE(meshcat.GetPackedObject("/meshcat/sphere").empty());
  meshcat.SetObject("/meshcat/sphere", Sphere(.25), Rgba(1.0, 0, 0, 1));
  EXPECT_FALSE(meshcat.GetPackedObject("/meshcat/sphere").empty());
  meshcat.SetObject("/meshcat/cylinder", Cylinder(.25, .5),
                    Rgba(0.0, 1.0, 0, 1));
  EXPECT_FALSE(meshcat.GetPackedObject("/meshcat/cylinder").empty());
  // HalfSpaces are not supported yet; this should only log a warning.
  meshcat.SetObject("/meshcat/halfspace", HalfSpace());
  EXPECT_TRUE(meshcat.GetPackedObject("/meshcat/halfspace").empty());
  meshcat.SetObject("/meshcat/box", Box(.25, .25, .5), Rgba(0, 0, 1, 1));
  EXPECT_FALSE(meshcat.GetPackedObject("/meshcat/box").empty());
  meshcat.SetObject("/meshcat/ellipsoid", Ellipsoid(.25, .25, .5),
                    Rgba(1., 0, 1, 1));
  EXPECT_FALSE(meshcat.GetPackedObject("/meshcat/ellipsoid").empty());
  // Capsules are not supported yet; this should only log a warning.
  meshcat.SetObject("/meshcat/capsule", Capsule(.25, .5));
  EXPECT_TRUE(meshcat.GetPackedObject("/meshcat/capsule").empty());
  meshcat.SetObject(
      "/meshcat/mesh",
      Mesh(FindResourceOrThrow(
               "drake/systems/sensors/test/models/meshes/box.obj"),
           .25));
  EXPECT_FALSE(meshcat.GetPackedObject("/meshcat/mesh").empty());
  meshcat.SetObject(
      "/meshcat/convex",
      Convex(FindResourceOrThrow(
                 "drake/systems/sensors/test/models/meshes/box.obj"),
             .25));
  EXPECT_FALSE(meshcat.GetPackedObject("/meshcat/convex").empty());
  // Bad filename (no extension).  Should only log a warning.
  meshcat.SetObject("/meshcat/bad", Mesh("test"));
  EXPECT_TRUE(meshcat.GetPackedObject("/meshcat/bad").empty());
  // Bad filename (file doesn't exist).  Should only log a warning.
  meshcat.SetObject("/meshcat/bad", Mesh("test.obj"));
  EXPECT_TRUE(meshcat.GetPackedObject("/meshcat/bad").empty());
}

GTEST_TEST(MeshcatTest, SetTransform) {
  Meshcat meshcat;
  EXPECT_FALSE(meshcat.HasPath("/meshcat/transform"));
  EXPECT_TRUE(meshcat.GetPackedTransform("/meshcat/transform").empty());
  const RigidTransformd X_PathParent{math::RollPitchYawd(.5, .26, -3),
                                     Vector3d{.9, -2., .12}};
  meshcat.SetTransform("/meshcat/transform", X_PathParent);

  std::string transform = meshcat.GetPackedTransform("/meshcat/transform");
  msgpack::object_handle oh =
      msgpack::unpack(transform.data(), transform.size());
  auto data = oh.get().as<internal::SetTransformData>();
  EXPECT_EQ(data.type, "set_transform");
  EXPECT_EQ(data.path, "/meshcat/transform");
  Eigen::Map<Eigen::Matrix4d> matrix(data.matrix);
  EXPECT_TRUE(CompareMatrices(matrix, X_PathParent.GetAsMatrix4()));
}

GTEST_TEST(MeshcatTest, Delete) {
  Meshcat meshcat;
  // Ok to delete the root.  (It never does anything).
  meshcat.Delete("");
  meshcat.Delete("/");
  EXPECT_FALSE(meshcat.HasPath("/meshcat/transform"));
  meshcat.SetTransform("/meshcat/transform", RigidTransformd{});
  EXPECT_TRUE(meshcat.HasPath("/meshcat/transform"));
  // Deleting a random string does nothing.
  meshcat.Delete("/meshcat/bad");
  // Using a trailing slash acts the same as a random string.
  meshcat.Delete("/meshcat/");
  EXPECT_TRUE(meshcat.HasPath("/meshcat/transform"));
  meshcat.Delete("/meshcat/transform");
  EXPECT_FALSE(meshcat.HasPath("/meshcat/transform"));

  // Deleting a parent directory deletes all children.
  meshcat.SetTransform("/meshcat/transform", RigidTransformd{});
  meshcat.SetTransform("/meshcat/transform2", RigidTransformd{});
  meshcat.SetTransform("/meshcat/another/transform", RigidTransformd{});
  EXPECT_TRUE(meshcat.HasPath("/meshcat/transform"));
  EXPECT_TRUE(meshcat.HasPath("/meshcat/transform2"));
  EXPECT_TRUE(meshcat.HasPath("/meshcat/another/transform"));
  meshcat.Delete("/meshcat");
  EXPECT_FALSE(meshcat.HasPath("/meshcat/transform"));
  EXPECT_FALSE(meshcat.HasPath("/meshcat/transform2"));
  EXPECT_FALSE(meshcat.HasPath("/meshcat/another/transform"));
  EXPECT_FALSE(meshcat.HasPath("/meshcat"));
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
}

}  // namespace
}  // namespace geometry
}  // namespace drake
