#include "drake/geometry/dev/geometry_visualization.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/geometry/dev/geometry_context.h"
#include "drake/geometry/dev/scene_graph.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/lcmt_viewer_geometry_data.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/systems/framework/context.h"

namespace drake {
namespace geometry {
namespace dev {
namespace {

using drake::geometry::dev::internal::GeometryVisualizationImpl;
using drake::systems::Context;
using Eigen::Isometry3d;
using Eigen::Vector4d;
using std::make_unique;
using std::unique_ptr;

// TODO(SeanCurtis-TRI): Test with more complex scenarios.

// Confirms that, for a simple scene, the message contains the right data.
GTEST_TEST(GeometryVisualization, SimpleScene) {
  SceneGraph<double> scene_graph;

  const std::string source_name("source");
  SourceId source_id = scene_graph.RegisterSource(source_name);

  const std::string frame_name("frame");
  FrameId frame_id = scene_graph.RegisterFrame(
      source_id, GeometryFrame(frame_name));

  // Floats because the lcm messages store floats.
  const float radius = 1.25f;
  const float r = 1.0;
  const float g = 0.5f;
  const float b = 0.25f;
  const float a = 0.125f;
  auto instance = make_unique<GeometryInstance>(
      Isometry3d::Identity(), make_unique<Sphere>(radius), "sphere");
  Vector4<double> color{r, g, b, a};
  instance->set_illustration_properties(
      geometry::MakeDrakeVisualizerProperties(color));
  scene_graph.RegisterGeometry(source_id, frame_id, std::move(instance));

  unique_ptr<Context<double>> context = scene_graph.AllocateContext();
  const GeometryContext<double>& geo_context =
      dynamic_cast<GeometryContext<double>&>(*context.get());

  lcmt_viewer_load_robot message = GeometryVisualizationImpl::BuildLoadMessage(
      geo_context.get_geometry_state());

  ASSERT_EQ(message.num_links, 1);
  const lcmt_viewer_link_data& link = message.link[0];
  EXPECT_EQ(link.name, source_name + "::" + frame_name);
  EXPECT_EQ(link.robot_num, 0);
  ASSERT_EQ(link.num_geom, 1);
  const lcmt_viewer_geometry_data& geometry = link.geom[0];
  // This is necessary because EXPECT_EQ takes a reference to the arguments and
  // lcmt_viewer_geometry_data::SPHERE is a static constexpr defined inline and
  // doesn't support odr usage.
  const int8_t sphere_type = lcmt_viewer_geometry_data::SPHERE;
  EXPECT_EQ(geometry.type, sphere_type);
  EXPECT_EQ(geometry.num_float_data, 1);
  EXPECT_EQ(geometry.float_data[0], radius);
  EXPECT_EQ(geometry.color[0], r);
  EXPECT_EQ(geometry.color[1], g);
  EXPECT_EQ(geometry.color[2], b);
  EXPECT_EQ(geometry.color[3], a);
}

}  // namespace
}  // namespace dev
}  // namespace geometry
}  // namespace drake
