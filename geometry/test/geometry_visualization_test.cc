#include "drake/geometry/geometry_visualization.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcmt_viewer_geometry_data.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/systems/framework/context.h"

namespace drake {
namespace geometry {
namespace {

using drake::geometry::internal::GeometryVisualizationImpl;
using drake::systems::Context;
using Eigen::Vector4d;
using math::RigidTransformd;
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
  GeometryId sphere_id = scene_graph.RegisterGeometry(
      source_id, frame_id,
      make_unique<GeometryInstance>(RigidTransformd::Identity(),
                                    make_unique<Sphere>(radius), "sphere"));
  Vector4<double> color{r, g, b, a};
  scene_graph.AssignRole(source_id, sphere_id,
                         MakePhongIllustrationProperties(color));

  // Add a second frame and geometry that only has proximity properties. It
  // should not impact the result.
  const std::string collision_frame_name("collision frame");
  FrameId collision_frame_id =
      scene_graph.RegisterFrame(source_id, GeometryFrame(collision_frame_name));
  GeometryId collision_id = scene_graph.RegisterGeometry(
      source_id, collision_frame_id,
      make_unique<GeometryInstance>(RigidTransformd::Identity(),
      make_unique<Sphere>(radius), "sphere_collision"));
  scene_graph.AssignRole(source_id, collision_id, ProximityProperties());

  unique_ptr<Context<double>> context = scene_graph.AllocateContext();

  // This exploits the knowledge that the GeometryState is the zero-indexed
  // abstract state in the scene graph-allocated context.
  const GeometryState<double>& geometry_state =
      context->get_state().get_abstract_state<GeometryState<double>>(0);

  // Checks the load message for visual geometries.
  {
    lcmt_viewer_load_robot message =
        GeometryVisualizationImpl::BuildLoadMessage(geometry_state,
                                                    Role::kIllustration);

    ASSERT_EQ(message.num_links, 1);
    const lcmt_viewer_link_data& link = message.link[0];
    EXPECT_EQ(link.name, source_name + "::" + frame_name);
    EXPECT_EQ(link.robot_num, 0);
    ASSERT_EQ(link.num_geom, 1);
    const lcmt_viewer_geometry_data& geometry = link.geom[0];
    // This is necessary because EXPECT_EQ takes a reference to the arguments
    // and lcmt_viewer_geometry_data::SPHERE is a static constexpr defined
    // inline and doesn't support odr usage.
    const int8_t sphere_type = lcmt_viewer_geometry_data::SPHERE;
    EXPECT_EQ(geometry.type, sphere_type);
    EXPECT_EQ(geometry.num_float_data, 1);
    EXPECT_EQ(geometry.float_data[0], radius);
    EXPECT_EQ(geometry.color[0], r);
    EXPECT_EQ(geometry.color[1], g);
    EXPECT_EQ(geometry.color[2], b);
    EXPECT_EQ(geometry.color[3], a);
  }

  // Checks the load message for collision geometries.
  {
    lcmt_viewer_load_robot message =
        GeometryVisualizationImpl::BuildLoadMessage(geometry_state,
                                                    Role::kProximity);

    ASSERT_EQ(message.num_links, 1);
    const lcmt_viewer_link_data& link = message.link[0];
    EXPECT_EQ(link.name, source_name + "::" + collision_frame_name);
    EXPECT_EQ(link.robot_num, 0);
    ASSERT_EQ(link.num_geom, 1);
    const lcmt_viewer_geometry_data& geometry = link.geom[0];
    // This is necessary because EXPECT_EQ takes a reference to the arguments
    // and lcmt_viewer_geometry_data::SPHERE is a static constexpr defined
    // inline and doesn't support odr usage.
    const int8_t sphere_type = lcmt_viewer_geometry_data::SPHERE;
    EXPECT_EQ(geometry.type, sphere_type);
    EXPECT_EQ(geometry.num_float_data, 1);
    EXPECT_EQ(geometry.float_data[0], radius);

    // GeometryVisualizationImpl::BuildLoadMessage() has a hard coded default
    // color.
    const Eigen::Vector4f default_color({0.9, 0.9, 0.9, 1.0});
    for (int i = 0; i < 4; i++) {
      EXPECT_EQ(geometry.color[i], default_color[i]);
    }
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
