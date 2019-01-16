#include "drake/geometry/geometry_visualization.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/never_destroyed.h"
#include "drake/geometry/geometry_state.h"
#include "drake/geometry/internal_geometry.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_viewer_geometry_data.hpp"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

namespace drake {
namespace geometry {

namespace {

// Simple class for converting shape specifications into LCM-compatible shapes.
class ShapeToLcm : public ShapeReifier {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ShapeToLcm)

  ShapeToLcm() = default;
  ~ShapeToLcm() override = default;

  lcmt_viewer_geometry_data Convert(const Shape& shape,
                                    const Isometry3<double>& X_PG,
                                    const Eigen::Vector4d& in_color) {
    X_PG_ = X_PG;
    // NOTE: Reify *may* change X_PG_ based on the shape. For example, the
    // half-space requires an additional offset to shift the box representing
    // the plane *to* the plane.
    shape.Reify(this);

    // Saves the location and orientation of the visualization geometry in the
    // `lcmt_viewer_geometry_data` object. The location and orientation are
    // specified in the body's frame.
    Eigen::Map<Eigen::Vector3f> position(geometry_data_.position);
    position = X_PG_.translation().cast<float>();
    // LCM quaternion must be w, x, y, z.
    Eigen::Quaternion<double> q(X_PG_.linear());
    geometry_data_.quaternion[0] = q.w();
    geometry_data_.quaternion[1] = q.x();
    geometry_data_.quaternion[2] = q.y();
    geometry_data_.quaternion[3] = q.z();

    Eigen::Map<Eigen::Vector4f> color(geometry_data_.color);
    color = in_color.cast<float>();
    return geometry_data_;
  }

  void ImplementGeometry(const Sphere& sphere, void*) override {
    geometry_data_.type = geometry_data_.SPHERE;
    geometry_data_.num_float_data = 1;
    geometry_data_.float_data.push_back(static_cast<float>(
                                            sphere.get_radius()));
  }

  void ImplementGeometry(const Cylinder& cylinder, void*) override {
    geometry_data_.type = geometry_data_.CYLINDER;
    geometry_data_.num_float_data = 2;
    geometry_data_.float_data.push_back(static_cast<float>(
                                            cylinder.get_radius()));
    geometry_data_.float_data.push_back(static_cast<float>(
                                            cylinder.get_length()));
  }

  void ImplementGeometry(const HalfSpace&, void*) override {
    // Currently representing a half space as a big box. This assumes that the
    // underlying box representation is centered on the origin.
    geometry_data_.type = geometry_data_.BOX;
    geometry_data_.num_float_data = 3;
    // Box width, height, and thickness.
    geometry_data_.float_data.push_back(50);
    geometry_data_.float_data.push_back(50);
    const float thickness = 1;
    geometry_data_.float_data.push_back(thickness);

    // The final pose of the box is the half-space's pose pre-multiplied by
    // an offset sufficient to move the box down so it's top face lies on the
    // z = 0 plane.
    Isometry3<double> box_xform = Isometry3<double>::Identity();
    // Shift it down so that the origin lies on the top surface.
    box_xform.translation() << 0, 0, -thickness / 2;
    X_PG_ = X_PG_ * box_xform;
  }

  void ImplementGeometry(const Box& box, void*) override {
    geometry_data_.type = geometry_data_.BOX;
    geometry_data_.num_float_data = 3;
    // Box width, depth, and height.
    geometry_data_.float_data.push_back(static_cast<float>(box.width()));
    geometry_data_.float_data.push_back(static_cast<float>(box.depth()));
    geometry_data_.float_data.push_back(static_cast<float>(box.height()));
  }

  void ImplementGeometry(const Mesh& mesh, void*) override {
    geometry_data_.type = geometry_data_.MESH;
    geometry_data_.num_float_data = 3;
    geometry_data_.float_data.push_back(static_cast<float>(mesh.scale()));
    geometry_data_.float_data.push_back(static_cast<float>(mesh.scale()));
    geometry_data_.float_data.push_back(static_cast<float>(mesh.scale()));
    geometry_data_.string_data = mesh.filename();
  }

  // For visualization, Convex is the same as Mesh.
  void ImplementGeometry(const Convex& mesh, void*) override {
    geometry_data_.type = geometry_data_.MESH;
    geometry_data_.num_float_data = 3;
    geometry_data_.float_data.push_back(static_cast<float>(mesh.scale()));
    geometry_data_.float_data.push_back(static_cast<float>(mesh.scale()));
    geometry_data_.float_data.push_back(static_cast<float>(mesh.scale()));
    geometry_data_.string_data = mesh.filename();
  }

 private:
  lcmt_viewer_geometry_data geometry_data_{};
  // The transform from the geometry frame to its parent frame.
  Eigen::Isometry3d X_PG_;
};

lcmt_viewer_geometry_data MakeGeometryData(const Shape& shape,
                                           const Isometry3<double>& X_PG,
                                           const Eigen::Vector4d& in_color) {
  ShapeToLcm converter;
  return converter.Convert(shape, X_PG, in_color);
}

}  // namespace

namespace internal {

lcmt_viewer_load_robot GeometryVisualizationImpl::BuildLoadMessage(
    const GeometryState<double>& state) {
  using internal::InternalFrame;
  using internal::InternalGeometry;

  lcmt_viewer_load_robot message{};
  // Populate the message.

  // Collect the dynamic frames that actually have illustration geometry. These
  // (plus possibly the world frame) are the frames that will be broadcast in
  // the message.
  std::vector<std::pair<FrameId, int>> dynamic_frames;
  for (const auto& pair : state.frames_) {
    const FrameId frame_id = pair.first;
    // We'll handle the world frame special.
    if (frame_id == InternalFrame::world_frame_id()) continue;
    const int count =
        state.NumGeometriesWithRole(frame_id, Role::kIllustration);
    if (count > 0) {
      dynamic_frames.push_back({frame_id, count});
    }
  }
  // Add the world frame if it has geometries with illustration role.
  const int anchored_count = state.NumGeometriesWithRole(
      InternalFrame::world_frame_id(), Role::kIllustration);
  const int frame_count = static_cast<int>(dynamic_frames.size()) +
      (anchored_count > 0 ? 1 : 0);

  message.num_links = frame_count;
  message.link.resize(frame_count);

  const Eigen::Vector4d default_color({0.9, 0.9, 0.9, 1.0});

  int link_index = 0;
  // Load anchored geometry into the world frame.
  if (anchored_count) {
    message.link[0].name = "world";
    message.link[0].robot_num = 0;
    message.link[0].num_geom = anchored_count;
    message.link[0].geom.resize(anchored_count);
    int geom_index = 0;
    const InternalFrame& world_frame =
        state.frames_.at(InternalFrame::world_frame_id());
    for (const GeometryId id : world_frame.child_geometries()) {
      const InternalGeometry& geometry = state.geometries_.at(id);
      const IllustrationProperties* props = geometry.illustration_properties();
      if (props != nullptr) {
        const Shape& shape = geometry.shape();
        const Eigen::Vector4d& color = props->GetPropertyOrDefault(
            "phong", "diffuse", default_color);
        message.link[0].geom[geom_index] = MakeGeometryData(
            shape, geometry.X_FG(), color);
        ++geom_index;
      }
    }
    link_index = 1;
  }

  // Load dynamic geometry into their own frames.
  for (const auto& pair : dynamic_frames) {
    const FrameId frame_id = pair.first;
    const int geometry_count = pair.second;
    const internal::InternalFrame& frame = state.frames_.at(frame_id);
    SourceId s_id = state.get_source_id(frame.id());
    const std::string& src_name = state.get_source_name(s_id);
    // TODO(SeanCurtis-TRI): The name in the load message *must* match the name
    // in the update message. Make sure this code and the SceneGraph output
    // use a common code-base to translate (source_id, frame) -> name.
    message.link[link_index].name = src_name + "::" + frame.name();
    message.link[link_index].robot_num = frame.frame_group();
    message.link[link_index].num_geom = geometry_count;
    message.link[link_index].geom.resize(geometry_count);
    int geom_index = 0;
    for (GeometryId geom_id : frame.child_geometries()) {
      const InternalGeometry& geometry = state.geometries_.at(geom_id);
      const IllustrationProperties* props = geometry.illustration_properties();
      if (props != nullptr) {
        const Shape& shape = geometry.shape();
        const Eigen::Vector4d& color = props->GetPropertyOrDefault(
            "phong", "diffuse", default_color);
        message.link[link_index].geom[geom_index] =
            MakeGeometryData(shape, geometry.X_FG(), color);
        ++geom_index;
      }
    }
    ++link_index;
  }

  return message;
}

}  // namespace internal

// TODO(sherm1) Per Sean Curtis, the load message should take its geometry
// from a Context rather than directly out of the SceneGraph. If geometry
// has been added to the Context it won't be loaded here. A runtime
// geometry change will likely require a geometry-changed event.
void DispatchLoadMessage(const SceneGraph<double>& scene_graph,
                         lcm::DrakeLcmInterface* lcm) {
  lcmt_viewer_load_robot message =
      internal::GeometryVisualizationImpl::BuildLoadMessage(
          *scene_graph.initial_state_);
  // Send a load message.
  Publish(lcm, "DRAKE_VIEWER_LOAD_ROBOT", message);
}

systems::lcm::LcmPublisherSystem* ConnectDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const SceneGraph<double>& scene_graph,
    const systems::OutputPort<double>& pose_bundle_output_port,
    lcm::DrakeLcmInterface* lcm_optional) {
  using systems::lcm::LcmPublisherSystem;
  using systems::lcm::Serializer;
  using systems::rendering::PoseBundleToDrawMessage;

  DRAKE_DEMAND(builder != nullptr);

  PoseBundleToDrawMessage* converter =
      builder->template AddSystem<PoseBundleToDrawMessage>();

  LcmPublisherSystem* publisher =
      builder->template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
          std::make_unique<Serializer<drake::lcmt_viewer_draw>>(),
          lcm_optional, 1 / 60.0 /* publish period */);

  // The functor we create in publisher here holds a reference to scene_graph,
  // which must therefore live as long as publisher does. We can count on that
  // because scene_graph is required to be contained in builder (see method
  // documentation), along with the converter and publisher we just added.
  // Builder will transfer ownership of all of these objects to the Diagram it
  // eventually builds.
  publisher->AddInitializationMessage([&scene_graph](
      const systems::Context<double>&, lcm::DrakeLcmInterface* lcm) {
    DispatchLoadMessage(scene_graph, lcm);
  });

  // Note that this will fail if scene_graph is not actually in builder.
  builder->Connect(pose_bundle_output_port, converter->get_input_port(0));
  builder->Connect(*converter, *publisher);

  return publisher;
}

systems::lcm::LcmPublisherSystem* ConnectDrakeVisualizer(
    systems::DiagramBuilder<double>* builder,
    const SceneGraph<double>& scene_graph, lcm::DrakeLcmInterface* lcm) {
  return ConnectDrakeVisualizer(builder, scene_graph,
                                scene_graph.get_pose_bundle_output_port(), lcm);
}

IllustrationProperties MakeDrakeVisualizerProperties(
    const Vector4<double>& diffuse) {
  IllustrationProperties props;
  props.AddProperty("phong", "diffuse", diffuse);
  return props;
}

}  // namespace geometry
}  // namespace drake
