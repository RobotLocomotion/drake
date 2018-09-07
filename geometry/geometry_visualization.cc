#include "drake/geometry/geometry_visualization.h"

#include <memory>
#include <string>
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
  using internal::InternalAnchoredGeometry;
  using internal::InternalGeometry;

  lcmt_viewer_load_robot message{};
  // Populate the message.
  const int frame_count = state.get_num_frames();
  const int anchored_count =
      static_cast<int>(state.anchored_geometry_index_id_map_.size());

  // Include the world frame as one of the frames (if there are anchored
  // geometries).
  int total_link_count = frame_count + (anchored_count > 0 ? 1 : 0);
  message.num_links = total_link_count;
  message.link.resize(total_link_count);

  int link_index = 0;
  // Load anchored geometry into the world frame.
  {
    if (anchored_count) {
      message.link[0].name = "world";
      message.link[0].robot_num = 0;
      message.link[0].num_geom = anchored_count;
      message.link[0].geom.resize(anchored_count);
      int geom_index = 0;
      for (const auto& pair : state.anchored_geometries_) {
        const InternalAnchoredGeometry& geometry = pair.second;
        const Shape& shape = geometry.get_shape();
        const Eigen::Vector4d& color =
            geometry.get_visual_material().diffuse();
        message.link[0].geom[geom_index] = MakeGeometryData(
            shape, geometry.get_pose_in_parent(), color);
        ++geom_index;
      }
      link_index = 1;
    }
  }

  // Load dynamic geometry into their own frames.
  for (const auto& pair : state.frames_) {
    const internal::InternalFrame& frame = pair.second;
    SourceId s_id = state.get_source_id(frame.get_id());
    const std::string& src_name = state.get_source_name(s_id);
    // TODO(SeanCurtis-TRI): The name in the load message *must* match the name
    // in the update message. Make sure this code and the SceneGraph output
    // use a common code-base to translate (source_id, frame) -> name.
    message.link[link_index].name = src_name + "::" + frame.get_name();
    message.link[link_index].robot_num = frame.get_frame_group();
    const int geom_count = static_cast<int>(
        frame.get_child_geometries().size());
    message.link[link_index].num_geom = geom_count;
    message.link[link_index].geom.resize(geom_count);
    int geom_index = 0;
    for (GeometryId geom_id : frame.get_child_geometries()) {
      const InternalGeometry& geometry = state.geometries_.at(geom_id);
      GeometryIndex index = geometry.get_engine_index();
      const Isometry3<double> X_FG = state.X_FG_.at(index);
      const Shape& shape = geometry.get_shape();
      const Eigen::Vector4d& color =
          geometry.get_visual_material().diffuse();
      message.link[link_index].geom[geom_index] =
          MakeGeometryData(shape, X_FG, color);
      ++geom_index;
    }
    ++link_index;
  }

  return message;
}

// This System should be included in a diagram for the purpose of ensuring
// that the one-time load geometry message gets sent to the visualizer upon
// Simulator initialization. It will also serve to own the DrakeLcm object if
// one isn't provided on construction.
// TODO(sherm1) Move this functionality to LcmPublisher and remove this System
//              (see issue #9397).
class LcmInitSystem : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmInitSystem)

  LcmInitSystem(const SceneGraph<double>* scene_graph,
                lcm::DrakeLcmInterface* lcm = nullptr)
      : scene_graph_(*scene_graph), lcm_(lcm) {
    DRAKE_DEMAND(scene_graph != nullptr);

    // Create a DrakeLcm object if we don't already have one.
    if (lcm_ == nullptr) {
      owned_lcm_ = std::make_unique<lcm::DrakeLcm>();
      lcm_ = owned_lcm_.get();
    }

    // Create an initialization event that sends the load message.
    // TODO(sherm1) This is pulling geometry from the SceneGraph but should
    // be changed to use the Context for the most current version of the
    // geometry, once geometry is changeable at runtime. Also, subsequent
    // geometry changes are likely to require a "geometry changed" message
    // to be sent.
    DeclareInitializationEvent(systems::PublishEvent<double>(
        systems::Event<double>::TriggerType::kInitialization,
        [this](const systems::Context<double>&,
               const systems::PublishEvent<double>&) {
          DispatchLoadMessage(this->scene_graph_, this->lcm_);
        }));
  }

  // Obtain a reference to the DrakeLcm object, which may have been supplied
  // on construction or may be internally owned.
  lcm::DrakeLcmInterface& lcm() { return *lcm_; }

 private:
  const SceneGraph<double>& scene_graph_;
  lcm::DrakeLcmInterface* lcm_{nullptr};
  std::unique_ptr<lcm::DrakeLcm> owned_lcm_;
};

}  // namespace internal

// TODO(sherm1) Per Sean Curtis, the load message should take its geometry
// from a Context rather than directly out of the SceneGraph. If geometry
// has been added to the Context it won't be loaded here.
void DispatchLoadMessage(const SceneGraph<double>& scene_graph,
                         lcm::DrakeLcmInterface* lcm) {
  lcmt_viewer_load_robot message =
      internal::GeometryVisualizationImpl::BuildLoadMessage(
          *scene_graph.initial_state_);
  // Send a load message.
  Publish(lcm, "DRAKE_VIEWER_LOAD_ROBOT", message);
}

void ConnectDrakeVisualizer(systems::DiagramBuilder<double>* builder,
                            const SceneGraph<double>& scene_graph,
                            lcm::DrakeLcmInterface* lcm_optional) {
  using systems::lcm::LcmPublisherSystem;
  using systems::lcm::Serializer;
  using systems::rendering::PoseBundleToDrawMessage;

  DRAKE_DEMAND(builder != nullptr);

  // This disembodied system serves to send out the visualizer's needed
  // initialization message. Allocates a DrakeLcm if the supplied one is null.
  internal::LcmInitSystem* lcm_system =
      builder->AddSystem<internal::LcmInitSystem>(&scene_graph, lcm_optional);

  PoseBundleToDrawMessage* converter =
      builder->template AddSystem<PoseBundleToDrawMessage>();

  LcmPublisherSystem* publisher =
      builder->template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
          std::make_unique<Serializer<drake::lcmt_viewer_draw>>(),
          &lcm_system->lcm());
  publisher->set_publish_period(1 / 60.0);

  builder->Connect(scene_graph.get_pose_bundle_output_port(),
                   converter->get_input_port(0));
  builder->Connect(*converter, *publisher);
}

}  // namespace geometry
}  // namespace drake
