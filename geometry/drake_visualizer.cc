#include "drake/geometry/drake_visualizer.h"

#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/extract_double.h"
#include "drake/common/value.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/geometry/utilities.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/lcmt_viewer_geometry_data.hpp"
#include "drake/lcmt_viewer_load_robot.hpp"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace geometry {

using Eigen::Quaterniond;
using math::RigidTransformd;
using std::make_unique;
using std::vector;
using systems::Context;
using systems::EventStatus;
using systems::SystemTypeTag;

namespace {

// Simple class for converting shape specifications into LCM-compatible shapes.
class ShapeToLcm : public ShapeReifier {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ShapeToLcm)

  ShapeToLcm() = default;
  ~ShapeToLcm() override = default;

  lcmt_viewer_geometry_data Convert(const Shape& shape,
                                    const RigidTransformd& X_PG,
                                    const Rgba& in_color) {
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
    Quaterniond q(X_PG_.rotation().ToQuaternion());
    geometry_data_.quaternion[0] = q.w();
    geometry_data_.quaternion[1] = q.x();
    geometry_data_.quaternion[2] = q.y();
    geometry_data_.quaternion[3] = q.z();

    Eigen::Map<Eigen::Vector4f> color(geometry_data_.color);
    Eigen::Vector4d color_vec(in_color.r(), in_color.g(), in_color.b(),
                              in_color.a());
    color = color_vec.cast<float>();
    return geometry_data_;
  }

  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Sphere& sphere, void*) override {
    geometry_data_.type = geometry_data_.SPHERE;
    geometry_data_.num_float_data = 1;
    geometry_data_.float_data.push_back(static_cast<float>(sphere.radius()));
  }

  void ImplementGeometry(const Ellipsoid& ellipsoid, void*) override {
    geometry_data_.type = geometry_data_.ELLIPSOID;
    geometry_data_.num_float_data = 3;
    geometry_data_.float_data.push_back(static_cast<float>(ellipsoid.a()));
    geometry_data_.float_data.push_back(static_cast<float>(ellipsoid.b()));
    geometry_data_.float_data.push_back(static_cast<float>(ellipsoid.c()));
  }

  void ImplementGeometry(const Cylinder& cylinder, void*) override {
    geometry_data_.type = geometry_data_.CYLINDER;
    geometry_data_.num_float_data = 2;
    geometry_data_.float_data.push_back(static_cast<float>(cylinder.radius()));
    geometry_data_.float_data.push_back(static_cast<float>(cylinder.length()));
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
    // Shift it down so that the origin lies on the top surface.
    RigidTransformd box_xform{Eigen::Vector3d{0, 0, -thickness / 2}};
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

  void ImplementGeometry(const Capsule& capsule, void*) override {
    geometry_data_.type = geometry_data_.CAPSULE;
    geometry_data_.num_float_data = 2;
    geometry_data_.float_data.push_back(static_cast<float>(capsule.radius()));
    geometry_data_.float_data.push_back(static_cast<float>(capsule.length()));
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
  RigidTransformd X_PG_;
};

}  // namespace

template <typename T>
DrakeVisualizer<T>::DrakeVisualizer(lcm::DrakeLcmInterface* lcm,
                                    DrakeVisualizerParams params)
    : DrakeVisualizer(lcm, std::move(params), true) {}

template <typename T>
template <typename U>
DrakeVisualizer<T>::DrakeVisualizer(const DrakeVisualizer<U>& other)
    : DrakeVisualizer(nullptr, other.params_, false) {
  DRAKE_DEMAND(owned_lcm_ == nullptr);
  DRAKE_DEMAND(lcm_ == nullptr);
  const lcm::DrakeLcm* owned_lcm =
      dynamic_cast<const lcm::DrakeLcm*>(other.owned_lcm_.get());
  if (owned_lcm == nullptr) {
    throw std::runtime_error(
        "DrakeVisualizer can only be scalar converted if it owns its "
        "DrakeLcmInterface instance.");
  }
  owned_lcm_ = make_unique<lcm::DrakeLcm>(owned_lcm->get_lcm_url());
  lcm_ = owned_lcm_.get();
}

template <typename T>
const DrakeVisualizer<T>& DrakeVisualizer<T>::AddToBuilder(
    systems::DiagramBuilder<T>* builder, const SceneGraph<T>& scene_graph,
    lcm::DrakeLcmInterface* lcm, DrakeVisualizerParams params) {
  return AddToBuilder(builder, scene_graph.get_query_output_port(), lcm,
                      params);
}

template <typename T>
const DrakeVisualizer<T>& DrakeVisualizer<T>::AddToBuilder(
    systems::DiagramBuilder<T>* builder,
    const systems::OutputPort<T>& query_object_port,
    lcm::DrakeLcmInterface* lcm, DrakeVisualizerParams params) {
  auto& visualizer =
      *builder->template AddSystem<DrakeVisualizer<T>>(lcm, params);
  builder->Connect(query_object_port, visualizer.query_object_input_port());
  return visualizer;
}

template <typename T>
void DrakeVisualizer<T>::DispatchLoadMessage(const SceneGraph<T>& scene_graph,
                                             lcm::DrakeLcmInterface* lcm,
                                             DrakeVisualizerParams params) {
  DRAKE_DEMAND(lcm != nullptr);
  vector<internal::DynamicFrameData> dynamic_frames;
  PopulateDynamicFrameData(scene_graph.model_inspector(), params,
                           &dynamic_frames);
  SendLoadMessage(scene_graph.model_inspector(), params, dynamic_frames, 0,
                  lcm);
}

template <typename T>
DrakeVisualizer<T>::DrakeVisualizer(lcm::DrakeLcmInterface* lcm,
                                    DrakeVisualizerParams params, bool use_lcm)
    : systems::LeafSystem<T>(SystemTypeTag<DrakeVisualizer>{}),
      owned_lcm_((lcm || !use_lcm) ? nullptr : new lcm::DrakeLcm()),
      lcm_((lcm && use_lcm) ? lcm : owned_lcm_.get()),
      params_(std::move(params)) {
  // This constructor should not do anything that requires lcm_ (or owned_lcm_)
  // to be non-nullptr. By design, both being nullptr is a desired, expected
  // possibility during the scope of the constructor.
  if (params_.publish_period <= 0) {
    throw std::runtime_error(fmt::format(
        "DrakeVisualizer requires a positive publish period; {} was given",
        params_.publish_period));
  }
  if (params_.role == Role::kUnassigned) {
    throw std::runtime_error(
        "DrakeVisualizer cannot be be used for geometries with the "
        "Role::kUnassigned value. Please choose proximity, perception, or "
        "illustration");
  }

  this->DeclarePeriodicPublishEvent(params_.publish_period, 0.0,
                                    &DrakeVisualizer<T>::SendGeometryMessage);
  this->DeclareForcedPublishEvent(&DrakeVisualizer<T>::SendGeometryMessage);

  query_object_input_port_ =
      this->DeclareAbstractInputPort("query_object", Value<QueryObject<T>>())
          .get_index();

  // This cache entry depends on *nothing*.
  dynamic_data_cache_index_ =
      this->DeclareCacheEntry("dynamic_frames",
                              vector<internal::DynamicFrameData>(),
                              &DrakeVisualizer<T>::CalcDynamicFrameData,
                              {this->nothing_ticket()})
          .cache_index();
}

template <typename T>
EventStatus DrakeVisualizer<T>::SendGeometryMessage(
    const Context<T>& context) const {
  const auto& query_object =
      query_object_input_port().template Eval<QueryObject<T>>(context);
  const GeometryVersion& current_version =
      query_object.inspector().geometry_version();

  bool send_load_message = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!version_.IsSameAs(current_version, params_.role)) {
      send_load_message = true;
      version_ = current_version;
    }
  }
  if (send_load_message) {
    SendLoadMessage(query_object.inspector(), params_,
                    RefreshDynamicFrameData(context),
                    ExtractDoubleOrThrow(context.get_time()), lcm_);
  }

  SendDrawMessage(query_object, EvalDynamicFrameData(context),
                  ExtractDoubleOrThrow(context.get_time()), lcm_);

  return EventStatus::Succeeded();
}

template <typename T>
void DrakeVisualizer<T>::SendLoadMessage(
    const SceneGraphInspector<T>& inspector,
    const DrakeVisualizerParams& params,
    const std::vector<internal::DynamicFrameData>& dynamic_frames, double time,
    lcm::DrakeLcmInterface* lcm) {
  lcmt_viewer_load_robot message{};

  // Add the world frame if it has geometries with the specified role.
  const int anchored_count = inspector.NumGeometriesForFrameWithRole(
      inspector.world_frame_id(), params.role);
  const int frame_count =
      static_cast<int>(dynamic_frames.size()) + (anchored_count > 0 ? 1 : 0);

  message.num_links = frame_count;
  message.link.resize(frame_count);

  // Helper utility to create lcm geometry description from geometry id.
  auto make_geometry = [&params, &inspector](GeometryId g_id) {
    const GeometryProperties* props =
        inspector.GetProperties(g_id, params.role);
    // We assume that the g_id was obtained by asking for geometries with the
    // indicated role. So, by definition, the properties should be non-null.
    DRAKE_ASSERT(props != nullptr);
    const Shape& shape = inspector.GetShape(g_id);
    Rgba default_color = params.default_color;
    if (params.role != Role::kIllustration) {
      const GeometryProperties* illus_props =
          inspector.GetIllustrationProperties(g_id);
      if (illus_props) {
        default_color = illus_props->GetPropertyOrDefault("phong", "diffuse",
                                                          default_color);
      }
    }
    const Rgba& color =
        props->GetPropertyOrDefault("phong", "diffuse", default_color);
    return ShapeToLcm().Convert(shape, inspector.GetPoseInFrame(g_id), color);
  };

  int link_index = 0;
  // Load anchored geometry into the world frame.
  if (anchored_count) {
    message.link[0].name = "world";
    message.link[0].robot_num = 0;
    message.link[0].num_geom = anchored_count;
    message.link[0].geom.resize(anchored_count);
    int geom_index = -1;  // We'll pre-increment before using.
    for (const GeometryId& g_id :
         inspector.GetGeometries(inspector.world_frame_id(), params.role)) {
      message.link[0].geom[++geom_index] = make_geometry(g_id);
    }
    link_index = 1;
  }

  // Load dynamic geometry into their own frames.
  for (const auto& [frame_id, geometry_count, name] : dynamic_frames) {
    message.link[link_index].name = name;
    message.link[link_index].robot_num = inspector.GetFrameGroup(frame_id);
    message.link[link_index].num_geom = geometry_count;
    message.link[link_index].geom.resize(geometry_count);
    int geom_index = -1;  // We'll pre-increment before using.
    for (const GeometryId& g_id :
         inspector.GetGeometries(frame_id, params.role)) {
      message.link[link_index].geom[++geom_index] = make_geometry(g_id);
    }
    ++link_index;
  }

  lcm::Publish(lcm, "DRAKE_VIEWER_LOAD_ROBOT", message, time);
}

template <typename T>
void DrakeVisualizer<T>::SendDrawMessage(
    const QueryObject<T>& query_object,
    const vector<internal::DynamicFrameData>& dynamic_frames, double time,
    lcm::DrakeLcmInterface* lcm) {
  lcmt_viewer_draw message{};

  const int frame_count = static_cast<int>(dynamic_frames.size());

  message.timestamp = static_cast<int64_t>(time * 1000.0);
  message.num_links = frame_count;
  message.link_name.resize(frame_count);
  message.robot_num.resize(frame_count);
  message.position.resize(frame_count);
  message.quaternion.resize(frame_count);

  const SceneGraphInspector<T>& inspector = query_object.inspector();
  for (int i = 0; i < frame_count; ++i) {
    const FrameId frame_id = dynamic_frames[i].frame_id;
    message.robot_num[i] = inspector.GetFrameGroup(frame_id);
    message.link_name[i] = dynamic_frames[i].name;

    const math::RigidTransformd& X_WF =
        internal::convert_to_double(query_object.GetPoseInWorld(frame_id));
    message.position[i].resize(3);
    message.position[i][0] = X_WF.translation()[0];
    message.position[i][1] = X_WF.translation()[1];
    message.position[i][2] = X_WF.translation()[2];

    const Eigen::Quaternion<double> q = X_WF.rotation().ToQuaternion();
    message.quaternion[i].resize(4);
    message.quaternion[i][0] = q.w();
    message.quaternion[i][1] = q.x();
    message.quaternion[i][2] = q.y();
    message.quaternion[i][3] = q.z();
  }

  lcm::Publish(lcm, "DRAKE_VIEWER_DRAW", message, time);
}

template <typename T>
void DrakeVisualizer<T>::CalcDynamicFrameData(
    const Context<T>& context,
    vector<internal::DynamicFrameData>* frame_data) const {
  const auto& query_object =
      query_object_input_port().template Eval<QueryObject<T>>(context);
  PopulateDynamicFrameData(query_object.inspector(), params_, frame_data);
}

template <typename T>
const vector<internal::DynamicFrameData>&
DrakeVisualizer<T>::RefreshDynamicFrameData(const Context<T>& context) const {
  // We'll need to make sure our knowledge of dynamic frames can get updated.
  this->get_cache_entry(dynamic_data_cache_index_)
      .get_mutable_cache_entry_value(context)
      .mark_out_of_date();

  return EvalDynamicFrameData(context);
}

template <typename T>
const vector<internal::DynamicFrameData>&
DrakeVisualizer<T>::EvalDynamicFrameData(const Context<T>& context) const {
  return this->get_cache_entry(dynamic_data_cache_index_)
      .template Eval<vector<internal::DynamicFrameData>>(context);
}

template <typename T>
void DrakeVisualizer<T>::PopulateDynamicFrameData(
    const SceneGraphInspector<T>& inspector,
    const DrakeVisualizerParams& params,
    vector<internal::DynamicFrameData>* frame_data) {
  // Collect the dynamic frames that actually have geometries of the
  // specified role. These are the frames broadcast in a draw message and are
  // also part of the load message (plus possibly the world frame).
  vector<internal::DynamicFrameData>& dynamic_frames = *frame_data;
  dynamic_frames.clear();

  for (const FrameId& frame_id : inspector.all_frame_ids()) {
    // We'll handle the world frame special.
    if (frame_id == inspector.world_frame_id()) continue;
    const int count =
        inspector.NumGeometriesForFrameWithRole(frame_id, params.role);
    if (count > 0) {
      dynamic_frames.push_back({frame_id, count,
                                inspector.GetOwningSourceName(frame_id) +
                                    "::" + inspector.GetName(frame_id)});
    }
  }
}

}  // namespace geometry
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::geometry::DrakeVisualizer)
