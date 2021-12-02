#include "drake/geometry/scene_graph.h"

#include <algorithm>
#include <string>
#include <utility>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_state.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace geometry {

using render::RenderLabel;
using std::make_unique;
using std::vector;
using systems::Context;
using systems::InputPort;
using systems::LeafSystem;
using systems::Parameters;
using systems::State;
using systems::SystemTypeTag;

namespace {

/* We're creating a T-valued abstract value (stored either as State or
 Parameter). AbstractValues aren't really supposed to do that. They should be
 the same type *regardless* the scalar type for the Context. However, this
 breaks that paradigm. I need to be able to assign (i.e., call
 Context::SetTimeStateAndParametersFrom()) on an AutoDiff-valued Context with
 a double-valued context. The Value type would die in that attempt (because the
 two types will be different). This implementation of the AbstractValue
 interface has special logic to catch and handle this case. It will *still*
 fail trying to set a double-valued Context from an autodiff-valued one; but
 that is generally a global failure and not an artifact of this abstract state.
 */
template <typename T>
class GeometryStateValue final : public Value<GeometryState<T>> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GeometryStateValue)

  GeometryStateValue() = default;
  explicit GeometryStateValue(const GeometryState<T>& state)
      : Value<GeometryState<T>>(state) {}

  std::unique_ptr<AbstractValue> Clone() const override {
    return make_unique<GeometryStateValue<T>>(this->get_value());
  }

  void SetFrom(const AbstractValue& other) override {
    if (!do_double_assign(other)) {
      // `other` doesn't contain GeometryState<double>, fall back to
      // Value::SetFrom() to see if assignment is otherwise valid.
      Value<GeometryState<T>>::SetFrom(other);
    }
  }

 private:
  /* Assigns to the GeometryState<T> stored in *this* Value iff other contains
   GeometryState<double>. Returns true if that assignment is successful.  */
  bool do_double_assign(const AbstractValue& other) {
    const GeometryStateValue<double>* double_value =
        dynamic_cast<const GeometryStateValue<double>*>(&other);
    if (double_value) {
      // This relies on the GeometryState::operator= conversion operation
      // assuming that *this* value's instance can be assigned to by a
      // double-valued instance.
      this->get_mutable_value() = double_value->get_value();
      return true;
    }
    return false;
  }

  template <typename>
  friend class GeometryStateValue;
};
}  // namespace

template <typename T>
SceneGraph<T>::SceneGraph()
    : LeafSystem<T>(SystemTypeTag<SceneGraph>{}) {
  model_inspector_.set(&model_);
  geometry_state_index_ =
      this->DeclareAbstractParameter(GeometryStateValue<T>());

  query_port_index_ =
      this->DeclareAbstractOutputPort("query", &SceneGraph::CalcQueryObject)
          .get_index();

  auto& pose_update_cache_entry = this->DeclareCacheEntry(
      "Cache guard for pose updates", &SceneGraph::CalcPoseUpdate,
      {this->all_input_ports_ticket()});
  pose_update_index_ = pose_update_cache_entry.cache_index();
}

template <typename T>
template <typename U>
SceneGraph<T>::SceneGraph(const SceneGraph<U>& other)
    : SceneGraph() {
  // TODO(SeanCurtis-TRI) This is very brittle; we are essentially assuming that
  //  T = AutoDiffXd. For now, that's true. If we ever support
  //  symbolic::Expression, this U --> T conversion will have to be more
  //  generic.
  model_ = *(other.model_.ToAutoDiffXd());

  // We need to guarantee that the same source ids map to the same port indices.
  // We'll do this by processing the source ids in monotonically increasing
  // order. This is predicated on several principles:
  //   1. Port indices monotonically increase.
  //   2. SourceIds monotonically increase.
  //   3. Registering sources generates a source id and its ports at the same
  //      time.
  // Therefore, for SourceIds i and j, the if i > j, then the port indices for
  // source i must all be greater than those for j.
  vector<SourceId> source_ids;
  for (const auto& pair : other.input_source_ids_) {
    source_ids.push_back(pair.first);
  }
  auto comparator = [](const SourceId& a, const SourceId& b) {
    return a.get_value() < b.get_value();
  };
  std::sort(source_ids.begin(), source_ids.end(), comparator);

  for (const auto& source_id : source_ids) {
    MakeSourcePorts(source_id);
    const auto& new_ports = input_source_ids_[source_id];
    const auto& ref_ports = other.input_source_ids_.at(source_id);
    DRAKE_DEMAND(new_ports.pose_port == ref_ports.pose_port);
  }
}

template <typename T>
SourceId SceneGraph<T>::RegisterSource(const std::string& name) {
  SourceId source_id = model_.RegisterNewSource(name);
  MakeSourcePorts(source_id);
  return source_id;
}

template <typename T>
bool SceneGraph<T>::SourceIsRegistered(SourceId id) const {
  return input_source_ids_.count(id) > 0;
}

template <typename T>
const InputPort<T>& SceneGraph<T>::get_source_pose_port(
    SourceId id) const {
  ThrowUnlessRegistered(id, "Can't acquire pose port for unknown source id: ");
  return this->get_input_port(input_source_ids_.at(id).pose_port);
}

template <typename T>
FrameId SceneGraph<T>::RegisterFrame(SourceId source_id,
                                     const GeometryFrame& frame) {
  return model_.RegisterFrame(source_id, frame);
}

template <typename T>
FrameId SceneGraph<T>::RegisterFrame(SourceId source_id, FrameId parent_id,
                                     const GeometryFrame& frame) {
  return model_.RegisterFrame(source_id, parent_id, frame);
}

template <typename T>
GeometryId SceneGraph<T>::RegisterGeometry(
    SourceId source_id, FrameId frame_id,
    std::unique_ptr<GeometryInstance> geometry) {
  return model_.RegisterGeometry(source_id, frame_id, std::move(geometry));
}

template <typename T>
GeometryId SceneGraph<T>::RegisterGeometry(
    Context<T>* context, SourceId source_id, FrameId frame_id,
    std::unique_ptr<GeometryInstance> geometry) const {
  auto& g_state = mutable_geometry_state(context);
  return g_state.RegisterGeometry(source_id, frame_id, std::move(geometry));
}

template <typename T>
GeometryId SceneGraph<T>::RegisterGeometry(
    SourceId source_id, GeometryId geometry_id,
    std::unique_ptr<GeometryInstance> geometry) {
  return model_.RegisterGeometryWithParent(source_id, geometry_id,
                                           std::move(geometry));
}

template <typename T>
GeometryId SceneGraph<T>::RegisterGeometry(
    Context<T>* context, SourceId source_id, GeometryId geometry_id,
    std::unique_ptr<GeometryInstance> geometry) const {
  auto& g_state = mutable_geometry_state(context);
  return g_state.RegisterGeometryWithParent(source_id, geometry_id,
                                            std::move(geometry));
}

template <typename T>
GeometryId SceneGraph<T>::RegisterAnchoredGeometry(
    SourceId source_id, std::unique_ptr<GeometryInstance> geometry) {
  return model_.RegisterAnchoredGeometry(source_id, std::move(geometry));
}

template <typename T>
void SceneGraph<T>::RemoveGeometry(SourceId source_id, GeometryId geometry_id) {
  model_.RemoveGeometry(source_id, geometry_id);
}

template <typename T>
void SceneGraph<T>::RemoveGeometry(Context<T>* context, SourceId source_id,
                                   GeometryId geometry_id) const {
  auto& g_state = mutable_geometry_state(context);
  g_state.RemoveGeometry(source_id, geometry_id);
}

template <typename T>
void SceneGraph<T>::AddRenderer(
    std::string name, std::unique_ptr<render::RenderEngine> renderer) {
  return model_.AddRenderer(std::move(name), std::move(renderer));
}

template <typename T>
bool SceneGraph<T>::HasRenderer(const std::string& name) const {
  return model_.HasRenderer(name);
}

template <typename T>
int SceneGraph<T>::RendererCount() const {
  return model_.RendererCount();
}

template <typename T>
vector<std::string> SceneGraph<T>::RegisteredRendererNames() const {
  return model_.RegisteredRendererNames();
}

template <typename T>
void SceneGraph<T>::AssignRole(SourceId source_id, GeometryId geometry_id,
                               ProximityProperties properties,
                               RoleAssign assign) {
  model_.AssignRole(source_id, geometry_id, std::move(properties), assign);
}

template <typename T>
void SceneGraph<T>::AssignRole(Context<T>* context, SourceId source_id,
                               GeometryId geometry_id,
                               ProximityProperties properties,
                               RoleAssign assign) const {
  auto& g_state = mutable_geometry_state(context);
  g_state.AssignRole(source_id, geometry_id, std::move(properties), assign);
}

template <typename T>
void SceneGraph<T>::AssignRole(SourceId source_id, GeometryId geometry_id,
                               PerceptionProperties properties,
                               RoleAssign assign) {
  model_.AssignRole(source_id, geometry_id, std::move(properties), assign);
}

template <typename T>
void SceneGraph<T>::AssignRole(Context<T>* context, SourceId source_id,
                               GeometryId geometry_id,
                               PerceptionProperties properties,
                               RoleAssign assign) const {
  auto& g_state = mutable_geometry_state(context);
  g_state.AssignRole(source_id, geometry_id, std::move(properties), assign);
}

template <typename T>
void SceneGraph<T>::AssignRole(SourceId source_id, GeometryId geometry_id,
                               IllustrationProperties properties,
                               RoleAssign assign) {
  model_.AssignRole(source_id, geometry_id, std::move(properties), assign);
}

template <typename T>
void SceneGraph<T>::AssignRole(Context<T>* context, SourceId source_id,
                               GeometryId geometry_id,
                               IllustrationProperties properties,
                               RoleAssign assign) const {
  static const logging::Warn one_time(
      "Due to a bug (see issue #13597), changing the illustration roles or "
      "properties in the context will not have any apparent effect in, at "
      "least, drake_visualizer. Please change the illustration role in the "
      "model prior to allocating the Context.");
  auto& g_state = mutable_geometry_state(context);
  g_state.AssignRole(source_id, geometry_id, std::move(properties), assign);
}

template <typename T>
int SceneGraph<T>::RemoveRole(SourceId source_id, FrameId frame_id, Role role) {
  return model_.RemoveRole(source_id, frame_id, role);
}

template <typename T>
int SceneGraph<T>::RemoveRole(Context<T>* context, SourceId source_id,
                              FrameId frame_id, Role role) const {
  auto& g_state = mutable_geometry_state(context);
  return g_state.RemoveRole(source_id, frame_id, role);
}

template <typename T>
int SceneGraph<T>::RemoveRole(SourceId source_id, GeometryId geometry_id,
                              Role role) {
  return model_.RemoveRole(source_id, geometry_id, role);
}

template <typename T>
int SceneGraph<T>::RemoveRole(Context<T>* context, SourceId source_id,
                              GeometryId geometry_id, Role role) const {
  auto& g_state = mutable_geometry_state(context);
  return g_state.RemoveRole(source_id, geometry_id, role);
}

template <typename T>
const SceneGraphInspector<T>& SceneGraph<T>::model_inspector() const {
  return model_inspector_;
}

template <typename T>
CollisionFilterManager SceneGraph<T>::collision_filter_manager() {
  return model_.collision_filter_manager();;
}

template <typename T>
CollisionFilterManager SceneGraph<T>::collision_filter_manager(
    Context<T>* context) const {
  return mutable_geometry_state(context).collision_filter_manager();
}

template <typename T>
void SceneGraph<T>::SetDefaultParameters(const Context<T>& context,
                                         Parameters<T>* parameters) const {
  LeafSystem<T>::SetDefaultParameters(context, parameters);
  parameters->template get_mutable_abstract_parameter<GeometryState<T>>(
      geometry_state_index_) = model_;
}

template <typename T>
void SceneGraph<T>::MakeSourcePorts(SourceId source_id) {
  // This will fail only if the source generator starts recycling source ids.
  DRAKE_ASSERT(input_source_ids_.count(source_id) == 0);
  // Create and store the input ports for this source id.
  SourcePorts& source_ports = input_source_ids_[source_id];
  source_ports.pose_port =
      this->DeclareAbstractInputPort(model_.GetName(source_id) + "_pose",
                                     Value<FramePoseVector<T>>())
          .get_index();
}

template <typename T>
void SceneGraph<T>::CalcQueryObject(const Context<T>& context,
                                    QueryObject<T>* output) const {
  // NOTE: This is an exception to the style guide. It takes a const reference
  // but then hangs onto a const pointer. The guide says the parameter should
  // itself be a const pointer. We're breaking the guide to satisfy the
  // following constraints:
  //   1. This function serves as the output port calc callback; the signature
  //      *must* be a const ref.
  //   2. The design of the QueryObject requires a persisted pointer to the
  //      context. However, the docs for the class emphasize that this should
  //      *not* be persisted (and copying it clears this persisted copy).
  //
  // See the todo in the header for an alternate formulation.
  output->set(&context, this);
}

template <typename T>
std::vector<FrameId> SceneGraph<T>::GetDynamicFrames(
    const GeometryState<T>& g_state, Role role) const {
  vector<FrameId> dynamic_frames;
  for (const auto& pair : g_state.frames_) {
    const FrameId frame_id = pair.first;
    if (frame_id == world_frame_id()) continue;
    if (g_state.NumGeometriesWithRole(frame_id, role) > 0) {
      dynamic_frames.push_back(frame_id);
    }
  }
  return dynamic_frames;
}

template <typename T>
void SceneGraph<T>::CalcPoseUpdate(const Context<T>& context,
                                   int*) const {
  // TODO(SeanCurtis-TRI): Update this when the cache is available.
  // This method is const and the context is const. Ultimately, this will pull
  // cached entities to do the query work. For now, we have to const cast the
  // thing so that we can update the geometry engine contained.

  using std::to_string;

  const GeometryState<T>& state = geometry_state(context);
  GeometryState<T>& mutable_state = const_cast<GeometryState<T>&>(state);

  // Process all sources *except*:
  //   - the internal source and
  //   - sources with no frames.
  // The internal source will be included in source_frame_id_map_ but *not* in
  // input_source_ids_.
  for (const auto& pair : state.source_frame_id_map_) {
    if (pair.second.size() > 0) {
      SourceId source_id = pair.first;
      const auto itr = input_source_ids_.find(source_id);
      if (itr != input_source_ids_.end()) {
        const auto& pose_port = this->get_input_port(itr->second.pose_port);
        if (!pose_port.HasValue(context)) {
          throw std::logic_error(
              fmt::format("Source '{}' (id: {}) has registered dynamic frames "
                          "but is not connected to the appropriate input port.",
                          state.GetName(source_id), source_id));
        }
        const auto& poses =
            pose_port.template Eval<FramePoseVector<T>>(context);
        mutable_state.SetFramePoses(source_id, poses);
      }
    }
  }

  mutable_state.FinalizePoseUpdate();
  // TODO(SeanCurtis-TRI): Add velocity as appropriate.
}

template <typename T>
void SceneGraph<T>::ThrowUnlessRegistered(SourceId source_id,
                                          const char* message) const {
  using std::to_string;
  if (input_source_ids_.find(source_id) == input_source_ids_.end()) {
    throw std::logic_error(message + to_string(source_id) + ".");
  }
}

template <typename T>
GeometryState<T>& SceneGraph<T>::mutable_geometry_state(
    Context<T>* context) const {
  return context->get_mutable_parameters()
      .template get_mutable_abstract_parameter<GeometryState<T>>(
          geometry_state_index_);
}

template <typename T>
const GeometryState<T>& SceneGraph<T>::geometry_state(
    const Context<T>& context) const {
  return context.get_parameters()
      .template get_abstract_parameter<GeometryState<T>>(geometry_state_index_);
}

// Explicitly instantiates on the most common scalar types.
template class SceneGraph<double>;
template class SceneGraph<AutoDiffXd>;

}  // namespace geometry
}  // namespace drake
