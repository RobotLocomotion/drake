#include "drake/geometry/geometry_system.h"

#include <algorithm>
#include <string>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/geometry/geometry_context.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_state.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/rendering/pose_bundle.h"

namespace drake {
namespace geometry {

using systems::AbstractValue;
using systems::Context;
using systems::InputPortDescriptor;
using systems::LeafContext;
using systems::LeafSystem;
using systems::rendering::PoseBundle;
using systems::SystemOutput;
using systems::SystemSymbolicInspector;
using systems::SystemTypeTag;
using systems::Value;
using std::make_unique;
using std::vector;

#define GS_THROW_IF_CONTEXT_ALLOCATED ThrowIfContextAllocated(__FUNCTION__);

namespace {
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
      Value<GeometryState<T>>::SetFrom(other);
    }
  }

  void SetFromOrThrow(const AbstractValue& other) override {
    if (!do_double_assign(other)) {
      Value<GeometryState<T>>::SetFromOrThrow(other);
    }
  }

 private:
  bool do_double_assign(const AbstractValue& other) {
    const GeometryStateValue<double>* double_value =
        dynamic_cast<const GeometryStateValue<double>*>(&other);
    if (double_value) {
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
GeometrySystem<T>::GeometrySystem()
    : LeafSystem<T>(SystemTypeTag<geometry::GeometrySystem>{}) {
  auto state_value = make_unique<GeometryStateValue<T>>();
  initial_state_ = &state_value->template GetMutableValue<GeometryState<T>>();
  geometry_state_index_ = this->DeclareAbstractState(std::move(state_value));

  bundle_port_index_ =
      this->DeclareAbstractOutputPort(&GeometrySystem::MakePoseBundle,
                                      &GeometrySystem::CalcPoseBundle)
          .get_index();

  query_port_index_ =
      this->DeclareAbstractOutputPort(&GeometrySystem::MakeQueryHandle,
                                      &GeometrySystem::CalcQueryHandle)
          .get_index();
}

template <typename T>
template <typename U>
GeometrySystem<T>::GeometrySystem(const GeometrySystem<U>& other)
    : GeometrySystem() {
  // NOTE: If other.initial_state_ is not null, it means we're converting a
  // system that hasn't had its context allocated yet. We want the converted
  // system to persist the same state.
  if (other.initial_state_ != nullptr) {
    *initial_state_ = *(other.initial_state_->ToAutoDiffXd());
  } else {
    initial_state_ = nullptr;
  }

  // We need to guarantee that the same source ids map to the same port indexes.
  // We'll do this by processing the source ids in monotonically increasing
  // order. This is predicated on several principles:
  //   1. Port indices monotonically increase.
  //   2. SourceIds monotonically increase.
  //   3. Registering sources generates a source id and its ports at the same
  //      time.
  // Therefore, for SourceIds i and j, the if i > j, then the port indices for
  // source i must all be greater than those for j.
  std::vector<SourceId> source_ids;
  for (const auto pair : other.input_source_ids_) {
    source_ids.push_back(pair.first);
  }
  auto comparator = [](const SourceId& a, const SourceId& b) {
    return a.get_value() < b.get_value();
  };
  std::sort(source_ids.begin(), source_ids.end(), comparator);

  for (const auto source_id : source_ids) {
    MakeSourcePorts(source_id);
    const auto& new_ports = input_source_ids_[source_id];
    const auto& ref_ports = other.input_source_ids_.at(source_id);
    DRAKE_DEMAND(new_ports.id_port == ref_ports.id_port &&
                 new_ports.pose_port == ref_ports.pose_port);
  }
}

template <typename T>
SourceId GeometrySystem<T>::RegisterSource(const std::string &name) {
  GS_THROW_IF_CONTEXT_ALLOCATED
  SourceId source_id = initial_state_->RegisterNewSource(name);
  MakeSourcePorts(source_id);
  return source_id;
}

template <typename T>
const systems::InputPortDescriptor<T>&
GeometrySystem<T>::get_source_frame_id_port(SourceId id) {
  ThrowUnlessRegistered(
      id, "Can't acquire id port for unknown source id: ");
  return this->get_input_port(input_source_ids_[id].id_port);
}

template <typename T>
const systems::InputPortDescriptor<T>&
GeometrySystem<T>::get_source_pose_port(SourceId id) {
  ThrowUnlessRegistered(
      id, "Can't acquire pose port for unknown source id: ");
  return this->get_input_port(input_source_ids_[id].pose_port);
}

template <typename T>
FrameId GeometrySystem<T>::RegisterFrame(SourceId source_id,
                                         const GeometryFrame& frame) {
  GS_THROW_IF_CONTEXT_ALLOCATED
  return initial_state_->RegisterFrame(source_id, frame);
}

template <typename T>
FrameId GeometrySystem<T>::RegisterFrame(SourceId source_id, FrameId parent_id,
                                         const GeometryFrame& frame) {
  GS_THROW_IF_CONTEXT_ALLOCATED
  return initial_state_->RegisterFrame(source_id, parent_id, frame);
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterGeometry(
    SourceId source_id, FrameId frame_id,
    std::unique_ptr<GeometryInstance> geometry) {
  GS_THROW_IF_CONTEXT_ALLOCATED
  return initial_state_->RegisterGeometry(source_id, frame_id,
                                          std::move(geometry));
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterGeometry(
    SourceId source_id, GeometryId geometry_id,
    std::unique_ptr<GeometryInstance> geometry) {
  GS_THROW_IF_CONTEXT_ALLOCATED
  return initial_state_->RegisterGeometryWithParent(source_id, geometry_id,
                                                    std::move(geometry));
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterAnchoredGeometry(
    SourceId source_id,
    std::unique_ptr<GeometryInstance> geometry) {
  GS_THROW_IF_CONTEXT_ALLOCATED
  return initial_state_->RegisterAnchoredGeometry(source_id,
                                                  std::move(geometry));
}

template <typename T>
void GeometrySystem<T>::ClearSource(SourceId source_id) {
  GS_THROW_IF_CONTEXT_ALLOCATED
  initial_state_->ClearSource(source_id);
}

template <typename T>
void GeometrySystem<T>::RemoveFrame(SourceId source_id, FrameId frame_id) {
  GS_THROW_IF_CONTEXT_ALLOCATED
  initial_state_->RemoveFrame(source_id, frame_id);
}

template <typename T>
void GeometrySystem<T>::RemoveGeometry(SourceId source_id,
                                       GeometryId geometry_id) {
  GS_THROW_IF_CONTEXT_ALLOCATED
  initial_state_->RemoveGeometry(source_id, geometry_id);
}

template <typename T>
const std::string& GeometrySystem<T>::get_source_name(
    const QueryHandle<T>& handle, SourceId id) const {
  unused(handle, id);
  throw std::runtime_error("Not implemented yet.");
}

template <typename T>
bool GeometrySystem<T>::SourceIsRegistered(SourceId id) const {
  return input_source_ids_.count(id) > 0;
}

template <typename T>
FrameId GeometrySystem<T>::GetFrameId(
    const QueryHandle<T>& handle, GeometryId geometry_id) const {
  unused(handle, geometry_id);
  throw std::runtime_error("Not implemented yet.");
}

template <typename T>
std::vector<PenetrationAsPointPair<T>> GeometrySystem<T>::ComputePenetration(
    const QueryHandle<T>& handle) const {
  unused(handle);
  throw std::runtime_error("Not implemented yet.");
}

template <typename T>
QueryHandle<T> GeometrySystem<T>::MakeQueryHandle(
    const systems::Context<T>&) const {
  return QueryHandle<T>(nullptr, 0);
}

template <typename T>
void GeometrySystem<T>::MakeSourcePorts(SourceId source_id) {
  // This will fail only if the source generator starts recycling source ids.
  DRAKE_ASSERT(input_source_ids_.count(source_id) == 0);
  // Create and store the input ports for this source id.
  SourcePorts& source_ports = input_source_ids_[source_id];
  source_ports.id_port = this->DeclareAbstractInputPort().get_index();
  source_ports.pose_port = this->DeclareAbstractInputPort().get_index();
}


template <typename T>
void GeometrySystem<T>::CalcQueryHandle(const Context<T>& context,
                                        QueryHandle<T>* output) const {
  output->context_ = &context;
}

template <typename T>
PoseBundle<T> GeometrySystem<T>::MakePoseBundle(
    const Context<T>& context) const {
  const auto& g_context = static_cast<const GeometryContext<T>&>(context);
  const auto& g_state = g_context.get_geometry_state();
  PoseBundle<T> bundle(g_state.get_num_frames());
  int i = 0;
  for (FrameId f_id : g_state.get_frame_ids()) {
    int frame_group = g_state.get_frame_group(f_id);
    bundle.set_model_instance_id(i, frame_group);

    SourceId s_id = g_state.get_source_id(f_id);
    const std::string& src_name = g_state.get_source_name(s_id);
    const std::string& frm_name = g_state.get_frame_name(f_id);
    std::string name = src_name + "::" + frm_name;
    bundle.set_name(i, name);
    ++i;
  }
  return bundle;
}

template <typename T>
void GeometrySystem<T>::CalcPoseBundle(const Context<T>& context,
                                       PoseBundle<T>* output) const {
  // NOTE: Adding/removing frames during discrete updates will
  // change the size/composition of the pose bundle. This calculation will *not*
  // explicitly test this. It is assumed the discrete update will also be
  // responsible for updating the bundle in the output port.
  int i = 0;

  const auto& g_context = static_cast<const GeometryContext<T>&>(context);
  // TODO(SeanCurtis-TRI): Modify this when the cache is available to use the
  // cache instead of this heavy-handed update.
  FullPoseUpdate(g_context);
  const auto& g_state = g_context.get_geometry_state();
  for (FrameId f_id : g_state.get_frame_ids()) {
    output->set_pose(i, g_state.get_pose_in_world(f_id));
    // TODO(SeanCurtis-TRI): Handle velocity.
    ++i;
  }
}

template <typename T>
void GeometrySystem<T>::FullPoseUpdate(
    const GeometryContext<T>& context) const {
  // TODO(SeanCurtis-TRI): Update this when the cache is available.
  // This method is const and the context is const. Ultimately, this will pull
  // cached entities to do the query work. For now, we have to const cast the
  // thing so that we can update the geometry engine contained.

  using std::to_string;

  const GeometryState<T>& state = context.get_geometry_state();
  GeometryState<T>& mutable_state = const_cast<GeometryState<T>&>(state);

  auto throw_error = [](SourceId source_id, const std::string& origin) {
    throw std::logic_error(
        "Source " + to_string(source_id) + " has registered frames "
            "but does not provide " + origin + " values on the input port.");
  };

  for (const auto& pair : state.source_frame_id_map_) {
    if (pair.second.size() > 0) {
      SourceId source_id = pair.first;
      const auto itr = input_source_ids_.find(source_id);
      DRAKE_ASSERT(itr != input_source_ids_.end());
      const int id_port = itr->second.id_port;
      const auto id_port_value =
          this->template EvalAbstractInput(context, id_port);
      if (id_port_value) {
        const FrameIdVector& ids =
            id_port_value->template GetValue<FrameIdVector>();
        // TODO(SeanCurtis-TRI): In future versions consider moving this to
        // a DRAKE_ASSERT_VOID execution.
        state.ValidateFrameIds(ids);
        const int pose_port = itr->second.pose_port;
        const auto pose_port_value =
            this->template EvalAbstractInput(context, pose_port);
        if (pose_port_value) {
          const auto& poses =
              pose_port_value->template GetValue<FramePoseVector<T>>();
          mutable_state.SetFramePoses(ids, poses);
        } else {
          throw_error(source_id, "pose");
        }
      } else {
        throw_error(source_id, "id");
      }
    }
  }

  // TODO(SeanCurtis-TRI): Propagate changes to the geometry engine.
  // Again, this will change significantly when caching becomes available.

  // TODO(SeanCurtis-TRI): Add velocity as appropriate.
}

template <typename T>
std::unique_ptr<LeafContext<T>> GeometrySystem<T>::DoMakeContext() const {
  // Disallow further geometry source additions.
  initial_state_ = nullptr;
  DRAKE_ASSERT(geometry_state_index_ >= 0);
  return make_unique<GeometryContext<T>>(geometry_state_index_);
}

template <typename T>
void GeometrySystem<T>::ThrowIfContextAllocated(
    const char* source_method) const {
  if (initial_state_ == nullptr) {
    throw std::logic_error(
        "The call to " + std::string(source_method) + " is invalid; a "
        "context has already been allocated.");
  }
}

template <typename T>
void GeometrySystem<T>::ThrowUnlessRegistered(SourceId source_id,
                                              const char* message) const {
  using std::to_string;
  if (input_source_ids_.find(source_id) == input_source_ids_.end()) {
    throw std::logic_error(message + to_string(source_id) + ".");
  }
}

// Explicitly instantiates on the most common scalar types.
template class GeometrySystem<double>;
template class GeometrySystem<AutoDiffXd>;

// Don't leave the macro defined.
#undef GS_THROW_IF_CONTEXT_ALLOCATED

}  // namespace geometry
}  // namespace drake
