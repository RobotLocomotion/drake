#include "drake/geometry/geometry_system.h"

#include <string>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace geometry {

using systems::Context;
using systems::InputPortDescriptor;
using systems::LeafContext;
using systems::LeafSystem;
using systems::SparsityMatrix;
using systems::SystemOutput;
using std::vector;

#define THROW_IF_CONTEXT_ALLOCATED ThrowIfContextAllocated(__FUNCTION__);

template <typename T>
GeometrySystem<T>::GeometrySystem() : LeafSystem<T>() {
  query_port_index_ =
      this->DeclareAbstractOutputPort(&GeometrySystem::MakeQueryHandle,
                                      &GeometrySystem::CalcQueryHandle)
          .get_index();
}

template <typename T>
GeometrySystem<T>::~GeometrySystem() {}

template <typename T>
SourceId GeometrySystem<T>::RegisterSource(const std::string &name) {
  unused(name);
  THROW_IF_CONTEXT_ALLOCATED
  // TODO(SeanCurtis-TRI): Replace dummy source id with actual id.
  SourceId source_id = SourceId::get_new_id();
  // This will fail only if the source generator starts recycling source ids.
  DRAKE_ASSERT(input_source_ids_.count(source_id) == 0);
  // Create and store the input ports for this source id.
  SourcePorts& source_ports = input_source_ids_[source_id];
  source_ports.id_port = this->DeclareAbstractInputPort().get_index();
  source_ports.pose_port = this->DeclareAbstractInputPort().get_index();
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
                                         const GeometryFrame<T>& frame) {
  // TODO(SeanCurtis-TRI): Replace dummy frame id with actual registration and
  // use all parameters.
  unused(source_id, frame);
  THROW_IF_CONTEXT_ALLOCATED
  return FrameId::get_new_id();
}

template <typename T>
FrameId GeometrySystem<T>::RegisterFrame(SourceId source_id, FrameId parent_id,
                                         const GeometryFrame<T>& frame) {
  // TODO(SeanCurtis-TRI): Replace dummy frame id with actual registration and
  // use all parameters.
  unused(source_id, parent_id, frame);
  THROW_IF_CONTEXT_ALLOCATED
  return FrameId::get_new_id();
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterGeometry(
    SourceId source_id, FrameId frame_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  // TODO(SeanCurtis-TRI): Replace dummy geometry id with actual registration.
  // and use all parameters.
  unused(source_id, frame_id, geometry);
  THROW_IF_CONTEXT_ALLOCATED
  return GeometryId::get_new_id();
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterGeometry(
    SourceId source_id, GeometryId geometry_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  // TODO(SeanCurtis-TRI): Replace dummy geometry id with actual registration.
  // and use all parameters.
  unused(source_id, geometry_id, geometry);
  THROW_IF_CONTEXT_ALLOCATED
  return GeometryId::get_new_id();
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterAnchoredGeometry(
    SourceId source_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  // TODO(SeanCurtis-TRI): Replace dummy geometry id with actual registration.
  // and use all parameters.
  unused(source_id, geometry);
  THROW_IF_CONTEXT_ALLOCATED
  return GeometryId::get_new_id();
}

template <typename T>
void GeometrySystem<T>::ClearSource(SourceId source_id) {
  // TODO(SeanCurtis-TRI): Actually do the work.
  unused(source_id);
  THROW_IF_CONTEXT_ALLOCATED
}

template <typename T>
void GeometrySystem<T>::RemoveFrame(SourceId source_id, FrameId frame_id) {
  // TODO(SeanCurtis-TRI): Actually do the work
  unused(source_id, frame_id);
  THROW_IF_CONTEXT_ALLOCATED
}

template <typename T>
void GeometrySystem<T>::RemoveGeometry(SourceId source_id,
                                       GeometryId geometry_id) {
  // TODO(SeanCurtis-TRI): Actually do the work.
  unused(source_id, geometry_id);
  THROW_IF_CONTEXT_ALLOCATED
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
bool GeometrySystem<T>::DoHasDirectFeedthrough(const SparsityMatrix*,
                                               int input_port,
                                               int output_port) const {
  // TODO(SeanCurtis-TRI): These parameters *will* be used in subsequent PRs.
  unused(input_port, output_port);
  // Query port has no feedthrough.
  return false;
}

template <typename T>
QueryHandle<T> GeometrySystem<T>::MakeQueryHandle(
    const systems::Context<T>& context) const {
  return QueryHandle<T>(nullptr);
}

template <typename T>
void GeometrySystem<T>::CalcQueryHandle(const Context<T>& context,
                                        QueryHandle<T>* output) const {
  output->context_ = &context;
}

template <typename T>
std::unique_ptr<LeafContext<T>> GeometrySystem<T>::DoMakeContext() const {
  // Disallow further geometry source additions.
  context_allocated_ = true;
  return std::make_unique<LeafContext<T>>();
}

template <typename T>
void GeometrySystem<T>::ThrowIfContextAllocated(
    const char* source_method) const {
  if (context_allocated_) {
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

}  // namespace geometry
}  // namespace drake
