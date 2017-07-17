#include "drake/geometry/geometry_system.h"

#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace geometry {

using systems::Context;
using systems::LeafContext;
using systems::LeafSystem;
using systems::SparsityMatrix;
using systems::SystemOutput;
using std::vector;

template <typename T>
GeometrySystem<T>::~GeometrySystem() {}

template <typename T>
SourceId GeometrySystem<T>::RegisterSource(const std::string &name) {
  unused(name);
  if (!context_allocated_) {
    // TODO(SeanCurtis-TRI): Replace dummy source id with actual id.
    return SourceId::get_new_id();
  } else {
    throw std::logic_error(
        "A context has been created for this system. Adding new geometry "
        "sources is no longer possible.");
  }
}

template <typename T>
FrameId GeometrySystem<T>::RegisterFrame(SourceId source_id,
                                         const GeometryFrame<T>& frame) {
  // TODO(SeanCurtis-TRI): Replace dummy frame id with actual registration and
  // use all parameters.
  unused(source_id, frame);
  ThrowIfContextAllocated();
  return FrameId::get_new_id();
}

template <typename T>
FrameId GeometrySystem<T>::RegisterFrame(SourceId source_id, FrameId parent_id,
                                         const GeometryFrame<T>& frame) {
  // TODO(SeanCurtis-TRI): Replace dummy frame id with actual registration and
  // use all parameters.
  unused(source_id, parent_id, frame);
  ThrowIfContextAllocated();
  return FrameId::get_new_id();
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterGeometry(
    SourceId source_id, FrameId frame_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  // TODO(SeanCurtis-TRI): Replace dummy geometry id with actual registration.
  // and use all parameters.
  unused(source_id, frame_id, geometry);
  ThrowIfContextAllocated();
  return GeometryId::get_new_id();
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterGeometry(
    SourceId source_id, GeometryId geometry_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  // TODO(SeanCurtis-TRI): Replace dummy geometry id with actual registration.
  // and use all parameters.
  unused(source_id, geometry_id, geometry);
  ThrowIfContextAllocated();
  return GeometryId::get_new_id();
}

template <typename T>
GeometryId GeometrySystem<T>::RegisterAnchoredGeometry(
    SourceId source_id,
    std::unique_ptr<GeometryInstance<T>> geometry) {
  // TODO(SeanCurtis-TRI): Replace dummy geometry id with actual registration.
  // and use all parameters.
  unused(source_id, geometry);
  ThrowIfContextAllocated();
  return GeometryId::get_new_id();
}

template <typename T>
void GeometrySystem<T>::ClearSource(SourceId source_id) {
  // TODO(SeanCurtis-TRI): Actually do the work.
  unused(source_id);
  ThrowIfContextAllocated();
}

template <typename T>
void GeometrySystem<T>::RemoveFrame(SourceId source_id, FrameId frame_id) {
  // TODO(SeanCurtis-TRI): Actually do the work
  unused(source_id, frame_id);
  ThrowIfContextAllocated();
}

template <typename T>
void GeometrySystem<T>::RemoveGeometry(SourceId source_id,
                                       GeometryId geometry_id) {
  // TODO(SeanCurtis-TRI): Actually do the work.
  unused(source_id, geometry_id);
  ThrowIfContextAllocated();
}

template <typename T>
std::unique_ptr<LeafContext<T>> GeometrySystem<T>::DoMakeContext() const {
  // Disallow further geometry source additions.
  context_allocated_ = true;
  return std::make_unique<LeafContext<T>>();
}

template <typename T>
void GeometrySystem<T>::ThrowIfContextAllocated() const {
  if (context_allocated_)
    throw std::logic_error("Operation invalid; a context has already been "
                           "allocated.");
}

// Explicitly instantiates on the most common scalar types.
template class GeometrySystem<double>;

}  // namespace geometry
}  // namespace drake
