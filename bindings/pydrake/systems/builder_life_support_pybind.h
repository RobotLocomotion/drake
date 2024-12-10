#pragma once

#include <any>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/string_map.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace pydrake {
namespace internal {

// BuilderLifeSupport provides helper functions to implement a last-resort
// defense against programs that add systems in python, and then issue a
// Build() call from c++.
//
// The hazard is that lifetime annotations belonging to added systems get
// deleted early, leaving invalid pointers (see #14355). If python calls
// Build(), the diagram, builder, and systems are all joined by
// garbage-collectible ref-cycles, and will remain alive until the next
// collection after losing reachability. If c++ calls Build(), then the
// builder-diagram cycle doesn't get added, and then builder, systems, and
// their associates are at risk.
//
// BuilderLifeSupport helps by exposing the ability (via stash()) to place a
// strong reference to the python builder in the ownership of the c++
// builder. The c++ builder will automatically transfer that reference to the
// diagram, so that a c++ Build() call will still result in a one-way reference
// from the c++ diagram to the python builder and its associates.
//
// For mixed-language programs, the one-way reference is typically good
// enough. However, for python programs, the Build() annotations are strictly
// better. Using both is bad, since it leads to diagram immortality (see
// #14387). Python Build() avoids immortality by calling abandon() to release
// the extra strong reference.
template <typename T>
struct BuilderLifeSupport {
  // Store a strong reference to the python builder into the c++ builder's life
  // support attributes.
  static void stash(systems::DiagramBuilder<T>* builder);

  // Delete a previously stored strong reference to the python builder from the
  // c++ builder's life support attributes.
  static void abandon(systems::DiagramBuilder<T>* builder);

  // (Internal use only) Return a reference to a map of attributes stored in
  // life support.
  static string_map<std::any>& attrs(systems::DiagramBuilder<T>* builder);
};

template <typename T, size_t Builder>
struct builder_life_support_stash {};

template <typename T>
void builder_life_support_stash_impl(size_t builder_index,
    const py::detail::function_call& call, py::handle ret);

}  // namespace internal
}  // namespace pydrake
}  // namespace drake

namespace pybind11 {
namespace detail {

// Provide a specialization of the pybind11 internal process_attribute
// template; this allows writing an annotation that works seamlessly in
// bindings definitions.
template <typename T, size_t Builder>
class process_attribute<
    drake::pydrake::internal::builder_life_support_stash<T, Builder>>
    : public process_attribute_default<
          drake::pydrake::internal::builder_life_support_stash<T, Builder>> {
 public:
  // NOLINTNEXTLINE(runtime/references)
  static void precall(function_call& call) {}

  // NOLINTNEXTLINE(runtime/references)
  static void postcall(function_call& call, handle ret) {
    drake::pydrake::internal::builder_life_support_stash_impl<T>(
        Builder, call, ret);
  }
};

}  // namespace detail
}  // namespace pybind11
