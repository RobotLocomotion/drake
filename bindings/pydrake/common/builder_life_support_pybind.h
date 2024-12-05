#pragma once

#include "drake/bindings/pydrake/pydrake_pybind.h"
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
  static constexpr char kKey[] = "_pydrake_internal_life_support";

  // Store a strong reference to the python builder into the c++ builder's life
  // support attributes.
  static void stash(systems::DiagramBuilder<T>* builder) {
    BuilderLifeSupport<T>::attrs(builder).emplace(kKey, py::cast(builder));
  }

  // Delete a previously stored strong reference to the python builder from the
  // c++ builder's life support attributes.
  static void abandon(systems::DiagramBuilder<T>* builder) {
    BuilderLifeSupport<T>::attrs(builder).erase(kKey);
  }

  static auto& attrs(systems::DiagramBuilder<T>* builder) {
    return builder->get_mutable_life_support().attributes;
  }
};

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
