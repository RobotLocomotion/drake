#pragma once

#include <any>

#include <fmt/format.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/string_map.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {

namespace systems {
template <typename T>
class DiagramBuilder;
}  // namespace systems

namespace pydrake {
namespace internal {

/* BuilderLifeSupport provides helper functions to implement a last-resort
  defense against programs that add systems in python, and then issue a Build()
  call from c++.

  The hazard is that lifetime annotations belonging to added systems get
  deleted early, leaving invalid pointers (see #14355). If python calls
  Build(), the diagram, builder, and systems are all joined by
  garbage-collectible ref-cycles, and will remain alive until the next
  collection after losing reachability. If c++ calls Build(), then the
  builder-diagram cycle doesn't get added, and then builder, systems, and their
  associates are at risk.

  BuilderLifeSupport helps by exposing the ability (via stash()) to place a
  strong reference to the python builder in the ownership of the c++
  builder. The c++ builder will automatically transfer that reference to the
  diagram, so that a c++ Build() call will still result in a one-way reference
  from the c++ diagram to the python builder and its associates.

  For mixed-language programs, the one-way reference is typically good
  enough. However, for python programs, the Build() annotations are strictly
  better. Using both is bad, since it leads to diagram immortality (see
  #14387). Python Build() avoids immortality by calling abandon() to release
  the extra strong reference. */
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

  // (Internal use only) Return a reference to a map of attributes stored in
  // life support.
  static string_map<std::any>& attrs(systems::DiagramBuilder<T>* builder) {
    return builder->get_mutable_life_support().attributes;
  }
};

/* pydrake::internal::builder_life_support_stash is a custom call policy for
  pybind11.

  For an overview of other call policies, See
  https://pybind11.readthedocs.io/en/stable/advanced/functions.html#additional-call-policies

  `builder_life_support_stash` applies the equivalent of
  BuilderLifeSupport::stash(), as a post-call step. Note that it stores a
  strong reference to the builder's python wrapper, in a place inaccessible to
  garbage collection. To avoid leaks, the complete builder workflow should call
  BuilderLifeSupport::abandon() before the actual build step.

  @tparam T the scalar type in use for DiagramBuilder
  @tparam Builder an argument index

  The argument index starts at 1; for methods, `self` is at index 1. Index 0
  denotes the return value.
*/
template <typename T, size_t Builder>
struct builder_life_support_stash {};

template <typename T>
void builder_life_support_stash_impl(size_t builder_index,
    const py::detail::function_call& call, py::handle ret) {
  // Returns the handle selected by the given index. Throws if the index is
  // invalid.
  auto get_arg = [&](size_t n) -> py::handle {
    if (n == 0) {
      return ret;
    }
    if (n == 1 && call.init_self) {
      return call.init_self;
    }
    if (n <= call.args.size()) {
      return call.args[n - 1];
    }
    py::pybind11_fail(
        fmt::format("Could not activate builder_life_support_stash: index {} "
                    "is invalid for function '{}'",
            n, call.func.name));
  };
  py::handle builder_handle = get_arg(builder_index);
  if (builder_handle.is_none()) {
    // Nothing useful to stash.
    return;
  }
  // Convert the handle to a strong reference for later stashing.
  py::object py_builder = py::cast<py::object>(builder_handle);
  // Recover the c++ pointer; pybind11 will throw if the cast can't work.
  systems::DiagramBuilder<T>* cc_builder =
      py::cast<systems::DiagramBuilder<T>*>(py_builder);
  DRAKE_ASSERT(cc_builder != nullptr);
  // Do the equivalent of stash(); we don't use the method since we've already
  // had to recover the python and c++ objects in a different order.
  BuilderLifeSupport<T>::attrs(cc_builder)
      .emplace(BuilderLifeSupport<T>::kKey, py_builder);
}

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
  static void postcall(function_call& call, handle ret) {
    drake::pydrake::internal::builder_life_support_stash_impl<T>(
        Builder, call, ret);
  }
};

}  // namespace detail
}  // namespace pybind11
