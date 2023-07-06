#pragma once

#include <vector>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/geometry/optimization/convex_set.h"

namespace drake {
namespace pydrake {

/** Deep-copies the ConvexSet pointers in the given list into the C++-compatible
object ConvexSets (which uses copyable_unique_ptr ownership). This is useful to
accept Python-natural function arguments (list of pointers) and then call the
C++ API that requires a more complicated type. */
geometry::optimization::ConvexSets CloneConvexSets(
    const std::vector<geometry::optimization::ConvexSet*>& sets_in) {
  geometry::optimization::ConvexSets sets;
  sets.reserve(sets_in.size());
  for (const geometry::optimization::ConvexSet* set : sets_in) {
    if (set == nullptr) {
      sets.emplace_back(nullptr);
    } else {
      sets.emplace_back(set->Clone());
    }
  }
  return sets;
}

}  // namespace pydrake
}  // namespace drake
