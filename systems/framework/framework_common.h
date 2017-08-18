#pragma once

#include <algorithm>
#include <memory>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/type_safe_index.h"

namespace drake {
namespace systems {

// These types and utility methods are shared among the framework classes. In
// particular, they are shared between the System and Context classes and have
// the same meaning in both class hierarchies. A System and its Context always
// have parallel internal structure.

/** Serves as a local index for a child subsystem within a parent
Diagram, or a child subcontext within a parent DiagramContext. A subsystem and
its matching subcontext have the same %SubsystemIndex. Unique only
within a given subsystem or subcontext. */
using SubsystemIndex = TypeSafeIndex<class SubsystemIndexTag>;

/** Serves as the local index for the input ports of a given System. The
indexes used by a subsystem and its corresponding subcontext are the same. */
using InputPortIndex = TypeSafeIndex<class InputPortTag>;

/** Serves as the local index for the output ports of a given System. The
indexes used by a subsystem and its corresponding subcontext are the same. */
using OutputPortIndex = TypeSafeIndex<class OutputPortTag>;

/** Serves as a local index of a DependencyTracker within a particular
subcontext, providing fast access to that tracker. Unique only within a given
subcontext. */
using DependencyTicket = TypeSafeIndex<class DependencyTrackerTag>;

/** Serves as a unique identifier of a particular CacheEntry in a System and the
corresponding CacheEntryValue in that System's Context. This is an index
providing extremely fast constant-time access to both. */
using CacheIndex = TypeSafeIndex<class CacheValueTag>;

using DiscreteStateIndex = TypeSafeIndex<class DiscreteStateTag>;
using AbstractStateIndex = TypeSafeIndex<class AbstractStateTag>;
using NumericParameterIndex = TypeSafeIndex<class NumericParameterTag>;
using AbstractParameterIndex = TypeSafeIndex<class AbstractParameterTag>;

constexpr int kAutoSize = -1;

/** All system ports are either vectors of Eigen scalars, or black-box
AbstractValues which may contain any type. */
typedef enum {
  kVectorValued = 0,
  kAbstractValued = 1,
} PortDataType;


namespace internal {

// These are some utility methods that are reused within the framework.

/** Returns a vector of raw pointers that correspond placewise with the
unique_ptrs in the vector `in`. */
template<typename U>
std::vector<U*> Unpack(const std::vector<std::unique_ptr<U>>& in) {
  std::vector<U*> out(in.size());
  std::transform(in.begin(), in.end(), out.begin(),
                 [](const std::unique_ptr<U>& p) { return p.get(); });
  return out;
}

/** Checks a vector of pointer-like objects to make sure no entries are null,
aborting if so. Use this as a Debug-only check:
@code{.cpp}
  std::vector<Thing*> things;
  std::vector<std::unique_ptr<Thing> owned_things;
  DRAKE_ASSERT_VOID(CheckNonNull(things));
  DRAKE_ASSERT_VOID(CheckNonNull(owned_things);
@endcode
This function can be applied to an std::vector of any type T that can be
meaningfully compared to `nullptr`. */
template <typename PtrType>
void CheckNonNull(const std::vector<PtrType>& pointers) {
  for (const PtrType& p : pointers)
    DRAKE_DEMAND(p != nullptr);
}

}  // namespace internal

}  // namespace systems
}  // namespace drake
