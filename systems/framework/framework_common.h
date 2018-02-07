#pragma once

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
// TODO(sherm1) Use this.
using SubsystemIndex = TypeSafeIndex<class SubsystemIndexTag>;

/** Serves as a local index of a DependencyTracker within a particular
subcontext, providing fast access to that tracker. Unique only within a given
subcontext. */
using DependencyTicket = TypeSafeIndex<class DependencyTrackerTag>;

/** Serves as a unique identifier of a particular CacheEntry in a System and the
corresponding CacheEntryValue in that System's Context. This is an index
providing extremely fast constant-time access to both. */
using CacheIndex = TypeSafeIndex<class CacheValueTag>;

constexpr int kAutoSize = -1;

/** All system ports are either vectors of Eigen scalars, or black-box
AbstractValues which may contain any type. */
typedef enum {
  kVectorValued = 0,
  kAbstractValued = 1,
} PortDataType;

}  // namespace systems
}  // namespace drake
