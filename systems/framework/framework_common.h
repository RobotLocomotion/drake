#pragma once

#include "drake/common/type_safe_index.h"

namespace drake {
namespace systems {

// These types and utility methods are shared among the framework classes. In
// particular, they are shared between the System and Context classes and have
// the same meaning in both class hierarchies. A System and its Context always
// have parallel internal structure.

// TODO(sherm1) Reveal these when they are used.
#ifndef DRAKE_DOXYGEN_CXX
/** Identifies a particular source value or computation for purposes of
declaring and managing dependencies. Unique only within a given subsystem
and its corresponding subcontext. */
// This is presented as an ID to end users but is implemented internally as
// a typed integer index for fast access into the std::vector of dependency
// trackers. That's why it is named differently than the other "indexes".
using DependencyTicket = TypeSafeIndex<class DependencyTag>;

/** Serves as a unique identifier for a particular CacheEntry in a System and
the corresponding CacheEntryValue in that System's Context. This is an index
providing extremely fast constant-time access to both. */
using CacheIndex = TypeSafeIndex<class CacheTag>;

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

/** Serves as the local index for discrete state groups within a given System
and its corresponding Context. */
using DiscreteStateIndex = TypeSafeIndex<class DiscreteStateTag>;

/** Serves as the local index for abstract state variables within a given System
and its corresponding Context. */
using AbstractStateIndex = TypeSafeIndex<class AbstractStateTag>;

/** Serves as the local index for numeric parameter groups within a given System
and its corresponding Context. */
using NumericParameterIndex = TypeSafeIndex<class NumericParameterTag>;

/** Serves as the local index for abstract parameters within a given System
and its corresponding Context. */
using AbstractParameterIndex = TypeSafeIndex<class AbstractParameterTag>;
#endif

/** All system ports are either vectors of Eigen scalars, or black-box
AbstractValues which may contain any type. */
typedef enum {
  kVectorValued = 0,
  kAbstractValued = 1,
} PortDataType;

/** Port type indicating a vector value whose size is not prespecified but
rather depends on what it is connected to (not yet implemented). */
// TODO(sherm1) Implement this.
constexpr int kAutoSize = -1;

}  // namespace systems
}  // namespace drake
