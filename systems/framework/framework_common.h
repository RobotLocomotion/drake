#pragma once

#include <algorithm>
#include <memory>
#include <string>
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

#ifndef DRAKE_DOXYGEN_CXX
namespace internal {

/** Any class that can provide a System name and (for Diagrams) a subsystem
path name should implement this interface. This is used by System and Context
so that contained objects can provide helpful error messages and log
diagnostics that identify the offending object within a diagram. (Diagram
Systems and their Contexts have identical substructure.) Providing
this as an separate interface allows us to avoid circular dependencies between
the containers and their contained objects. */
class SystemPathnameInterface {
 public:
  virtual ~SystemPathnameInterface() = default;

  /** Returns the simple name of this subsystem, with no path separators. */
  virtual std::string GetSystemName() const = 0;

  /** Returns the full path name of this subsystem, starting at the root
  of the containing Diagram, with path name separators between segments. */
  virtual std::string GetSystemPathname() const = 0;
};

/** These dependency ticket numbers are common to all systems and contexts so
are defined here. Actual ticket objects are created from these integers.
Ticket numbers for conditionally-allocated objects like ports and cache
entries are allocated beginning with kNextAvailableTicket defined below. */
enum WellKnownTicketNumbers {
  kNothingTicket        =  0,
  kTimeTicket           =  1,
  kAccuracyTicket       =  2,
  kQTicket              =  3,
  kVTicket              =  4,
  kZTicket              =  5,
  kXcTicket             =  6,
  kXdTicket             =  7,
  kXaTicket             =  8,
  kXTicket              =  9,
  kConfigurationTicket  = 10,
  kVelocityTicket       = 11,
  kKinematicsTicket     = 12,
  kAllParametersTicket  = 13,
  kAllInputPortsTicket  = 14,
  kAllSourcesTicket     = 15,
  kXcdotTicket          = 16,
  kXdhatTicket          = 17,
  kNextAvailableTicket  = kXdhatTicket+1
};

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
#endif

}  // namespace systems
}  // namespace drake
