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

#ifndef DRAKE_DOXYGEN_CXX
class AbstractValue;
class ContextBase;

// (Stub, please ignore) An empty type-agnostic base class for input ports.
// TODO(sherm1) Replace with the real InputPortBase in its own header (see
// caching branch).
class InputPortBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InputPortBase)
  InputPortBase() = default;
  virtual ~InputPortBase() = default;
};

namespace internal {

// TODO(sherm1) These interface classes shouldn't be here -- split into their
// own headers. As written they obscure the limited use of these interfaces
// in the framework, and require forward declarations that would not be
// required if moved.

// SystemBase should implement this interface so that its contained objects
// can provide helpful error messages and log diagnostics that identify the
// offending object within a diagram. Providing this as a separate interface
// allows us to avoid mutual dependencies between the containers and their
// contained objects, which allows us to put the contained objects in their
// own Bazel libraries.
class SystemMessageInterface {
 public:
  virtual ~SystemMessageInterface() = default;

  // Returns a human-readable simple name of this subsystem, suitable for use
  // in constructing a system pathname. If there is no name this should still
  // return some non-empty placeholder name.
  virtual const std::string& GetSystemName() const = 0;

  // Generates and returns the full path name of this subsystem, starting at
  // the root of the containing Diagram, with path name separators between
  // segments. The individual segment names should come from GetSystemName().
  virtual std::string GetSystemPathname() const = 0;

  // Returns the concrete type of this subsystem. This should be the
  // namespace-decorated human-readable name as returned by NiceTypeName.
  virtual std::string GetSystemType() const = 0;

  // Throws an std::logic_error if the given Context is incompatible with
  // this System. This is likely to be _very_ expensive and should generally be
  // done only in Debug builds, like this:
  // @code
  //    DRAKE_ASSERT_VOID(ThrowIfContextNotCompatible(context));
  // @endcode
  virtual void ThrowIfContextNotCompatible(const ContextBase&) const = 0;

 protected:
  SystemMessageInterface() = default;
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SystemMessageInterface);
};

// ContextBase should implement this interface so that its contained objects
// can provide helpful error messages and log diagnostics that identify the
// offending object within a Diagram, in code where only the Context (no System)
// is available. (Diagram Systems and their Contexts have identical
// substructure.) See SystemMessageInterface for motivation.
class ContextMessageInterface {
 public:
  virtual ~ContextMessageInterface() = default;

  // Returns a human-readable simple name of this subcontext's corresponding
  // subsystem, suitable for use in constructing a system pathname. If there is
  // no name this should still return some non-empty placeholder name, using
  // the same method as SystemMessageInterface does.
  virtual const std::string& GetSystemName() const = 0;

  // Generates and returns the full path name of this subsystem, starting at
  // the root of the containing Diagram, with path name separators between
  // segments. The individual segment names should come from GetSystemName().
  virtual std::string GetSystemPathname() const = 0;

 protected:
  ContextMessageInterface() = default;
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContextMessageInterface);
};

// A System that contains child subsystems should implement this to provide
// services that the children can request. In current practice, this is
// only implemented by Diagram<T>. This allows us to expose just necessary
// Diagram functionality to Leaf subsystems.
class SystemParentServiceInterface {
 public:
  virtual ~SystemParentServiceInterface() = default;

  // This method is invoked when we need to evaluate a connected input port of
  // a child subsystem of this System. This need arises only if this System
  // contains subsystems, and the framework promises only to invoke this method
  // under that circumstance. Hence Diagram must implement this to evaluate the
  // connected-to output port (likely belonging to a different child subsystem)
  // and returning its value as the value for the given input port.
  virtual const AbstractValue* EvalConnectedSubsystemInputPort(
      const ContextBase& context,
      const InputPortBase& input_port) const = 0;

  // Generates and returns the full path name of the parent subsystem, starting
  // at the root of the containing Diagram, with path name separators between
  // segments. The returned string must be what would be returned by invoking
  // GetSystemPathname() on the parent subsystem. (See SystemMessageInterface
  // above.)
  virtual std::string GetParentPathname() const = 0;

 protected:
  SystemParentServiceInterface() = default;
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SystemParentServiceInterface);
};

// These dependency ticket numbers are common to all systems and contexts so
// are defined here. Actual ticket objects are created from these integers.
// Ticket numbers for conditionally-allocated objects like ports and cache
// entries are allocated beginning with kNextAvailableTicket defined below.
enum BuiltInTicketNumbers {
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

// Returns a vector of raw pointers that correspond placewise with the
// unique_ptrs in the vector `in`.
template<typename U>
std::vector<U*> Unpack(const std::vector<std::unique_ptr<U>>& in) {
  std::vector<U*> out(in.size());
  std::transform(in.begin(), in.end(), out.begin(),
                 [](const std::unique_ptr<U>& p) { return p.get(); });
  return out;
}

// Checks a vector of pointer-like objects to make sure no entries are null,
// return false otherwise. Use this as a Debug-only check:
// @code{.cpp}
//   std::vector<Thing*> things;
//   std::vector<std::unique_ptr<Thing> owned_things;
//   DRAKE_ASSERT(internal::IsNonNull(things));
//   DRAKE_ASSERT(internal::IsNonNull(owned_things);
// @endcode
// This function can be applied to an std::vector of any type T that can be
// meaningfully compared to `nullptr`.
template <typename PtrType>
bool IsNonNull(const std::vector<PtrType>& pointers) {
  for (const PtrType& p : pointers)
    if (p == nullptr) return false;
  return true;
}

}  // namespace internal
#endif

}  // namespace systems
}  // namespace drake
