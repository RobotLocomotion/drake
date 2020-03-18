#pragma once

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/identifier.h"
#include "drake/common/type_safe_index.h"
#include "drake/common/value.h"

namespace drake {
namespace systems {

// These types and utility methods are shared among the framework classes. In
// particular, they are shared between the System and Context classes and have
// the same meaning in both class hierarchies. A System and its Context always
// have parallel internal structure.

// This is presented as an ID to end users but is implemented internally as
// a typed integer index for fast access into the std::vector of dependency
// trackers. That's why it is named differently than the other "indexes".
/** Identifies a particular source value or computation for purposes of
declaring and managing dependencies. Unique only within a given subsystem
and its corresponding subcontext. */
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

/** Serves as the local index for constraints declared on a given System. */
using SystemConstraintIndex = TypeSafeIndex<class SystemConstraintTag>;

/** All system ports are either vectors of Eigen scalars, or black-box
AbstractValues which may contain any type. */
typedef enum {
  kVectorValued = 0,
  kAbstractValued = 1,
} PortDataType;

// TODO(sherm1) Implement this.
/** Port type indicating a vector value whose size is not prespecified but
rather depends on what it is connected to (not yet implemented). */
constexpr int kAutoSize = -1;

/** (Advanced.)  Tag type that indicates a system or port should use a default
name, instead of a user-provided name.  Most users will use the kUseDefaultName
constant, without ever having to mention this type. */
struct UseDefaultName final {};

/** Name to use when you want a default one generated. You should normally
give meaningful names to all Drake System entities you create rather than
using this. */
constexpr UseDefaultName kUseDefaultName = {};

/** (Advanced.) Sugar that compares a variant against kUseDefaultName. */
inline bool operator==(
    const std::variant<std::string, UseDefaultName>& value,
    const UseDefaultName&) {
  return std::holds_alternative<UseDefaultName>(value);
}

/** Intended for use in e.g. variant<InputPortSelection, InputPortIndex> for
algorithms that support optional and/or default port indices. */
enum class InputPortSelection { kNoInput = -1, kUseFirstInputIfItExists = -2 };

/** Intended for use in e.g. variant<OutputPortSelection, OutputPortIndex> for
algorithms that support optional and/or default port indices. */
enum class OutputPortSelection { kNoOutput = -1, kUseFirstOutputIfItExists =
    -2 };

#ifndef DRAKE_DOXYGEN_CXX
class ContextBase;
class InputPortBase;
class SystemBase;

namespace internal {

// Type used to match a Context to its System.
using SystemId = drake::Identifier<class SystemIdTag>;

// A utility to call the package-private constructor of some framework classes.
class FrameworkFactory {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FrameworkFactory)
  FrameworkFactory() = delete;
  ~FrameworkFactory() = delete;

  template <typename T, typename... Args>
  static std::unique_ptr<T> Make(Args... args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
  }
};

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
  // in constructing a system pathname. If there is no name this should return
  // the string provided by no_name() below.
  virtual const std::string& GetSystemName() const = 0;

  // Generates and returns the full path name of this subsystem, starting at
  // the root of the containing Diagram, with path name separators between
  // segments. The individual segment names should come from GetSystemName(),
  // and path separators should come from path_separator() below.
  virtual std::string GetSystemPathname() const = 0;

  // Returns the concrete type of this subsystem. This should be the
  // namespace-decorated human-readable name as returned by NiceTypeName.
  virtual std::string GetSystemType() const = 0;

  // Checks whether the given context was created for this system.
  // See SystemBase::ValidateContext for full documentation.
  virtual void ValidateContext(const ContextBase& context) const = 0;

  // Use this string as a stand-in name for an unnamed System. This is not
  // a default name, just an indicator that there is no name. This is a policy
  // and should be used by both System/ContextMessageInterface.
  // TODO(sherm1) Revisit this "_" business. Maybe something like "(noname)",
  // or a unique default like DiagramBuilder uses?
  // Be sure to update implementation comments if you change this policy.
  static const std::string& no_name() {
    static never_destroyed<std::string> dummy("_");
    return dummy.access();
  }

  // Use this string as the separator in System path names. This is a policy
  // and should be used by both System/ContextMessageInterface.
  // TODO(sherm1) Change to more conventional "/" delimiter.
  // Be sure to update implementation comments if you change this policy.
  static const std::string& path_separator() {
    static never_destroyed<std::string> separator("::");
    return separator.access();
  }

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
  // the SystemMessageInterface::no_name() method.
  virtual const std::string& GetSystemName() const = 0;

  // Generates and returns the full path name of this subsystem, starting at
  // the root of the containing Diagram, with path name separators between
  // segments. The individual segment names should come from GetSystemName()
  // and the path separator from SystemMessageInterface::path_separator().
  virtual std::string GetSystemPathname() const = 0;

  // Returns true if the cache in this subcontext has been frozen.
  virtual bool is_cache_frozen() const = 0;

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

  // Returns the root Diagram at the top of this Diagram tree. Will return
  // `this` if we are already at the root.
  virtual const SystemBase& GetRootSystemBase() const = 0;

 protected:
  SystemParentServiceInterface() = default;
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SystemParentServiceInterface);
};

// These dependency ticket numbers are common to all systems and contexts so
// are defined here. Actual ticket objects are created from these integers.
// Ticket numbers for conditionally-allocated objects like ports and cache
// entries are allocated beginning with kNextAvailableTicket defined below.
enum BuiltInTicketNumbers {
  // This set of tickets represents independent source values in a Context,
  // and groupings of such source values.
  kNothingTicket        =  0,  // Indicates "not dependent on anything".
  kTimeTicket           =  1,  // Time.
  kAccuracyTicket       =  2,  // Accuracy.
  kQTicket              =  3,  // Continuous configuration variables.
  kVTicket              =  4,  // Continuous velocity variables.
  kZTicket              =  5,  // Miscellaneous continuous variables.
  kXcTicket             =  6,  // All continuous variables xc = {q, v, z}.
  kXdTicket             =  7,  // All discrete (numeric) state variables.
  kXaTicket             =  8,  // All abstract state variables.
  kXTicket              =  9,  // All state variables x = {xc, xd, xa}.
  kPnTicket             = 10,  // All numeric parameters.
  kPaTicket             = 11,  // All abstract parameters.
  kAllParametersTicket  = 12,  // All parameters p = {pn, pa}.
  kAllInputPortsTicket  = 13,  // All input ports u.
  kAllSourcesTicket     = 14,  // All of the above.
  kConfigurationTicket  = 15,  // All values that may affect configuration.
  kKinematicsTicket     = 16,  // Configuration plus velocity-affecting values.
  kLastSourceTicket     = kKinematicsTicket,  // (Used in testing.)

  // The rest of these are pre-defined computations with associated cache
  // entries.
  kXcdotTicket          = 17,  // d/dt xc = {qdot, vdot, zdot}.
  kPeTicket             = 18,  // Potential energy.
  kKeTicket             = 19,  // Kinetic energy.
  kPcTicket             = 20,  // Conservative power.
  kPncTicket            = 21,  // Non-conservative power.

  kNextAvailableTicket  = kPncTicket+1
};

// Specifies the prerequisite of an output port. It will always be either
// an internal cache entry within the subsystem that owns the output port, or
// an output port of a child subsystem that is being forwarded as an output port
// of the child's parent Diagram. If the `child_subsystem` index is missing it
// indicates that the prerequisite is internal.
struct OutputPortPrerequisite {
  std::optional<SubsystemIndex> child_subsystem;
  DependencyTicket dependency;
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
