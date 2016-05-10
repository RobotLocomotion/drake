#pragma once

#include <string>

#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/cache.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/system_output.h"

namespace drake {
namespace systems {

/// A fully type-erased abstract superclass for dynamical systems.
///
/// Do not write concrete classes that inherit directly from
/// AbstractSystemInterface. Instead, use a more specific interface in the
/// *SystemInterface family.
class DRAKESYSTEMFRAMEWORK_EXPORT AbstractSystemInterface {
 public:
  AbstractSystemInterface() {}
  virtual ~AbstractSystemInterface() {}

  /// Returns the name of this System.
  virtual std::string get_name() const = 0;

 private:
  // AbstractSystemInterface objects are neither copyable nor moveable.
  AbstractSystemInterface(const AbstractSystemInterface& other) = delete;
  AbstractSystemInterface& operator=(const AbstractSystemInterface& other) =
      delete;
  AbstractSystemInterface(AbstractSystemInterface&& other) = delete;
  AbstractSystemInterface& operator=(AbstractSystemInterface&& other) = delete;
};

/// A superclass template for systems that receive input, maintain state, and
/// produce output of a given ScalarType.
///
/// Do not write concrete classes that inherit directly from SystemInterface.
/// Instead, use a more specific interface in the *SystemInterface family.
template <typename ScalarType>
class SystemInterface : public AbstractSystemInterface {
 public:
  SystemInterface() {}
  virtual ~SystemInterface() {}

  // Returns a default context, initialized with the correct
  // numbers of concrete input ports and state variables for this System.
  // Since input port pointers are not owned by the Context, they should
  // simply be initialized to nullptr.
  virtual Context<ScalarType> CreateDefaultContext() const = 0;

  // Returns a default output, initialized with the correct number of
  // concrete output ports for this System.
  virtual SystemOutput<ScalarType> CreateDefaultOutput() const = 0;

  // Computes the output for the given context, possibly updating values
  // in the cache.
  virtual void Output(const Context<ScalarType>& context,
                      Cache<ScalarType>* cache,
                      SystemOutput<ScalarType>* output) const = 0;

 private:
  // SystemInterface objects are neither copyable nor moveable.
  SystemInterface(const SystemInterface<ScalarType>& other) = delete;
  SystemInterface& operator=(const SystemInterface<ScalarType>& other) = delete;
  SystemInterface(SystemInterface<ScalarType>&& other) = delete;
  SystemInterface& operator=(SystemInterface<ScalarType>&& other) = delete;
};

}  // namespace systems
}  // namespace drake
