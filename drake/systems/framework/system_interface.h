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
  virtual ~AbstractSystemInterface() {}

  /// Returns the name of this System.
  virtual std::string get_name() const = 0;

 protected:
  AbstractSystemInterface() {}

 private:
  // AbstractSystemInterface objects are neither copyable nor moveable.
  AbstractSystemInterface(const AbstractSystemInterface& other) = delete;
  AbstractSystemInterface& operator=(const AbstractSystemInterface& other) =
      delete;
  AbstractSystemInterface(AbstractSystemInterface&& other) = delete;
  AbstractSystemInterface& operator=(AbstractSystemInterface&& other) = delete;
};

/// A superclass template for systems that receive input, maintain state, and
/// produce output of a given mathematical type T.
///
/// Do not write concrete classes that inherit directly from SystemInterface.
/// Instead, use a more specific interface in the *SystemInterface family.
template <typename T>
class SystemInterface : public AbstractSystemInterface {
 public:
  virtual ~SystemInterface() {}

  // Returns a default context, initialized with the correct
  // numbers of concrete input ports and state variables for this System.
  // Since input port pointers are not owned by the Context, they should
  // simply be initialized to nullptr.
  virtual Context<T> CreateDefaultContext() const = 0;

  // Returns a default output, initialized with the correct number of
  // concrete output ports for this System.
  virtual SystemOutput<T> CreateDefaultOutput() const = 0;

  // Computes the output for the given context, possibly updating values
  // in the cache.
  virtual void Output(const Context<T>& context, Cache<T>* cache,
                      SystemOutput<T>* output) const = 0;

 protected:
  SystemInterface() {}

 private:
  // SystemInterface objects are neither copyable nor moveable.
  SystemInterface(const SystemInterface<T>& other) = delete;
  SystemInterface& operator=(const SystemInterface<T>& other) = delete;
  SystemInterface(SystemInterface<T>&& other) = delete;
  SystemInterface& operator=(SystemInterface<T>&& other) = delete;
};

}  // namespace systems
}  // namespace drake
