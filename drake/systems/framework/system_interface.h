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
/// Concrete systems with no state should inherit directly from this interface.
/// Concrete systems with state should use a more specific interface in the
/// SystemInterface family.
///
/// TODO(david-german-tri): Add static_asserts on T.
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class SystemInterface : public AbstractSystemInterface {
 public:
  ~SystemInterface() override {}

  /// Returns a default context, initialized with the correct
  /// numbers of concrete input ports and state variables for this System.
  /// Since input port pointers are not owned by the Context, they should
  /// simply be initialized to nullptr.
  virtual std::unique_ptr<Context<T>> CreateDefaultContext() const = 0;

  /// Returns a default output, initialized with the correct number of
  /// concrete output ports for this System.
  virtual std::unique_ptr<SystemOutput<T>> AllocateOutput() const = 0;

  /// Computes the output for the given context, possibly updating values
  /// in the cache.
  virtual void EvalOutput(const Context<T>& context,
                          SystemOutput<T>* output) const = 0;

  // TODO(sherm): these two energy methods should be present only for systems
  // that represent some kind of physical system that can store energy in its
  // configuration or motion. Consider introducing a PhysicalSystemInterface
  // class so that a simple System (for example, an adder) doesn't have these
  // methods. For now I'm breaking the no-code-in-interface rule to provide
  // zero defaults so that these don't have to be implemented in non-physical
  // systems.

  /// Return the potential energy currently stored in the configuration provided
  /// in the given Context. Non-physical Systems will return 0.
  virtual T EvalPotentialEnergy(const Context<T>& context) const {
    return T(0);
  }

  /// Return the kinetic energy currently present in the motion provided in the
  /// given Context. Non-physical Systems will return 0.
  virtual T EvalKineticEnergy(const Context<T>& context) const { return T(0); }

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
