#pragma once

#include "drake/common/constants.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/drake_optional.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/input_port_base.h"

namespace drake {
namespace systems {

template <typename T>
class System;

/// This extends InputPortBase with some scalar type-dependent methods.
///
/// @tparam T The mathematical type of the context, which must be a valid Eigen
///           scalar.
template <typename T>
class InputPort final : public InputPortBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InputPort)

  /// (Internal use only)
  /// Constructs a type-specific input port. See InputPortBase::InputPortBase()
  /// for the meaning of these parameters. The additional `system` parameter
  /// here must be the same object as the `system_base` parameter.
  // The System and SystemBase are provided separately since we don't have
  // access to System's declaration here so can't cast but the caller can.
  InputPort(InputPortIndex index, DependencyTicket ticket,
                      PortDataType data_type, int size,
                      const optional<RandomDistribution>& random_type,
                      const System<T>* system, SystemBase* system_base)
      : InputPortBase(index, ticket, data_type, size, random_type, system_base),
        system_(*system) {
    DRAKE_DEMAND(system != nullptr);
    DRAKE_DEMAND(static_cast<const void*>(system) == system_base);
  }

  /// Returns a reference to the System that owns this input port. Note that
  /// for a Diagram input port this will be the Diagram, not the LeafSystem
  /// whose input port was exported.
  // TODO(sherm1) Switch to an actual reference.
  const System<T>* get_system() const { return &system_; }

 private:
  const System<T>& system_;
};

#ifndef DRAKE_DOXYGEN_CXX
/// DEPRECATED -- use InputPort instead.
// TODO(sherm1) Remove this after 10/12/2018 (3 months).
template <typename T>
using InputPortDescriptor DRAKE_DEPRECATED("Use InputPort instead.") =
    InputPort<T>;
#endif

}  // namespace systems
}  // namespace drake
