#pragma once

#include <string>
#include <utility>

#include "drake/common/constants.h"
#include "drake/common/default_scalars.h"
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
  /// here must be the same object as the `system_base` parameter. The
  /// `name` must not be empty.
  // The System and SystemBase are provided separately since we don't have
  // access to System's declaration here so can't cast but the caller can.
  InputPort(const System<T>* system, SystemBase* system_base, std::string name,
            InputPortIndex index, DependencyTicket ticket,
            PortDataType data_type, int size,
            const optional<RandomDistribution>& random_type)
      : InputPortBase(system_base, std::move(name), index, ticket, data_type,
                      size, random_type),
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

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::InputPort)
