#pragma once

#include "drake/common/constants.h"
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
class InputPortDescriptor final: public InputPortBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InputPortDescriptor)

  ~InputPortDescriptor() final {}

  /// (Internal use only)
  /// @param index The index of the input port described, starting from zero and
  ///              incrementing by one per port.
  /// @param data_type Whether the port described is vector or abstract valued.
  /// @param size If the port described is vector-valued, the number of
  ///             elements, or kAutoSize if determined by connections.
  /// @param random_type Input ports may optionally be labeled as random, if the
  ///                    port is intended to model a random-source "noise" or
  ///                    "disturbance" input.
  /// @param system The System<T> to which this descriptor belongs. Retained
  ///               internally so must outlive this port.
  /// @param system_base The SystemBase that owns this input port. Must be
  ///                    the same object as referred to by `system`.
  // The System and SystemBase are provided separately since we don't have
  // access to System's declaration here so can't cast but the caller can.
  InputPortDescriptor(InputPortIndex index, PortDataType data_type, int size,
                      const optional<RandomDistribution>& random_type,
                      const System<T>* system, SystemBase* system_base)
      : InputPortBase(index, data_type, size, random_type, system_base),
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
