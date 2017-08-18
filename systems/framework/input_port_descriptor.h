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
class InputPortDescriptor : public InputPortBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InputPortDescriptor)

  ~InputPortDescriptor() override {}

  /// @param index The index of the input port described, starting from zero and
  ///              incrementing by one per port.
  /// @param data_type Whether the port described is vector or abstract valued.
  /// @param size If the port described is vector-valued, the number of
  ///             elements, or kAutoSize if determined by connections.
  /// @param random_type Input ports may optionally be labeled as random, if the
  ///                    port is intended to model a random-source "noise" or
  ///                    "disturbance" input.
  /// @param system The system to which this descriptor belongs. Retained
  ///               internally so must outlive this port.
  InputPortDescriptor(InputPortIndex index, PortDataType data_type, int size,
                      const optional<RandomDistribution>& random_type,
                      SystemBase* system)
      : InputPortBase(index, data_type, size, random_type, system) {}

  /// Returns a reference to the System that owns this input port. Note that
  /// for a diagram input port this will be the diagram, not the leaf system
  /// whose input port was exported. */
  // TODO(sherm1) Switch to an actual reference.
  const System<T>* get_system() const;
};

}  // namespace systems
}  // namespace drake
