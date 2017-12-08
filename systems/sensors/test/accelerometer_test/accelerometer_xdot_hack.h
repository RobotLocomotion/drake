#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"

namespace drake {
namespace systems {
namespace sensors {

// TODO(liang.fok): Remove this system once RigidBodyPlant is able to output
// xdot.

/// Implements a system with one input port and one output port, both vector
/// valued. If the input port contains a vector with non-finite values, this
/// system outputs a vector containing zeros. Otherwise it outputs the same
/// value as the input. This is useful for providing a valid `xdot` for the
/// accelerometer, which will contain non-finite values during the first
/// simulation tick.
///
class AccelerometerXdotHack : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AccelerometerXdotHack);

  /// The constructor.
  ///
  /// @param[in] port_size The size of the input and output ports.
  ///
  explicit AccelerometerXdotHack(int port_size);

  /// @name System input port descriptor accessors.
  ///@{

  /// Returns a descriptor of the input port. The size of this port is
  /// equal to the `port_size` parameter provided to the constructor.
  const InputPortDescriptor<double>& get_input_port() const {
    return System<double>::get_input_port(input_port_index_);
  }
  ///@}

  /// @name System output port accessors.
  ///@{

  /// Returns the output port. The size of this port's output vector is
  /// equal to the `port_size` parameter provided to the constructor.
  const OutputPort<double>& get_output_port() const {
    return System<double>::get_output_port(output_port_index_);
  }
  ///@}

 private:
  // Filters the input vector and outputs the results.
  void CalcXdotOutput(const systems::Context<double>& context,
                      systems::BasicVector<double>* output) const;

  int input_port_index_{};
  int output_port_index_{};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
