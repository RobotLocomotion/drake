#pragma once

#include <memory>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace bouncingball {

/// A gain block with input `u` and output `y = k*u` with `k` a constant.
/// The input to this system directly feeds through to its output.
///
/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to http://drake.mit.edu/cxx_inl.html.
///
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in drakeSystemFramework.
///
/// To use other specific scalar types see gain-inl.h.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
  class Ball : public systems::LeafSystem<T> {
 public:
  /// Constructs a Ball system
  Ball();

  /// Sets the output port value to the product of the gain and the input port
  /// value. The gain is specified in the constructor.
  /// If number of connected input or output ports differs from one or, the
  /// input ports are not of size length_, std::runtime_error will be thrown.
  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override;

  void EvalTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

 protected:
    std::unique_ptr<systems::ContinuousState<T>> AllocateContinuousState()
      const override;
    std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::SystemPortDescriptor<T>& descriptor) const override;
};

}  // namespace bouncingball
}  // namespace drake
