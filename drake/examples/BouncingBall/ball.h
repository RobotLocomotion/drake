#pragma once

#include <memory>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace bouncingball {

/// This class uses Drake's `-inl.h` pattern.  When seeing linker errors from
/// this class, please refer to http://drake.mit.edu/cxx_inl.html.
///
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in drakeSystemFramework.
///
/// To use other specific scalar types see ball-inl.h.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class Ball : public systems::LeafSystem<T> {
 public:
  /// Constructs a Ball system.
  Ball();

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
