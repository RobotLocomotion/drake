# pragma once

#include "drake/systems/framework/primitives/linear_system_plant.h"

namespace drake {
namespace systems {

/// A MIMO (Multi-Input Multi-Output) gain system that is a specialization of a
/// LinearSystemPlant where coefficient matrices `A`, `B, and `C` are all zero.
/// Given an input signal `u` and a state `x` the output of this system, 'y',
/// is:
///
/// <pre>
///   y = Du
/// </pre>
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in libdrakeSystemFramework.
/// No other values for T are currently supported.
/// @ingroup primitive_systems
template<typename T>
class MimoGain: public LinearSystemPlant<T> {
 public:
  MimoGain(const Eigen::Ref<const MatrixX<T>> &D);
};

}  // namespace systems
}  // namespace drake
