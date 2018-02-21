#pragma once

#include <map>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// An element-wise wrapping block that transforms the specified indices
/// of the input signal `u` into the interval `[low, high)`.  Precisely, the
/// output element `i` is given the value:
///   outputᵢ = inputᵢ + kᵢ*(highᵢ-lowᵢ)
/// for the unique integer value `kᵢ` that lands the output in the desired
/// interval.
///
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// @ingroup primitive_systems
template <typename T>
class WrapTo : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WrapTo)

  /// Constructs a system to pass through a fixed-size input vector to the
  /// output.  Additional calls to %set_interval are required to produce any
  /// wrapping behavior.
  explicit WrapTo(int input_size);

  /// Sets the system to wrap the @p index element of the input vector to the
  /// interval `[low, high)`.  If this method is called multiple times for
  /// the same index, then only the last interval will be used.
  /// @p low and @p high should be finite.
  void set_interval(int index, const T& low, const T& high);

  /// Returns the size.
  int get_size() const { return input_size_; }

  // TODO(russt): Add a witness function for signaling wrapping discontinuities.

 private:
  void CalcWrappedOutput(const Context<T>& context,
                           BasicVector<T>* output) const;

  const int input_size_{};
  // Note that this sparse representation was implemented because many
  // use cases will only wrap a small fraction of the signals (and pass the
  // others through), and to play nicely with symbolic::Expression.
  std::map<int, std::pair<T, T>> intervals_;
};

}  // namespace systems
}  // namespace drake
