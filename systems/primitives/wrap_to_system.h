#pragma once

#include <map>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// An element-wise wrapping block that transforms the specified indices
/// of the input signal `u` into the interval `[low, high)`.  Precisely, the
/// output element `i` is given the value:
/// @code
///   outputᵢ = inputᵢ + kᵢ*(highᵢ-lowᵢ)
/// @endcode
/// for the unique integer value `kᵢ` that lands the output in the desired
/// interval.
///
/// @tparam_default_scalar
/// @ingroup primitive_systems
template <typename T>
class WrapToSystem final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WrapToSystem)

  /// Constructs a system to pass through a fixed-size input vector to the
  /// output.  Additional calls to set_interval() are required to produce any
  /// wrapping behavior.
  explicit WrapToSystem(int input_size);

  /// Sets the system to wrap the @p index element of the input vector to the
  /// interval `[low, high)`.  If this method is called multiple times for
  /// the same index, then only the last interval will be used.
  /// @p low and @p high should be finite, and low < high.
  void set_interval(int index, const T& low, const T& high);

  /// Returns the size.
  int get_size() const { return input_size_; }

  // TODO(russt): Add a witness function for signaling wrapping discontinuities.

 private:
  void CalcWrappedOutput(const Context<T>& context,
                         BasicVector<T>* output) const;

  const int input_size_{};

  struct Interval {
    T low{};
    T high{};
  };

  // Note that this sparse representation was implemented because many
  // use cases will only wrap a small fraction of the signals (and pass the
  // others through), and to play nicely with symbolic::Expression (since
  // comparing against a T value, e.g. inf, in a dense representation would
  // insert false branches in the symbolic computation).
  std::map<int, Interval> intervals_;
};

}  // namespace systems
}  // namespace drake
