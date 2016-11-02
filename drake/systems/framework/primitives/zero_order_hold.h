#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/// A ZeroOrderHold block with input `u`, which may be discrete or continuous,
/// and discrete output `y`, where the y is sampled from u with a fixed period.
/// @ingroup primitive_systems
template <typename T>
class ZeroOrderHold : public LeafSystem<T> {
 public:
  /// Constructs a ZeroOrderHold system with the given @p period_sec, over a
  /// vector-valued input of size @p size.
  ZeroOrderHold(const T& period_sec, int size);

  // In a zero-order hold, the output depends only on the state, so there is
  // no direct-feedthrough.
  bool has_any_direct_feedthrough() const override { return false; }

  /// Sets the output port value to the value that is currently latched in the
  /// zero-order hold.
  void EvalOutput(const Context<T>& context,
                  SystemOutput<T>* output) const override;

 protected:
  /// Latches the input port into the discrete state.
  void DoEvalDifferenceUpdates(
      const Context<T>& context,
      DifferenceState<T>* difference_state) const override;

  std::unique_ptr<DifferenceState<T>> AllocateDifferenceState() const override;
};

}  // namespace systems
}  // namespace drake
