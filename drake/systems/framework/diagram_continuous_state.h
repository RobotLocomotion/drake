#include <vector>

#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/supervector.h"

namespace drake {
namespace systems {

/// DiagramContinuousState is a ContinuousState consisting of Supervectors
/// over a set of constituent ContinuousStates.
///
/// @tparam T The type of the output data. Must be a valid Eigen scalar.
template <typename T>
class DiagramContinuousState : public ContinuousState<T> {
 public:
  /// Constructs a ContinuousState that is composed of other ContinuousStates,
  /// which are not owned by this object and must outlive it.
  ///
  /// The DiagramContinuousState vector xc = [q v z] will have the same
  /// ordering as the @p substates parameter, which should be the sort order of
  /// the Diagram itself. This fact is an implementation detail that should
  /// only be of interest to framework authors. Everyone else can just use
  /// Diagram<T>::GetMutableSubsystemState.
  explicit DiagramContinuousState(std::vector<ContinuousState<T>*> substates)
      : ContinuousState<T>(
            Span(substates, x_selector), Span(substates, q_selector),
            Span(substates, v_selector), Span(substates, z_selector)),
        substates_(std::move(substates)) {}

  ~DiagramContinuousState() override {}

  int get_num_substates() const { return static_cast<int>(substates_.size()); }

  /// Returns the continuous state at the given @p index. Aborts if @p index is
  /// out-of-bounds.
  const ContinuousState<T>* get_substate(int index) const {
    DRAKE_DEMAND(index >= 0 && index < get_num_substates());
    return substates_[index];
  }

  /// Returns the continuous state at the given @p index. Aborts if @p index is
  /// out-of-bounds.
  ContinuousState<T>* get_mutable_substate(int index) {
    DRAKE_DEMAND(index >= 0 && index < get_num_substates());
    return substates_[index];
  }

 private:
  // Returns a Supervector over the x, q, v, or z components of each
  // substate in @p substates, as indicated by @p selector.
  static std::unique_ptr<VectorBase<T>> Span(
      const std::vector<ContinuousState<T>*>& substates,
      std::function<VectorBase<T>*(ContinuousState<T>&)> selector) {
    std::vector<VectorBase<T>*> sub_xs;
    for (const auto& substate : substates) {
      DRAKE_DEMAND(substate != nullptr);
      sub_xs.push_back(selector(*substate));
    }
    return std::make_unique<Supervector<T>>(sub_xs);
  }

  // Returns the entire state vector in @p xc.
  // TODO(#2274) Fix this NOLINTNEXTLINE(runtime/references).
  static VectorBase<T>* x_selector(ContinuousState<T>& xc) {
    return xc.get_mutable_vector();
  }
  // Returns the generalized position vector in @p xc.
  // TODO(#2274) Fix this NOLINTNEXTLINE(runtime/references).
  static VectorBase<T>* q_selector(ContinuousState<T>& xc) {
    return xc.get_mutable_generalized_position();
  }
  // Returns the generalized velocity vector in @p xc.
  // TODO(#2274) Fix this NOLINTNEXTLINE(runtime/references).
  static VectorBase<T>* v_selector(ContinuousState<T>& xc) {
    return xc.get_mutable_generalized_velocity();
  }
  // Returns the misc continuous state vector in @p xc.
  // TODO(#2274) Fix this NOLINTNEXTLINE(runtime/references).
  static VectorBase<T>* z_selector(ContinuousState<T>& xc) {
    return xc.get_mutable_misc_continuous_state();
  }

  std::vector<ContinuousState<T>*> substates_;
};

}  // namespace systems
}  // namespace drake
