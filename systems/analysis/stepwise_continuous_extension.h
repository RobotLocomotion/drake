#pragma once

#include "drake/systems/analysis/continuous_extension.h"

namespace drake {
namespace systems {

/// A ContinuousExtension class interface extension, geared towards
/// step-wise construction processes. Steps can be rolled back one by
/// one on a last-input-first-output basis until consolidation takes place.
/// Implementations are thus encouraged to keep recent updates in a light weight
/// form, deferring heavier computations and construction of a better suited
/// form for evaluation to the consolidation step. The exact form of the updates
/// remains implementation specific.
///
/// @tparam T A valid Eigen scalar type.
template <typename T>
class StepwiseContinuousExtension : public ContinuousExtension<T> {
 public:
  /// Rolls back the last update.
  /// @remarks This process is irreversible.
  /// @pre Updates have taken place since instantiation or last
  ///      consolidation (via Consolidate()).
  /// @throw std::logic_error if any of the preconditions is not met.
  virtual void Rollback() = 0;

  /// Consolidates latest updates.
  /// @remarks This process is irreversible.
  /// @pre Updates have taken place since instantiation or last
  ///      consolidation.
  /// @post The extents covered by updates since instantiation or
  ///       last consolidation can be evaluated (via Evaluate()).
  /// @post Time extents covered by updates can be evaluated
  ///       (via get_start_time()/get_end_time()).
  /// @throw std::logic_error if any of the preconditions is not met.
  virtual void Consolidate() = 0;
};

}  // namespace systems
}  // namespace drake
