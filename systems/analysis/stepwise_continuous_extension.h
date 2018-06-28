#pragma once

#include "drake/systems/analysis/continuous_extension.h"

namespace drake {
namespace systems {

/// A ContinuousExtension class interface extension, geared towards
/// step-wise construction procedures. Extensions of this kind are to be
/// built incrementally by means of discrete updates that extend its domain.
/// Nature of an update remains implementation specific.
///
/// To allow for update rectification (i.e. drop and replacement), in case it
/// fails to meet certain criteria (e.g. not within tolerances), construction
/// can be deferred to a consolidation step. In between consolidations, updates
/// can be rolled back (i.e. discarded) one by one on a last-input-first-output
/// basis. Implementations are thus encouraged to keep recent updates in a light
/// weight form, deferring heavier computations and construction of a better
/// suited representation for evaluation. As such, evaluation is bound to
/// succeed only after consolidation.
///
/// @tparam T A valid Eigen scalar type.
template <typename T>
class StepwiseContinuousExtension : public ContinuousExtension<T> {
 public:
  /// Rolls back (drops) the last update.
  /// @remarks This process is irreversible.
  /// @pre Updates have taken place since instantiation or last
  ///      consolidation (via Consolidate()).
  /// @throw std::logic_error if any of the preconditions is not met.
  virtual void Rollback() = 0;

  /// Consolidates latest updates.
  ///
  /// All updates since last call or construction are put into a form
  /// that is suitable for evaluation.
  ///
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
