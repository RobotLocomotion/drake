#pragma once

#include <string>
#include <unordered_map>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
namespace internal {
/**
 * Represents a collection of quantities characterized by a sequence of symbolic
 * expressions, one for each index in [0,  num_samples].
 */
class SequentialExpressionManager {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SequentialExpressionManager);

  SequentialExpressionManager(int num_samples);

  ~SequentialExpressionManager() = default;

  /**
   * Registers `sequential_expressions` with `this`.
   * @pre `sequential_expressions` has num_samples() columns.
   * @return placeholder variable vector for use with
   * AddPlaceholderVariableSubstitutionsForIndex().
   */
  VectorX<symbolic::Variable> RegisterSequentialExpressions(
      const Eigen::Ref<const MatrixX<symbolic::Expression>>&
          sequential_expressions,
      const std::string& name);

  /**
   * Adds terms for substituting all placeholder variables (returned by
   * RegisterSequentialExpressions) with their respective `index`-th expression.
   * @pre 0 <= index < num_samples()
   */
  void AddPlaceholderVariableSubstitutionsForIndex(
      int index, symbolic::Substitution* substitution) const;

  /**
   * Returns the `index`-th expression for each element of `name`.
   * @pre 0 <= index < num_samples()
   **/
  VectorX<symbolic::Expression> GetSequentialExpressionsByName(
      const std::string& name, int index) const;

  /**
   * Returns the number of samples for the sequential expressions managed by
   * `this`.
   */
  int num_samples() const;

 private:
  int num_samples_{};
  std::unordered_map<std::string, std::pair<VectorX<symbolic::Variable>,
                                            MatrixX<symbolic::Expression>>>
      name_to_placeholders_and_sequential_expressions_{};
};
}  // namespace internal
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
