#pragma once

#include <string>
#include <unordered_map>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
class TrajectoryOptimization : public solvers::MathematicalProgram {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectoryOptimization)

  TrajectoryOptimization() = default;

  virtual ~TrajectoryOptimization() = default;

  VectorX<symbolic::Variable> NewSequentialVariables(int rows,
                                                     const std::string& name);

  virtual void AddRunningCost(const symbolic::Expression& cost) = 0;

  virtual void AddFinalCost(const symbolic::Expression& cost) = 0;

  void AddPlaceholderVariableSubstitutionsForIndex(
      int index, symbolic::Substitution* substitution) const;

 protected:
  virtual int SamplesPerSequentialVariable() const = 0;

  symbolic::Variable GetSequentialVariableAtIndex(const symbolic::Variable&,
                                                  int index);

  VectorX<symbolic::Variable> GetSequentialVariablesAtIndex(
      const VectorX<symbolic::Variable>&, int index);

 private:
  std::unordered_map<symbolic::Variable, RowVectorX<symbolic::Variable>>
      placeholder_to_sequential_variables_{};
};

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
