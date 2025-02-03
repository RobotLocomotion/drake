#pragma once

#include <cstdint>
#include <tuple>
#include <vector>

#include "planning/default_state_types.h"
#include "planning/path_planning_errors.h"

namespace anzu {
namespace planning {

/// Wrapper type for valid start & goal configs with errors. Supports structured
/// bindings with const reference accessors.
template <typename StateType>
class ValidStartsAndGoals {
 public:
  /// Provides all copy/move/assign operations.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ValidStartsAndGoals);

  ValidStartsAndGoals(
      std::vector<StateType> valid_starts, std::vector<StateType> valid_goals);

  ValidStartsAndGoals();

  const std::vector<StateType>& valid_starts() const { return valid_starts_; }

  const std::vector<StateType>& valid_goals() const { return valid_goals_; }

  const PathPlanningErrors& errors() const { return errors_; }

  // Accessor required for structured bindings.
  template <size_t Index>
  const std::tuple_element_t<Index, ValidStartsAndGoals<StateType>>&
  get() const {
    if constexpr (Index == 0) { return valid_starts_; }
    if constexpr (Index == 1) { return valid_goals_; }
    if constexpr (Index == 2) { return errors_; }
  }

 private:
  void SetErrors();

  std::vector<StateType> valid_starts_;
  std::vector<StateType> valid_goals_;
  PathPlanningErrors errors_;
};
}  // namespace planning
}  // namespace anzu

// Interface bits required for structured binding of ValidStartsAndGoals.
namespace std {
template <typename StateType>
struct tuple_size<anzu::planning::ValidStartsAndGoals<StateType>> {
  static constexpr size_t value = 3;
};

template <size_t Index, typename StateType>
struct tuple_element<Index, anzu::planning::ValidStartsAndGoals<StateType>>
    : tuple_element<
        Index,
        tuple<std::vector<StateType>,
              std::vector<StateType>,
              anzu::planning::PathPlanningErrors>> {};
}  // namespace std

ANZU_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::anzu::planning::ValidStartsAndGoals)
