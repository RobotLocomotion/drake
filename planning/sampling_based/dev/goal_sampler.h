#pragma once

#include <functional>
#include <random>

#include "drake/common/drake_throw.h"
#include "planning/default_state_types.h"

namespace anzu {
namespace planning {

/// Interface used by RRT and ParallelRRT planners to sample goal states. Since
/// ParallelRRT performs goal sampling in parallel, implementations of
/// GoalSampler must be thread safe.
template <typename StateType>
class GoalSampler {
 public:
  // The copy constructor is protected to allow derived classes to be copyable.
  // Does not allow copy, move, or assignment.
  GoalSampler(GoalSampler&&) = delete;
  GoalSampler<StateType>& operator=(const GoalSampler&) = delete;
  GoalSampler<StateType>& operator=(GoalSampler&&) = delete;

  virtual ~GoalSampler();

  /// Samples a goal state. To achieve thread safety, the calling planner
  /// provides the appropriate generator, and a thread number for use with other
  /// resources, such as performing collision checks on a PlanningSpace.
  /// @pre generator != nullptr.
  StateType Sample(std::mt19937_64* generator, int thread_number) const {
    DRAKE_THROW_UNLESS(generator != nullptr);
    return DoSample(generator, thread_number);
  }

 protected:
  GoalSampler();

  GoalSampler(const GoalSampler& other);

  /// Derived goal checkers must implement goal sampling. Base class ensures
  /// that the provided `generator` is non-null.
  virtual StateType DoSample(
      std::mt19937_64* generator, int thread_number) const = 0;
};

}  // namespace planning
}  // namespace anzu

ANZU_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::anzu::planning::GoalSampler)
