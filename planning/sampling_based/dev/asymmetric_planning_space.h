#pragma once

#include <cstdint>

#include "drake/planning/sampling_based/dev/default_state_types.h"
#include "drake/planning/sampling_based/dev/per_thread_random_source.h"
#include "drake/planning/sampling_based/dev/planning_space.h"

namespace drake {
namespace planning {
/// Base class for implementations of asymmetric planning spaces.
template <typename StateType>
class AsymmetricPlanningSpace : public PlanningSpace<StateType> {
 public:
  // The copy constructor is protected for use in implementing Clone().
  // Does not allow copy, move, or assignment.
  AsymmetricPlanningSpace(AsymmetricPlanningSpace<StateType>&&) = delete;
  AsymmetricPlanningSpace& operator=(
      const AsymmetricPlanningSpace<StateType>&) = delete;
  AsymmetricPlanningSpace& operator=(AsymmetricPlanningSpace<StateType>&&) =
      delete;

  ~AsymmetricPlanningSpace() override;

 protected:
  /// Copy constructor for use in Clone().
  AsymmetricPlanningSpace(const AsymmetricPlanningSpace<StateType>& other);

  /// Constructor.
  /// @param seed Seed for per-thread random source.
  /// @param parallelism Supported parallelism for operations using this
  /// planning space via thread numbers or OpenMP.
  AsymmetricPlanningSpace(uint64_t seed, Parallelism parallelism);
};

}  // namespace planning
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::drake::planning::AsymmetricPlanningSpace)
