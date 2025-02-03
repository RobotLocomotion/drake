#pragma once

#include <cstdint>

#include "planning/default_state_types.h"
#include "planning/per_thread_random_source.h"
#include "planning/planning_space.h"

namespace anzu {
namespace planning {
/// Base class for implementations of asymmetric planning spaces.
template<typename StateType>
class AsymmetricPlanningSpace : public PlanningSpace<StateType> {
 public:
  // The copy constructor is protected for use in implementing Clone().
  // Does not allow copy, move, or assignment.
  AsymmetricPlanningSpace(AsymmetricPlanningSpace<StateType>&&) = delete;
  AsymmetricPlanningSpace& operator=(
      const AsymmetricPlanningSpace<StateType>&) = delete;
  AsymmetricPlanningSpace& operator=(
      AsymmetricPlanningSpace<StateType>&&) = delete;

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
}  // namespace anzu

ANZU_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::anzu::planning::AsymmetricPlanningSpace)
