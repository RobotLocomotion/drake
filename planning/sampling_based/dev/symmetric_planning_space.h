#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "planning/default_state_types.h"
#include "planning/per_thread_random_source.h"
#include "planning/planning_space.h"

namespace anzu {
namespace planning {
/// Base class for implementations of symmetric planning spaces, in which each
/// pair of *Forward* and *Backwards* methods is provided by a single
/// implementation.
// TODO(calderpg) Add test coverage to ensure that thread_number is properly
// wired through to implementations.
template<typename StateType>
class SymmetricPlanningSpace : public PlanningSpace<StateType> {
 public:
  // The copy constructor is protected for use in implementing Clone().
  // Does not allow copy, move, or assignment.
  SymmetricPlanningSpace(SymmetricPlanningSpace<StateType>&&) = delete;
  SymmetricPlanningSpace& operator=(
      const SymmetricPlanningSpace<StateType>&) = delete;
  SymmetricPlanningSpace& operator=(
      SymmetricPlanningSpace<StateType>&&) = delete;

  ~SymmetricPlanningSpace() override;

  /// Computes the distance for use in nearest neighbors checks in (Bi)RRT
  /// planners. This method is provided to allow for faster approximate distance
  /// functions used in nearest neighbor checks.
  /// By default, the state distance function is used.
  /// @param from Starting state, from the start tree of a (Bi)RRT planner.
  /// @param to Ending state, generally a sampled state.
  /// @return Distance (potentially approximate) from start state to end state.
  double NearestNeighborDistance(
      const StateType& from, const StateType& to) const {
    return DoNearestNeighborDistance(from, to);
  }

  /// Computes the distance between two states. In the PRM planner, this is used
  /// for both nearest neighbor and edge weighting. In the RRT planner, this is
  /// used for goal distance checks. In the BiRRT planner, this is used for
  /// connection distance checks.
  /// @param from Starting state.
  /// @param to Ending state.
  /// @return Distance from start state to end state.
  double StateDistance(const StateType& from, const StateType& to) const {
    return DoStateDistance(from, to);
  }

  /// Interpolates between the provided states. In the path processor, this is
  /// used to resample intermediate states on a path.
  /// @param from Starting state.
  /// @param to Ending state.
  /// @param ratio Interpolation ratio. @pre 0 <= ratio <= 1.
  /// @return Interpolated state.
  StateType Interpolate(
      const StateType& from, const StateType& to, double ratio) const {
    DRAKE_THROW_UNLESS(ratio >= 0.0);
    DRAKE_THROW_UNLESS(ratio <= 1.0);
    return DoInterpolate(from, to, ratio);
  }

  /// Performs propagation from the provided start state towards the provided
  /// end state, as used in (Bi)RRT planners. All propagated states must be
  /// valid.
  /// @param from Starting state.
  /// @param to Ending state.
  /// @param propagation_statistics Map<string, double> that is provided by
  /// (Bi)RRT planners for planning spaces to use in collecting statistics about
  /// propagation for use in debugging. @pre propagation_statistics != nullptr.
  /// @param thread_number Optional thread number.
  /// @return Sequence of propagated states.
  std::vector<StateType> Propagate(
      const StateType& from, const StateType& to,
      std::map<std::string, double>* propagation_statistics,
      std::optional<int> thread_number = std::nullopt) {
    DRAKE_THROW_UNLESS(propagation_statistics != nullptr);
    return DoPropagate(
        from, to, propagation_statistics,
        this->ResolveThreadNumber(thread_number));
  }

  /// Computes the cost for the motion between the provided states, as would be
  /// used in a cost-sensitive (Bi)RRT (e.g. T-RRT/BiT-RRT or RRT*).
  /// By default, the state distance function is used.
  /// @param from Starting state.
  /// @param to Ending state.
  /// @return Motion cost from start state to end state.
  double MotionCost(const StateType& from, const StateType& to) const {
    return DoMotionCost(from, to);
  }

 protected:
  /// Interface to be implemented by derived classes. See public methods above
  /// for more information.

  virtual double DoNearestNeighborDistance(
      const StateType& from, const StateType& to) const {
    return StateDistance(from, to);
  }

  virtual double DoStateDistance(
      const StateType& from, const StateType& to) const = 0;

  virtual StateType DoInterpolate(
      const StateType& from, const StateType& to, double ratio) const = 0;

  virtual std::vector<StateType> DoPropagate(
      const StateType& from, const StateType& to,
      std::map<std::string, double>* propagation_statistics,
      int thread_number) = 0;

  virtual double DoMotionCost(
      const StateType& from, const StateType& to) const {
    return StateDistance(from, to);
  }

  /// Implements PlanningSpace API, where each pair of *Forwards* and
  /// *Backwards* methods is provided by a single implementation.

  double DoNearestNeighborDistanceForwards(
      const StateType& from, const StateType& to) const final {
    return NearestNeighborDistance(from, to);
  }

  double DoNearestNeighborDistanceBackwards(
      const StateType& from, const StateType& to) const final {
    return NearestNeighborDistance(from, to);
  }

  double DoStateDistanceForwards(
      const StateType& from, const StateType& to) const final {
    return StateDistance(from, to);
  }

  double DoStateDistanceBackwards(
      const StateType& from, const StateType& to) const final {
    return StateDistance(from, to);
  }

  StateType DoInterpolateForwards(
      const StateType& from, const StateType& to, double ratio) const final {
    return Interpolate(from, to, ratio);
  }

  StateType DoInterpolateBackwards(
      const StateType& from, const StateType& to, double ratio) const final {
    return Interpolate(from, to, ratio);
  }

  std::vector<StateType> DoPropagateForwards(
      const StateType& from, const StateType& to,
      std::map<std::string, double>* propagation_statistics,
      int thread_number) final {
    return Propagate(from, to, propagation_statistics, thread_number);
  }

  std::vector<StateType> DoPropagateBackwards(
      const StateType& from, const StateType& to,
      std::map<std::string, double>* propagation_statistics,
      int thread_number) final {
    return Propagate(from, to, propagation_statistics, thread_number);
  }

  double DoMotionCostForwards(
      const StateType& from, const StateType& to) const final {
    return MotionCost(from, to);
  }

  double DoMotionCostBackwards(
      const StateType& from, const StateType& to) const final {
    return MotionCost(from, to);
  }

  // Copy constructor for use in Clone().
  SymmetricPlanningSpace(
      const SymmetricPlanningSpace<StateType>& other);

  /// Constructor.
  /// @param seed Seed for per-thread random source.
  /// @param parallelism Supported parallelism for operations using this
  /// planning space via thread numbers or OpenMP.
  SymmetricPlanningSpace(uint64_t seed, Parallelism parallelism);
};
}  // namespace planning
}  // namespace anzu

ANZU_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::anzu::planning::SymmetricPlanningSpace)
