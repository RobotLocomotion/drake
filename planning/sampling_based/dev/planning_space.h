#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "planning/default_state_types.h"
#include "planning/per_thread_random_source.h"
#include "planning/valid_starts.h"
#include "planning/valid_starts_and_goals.h"

namespace anzu {
namespace planning {
// Forward declarations.
template<typename StateType>
class AsymmetricPlanningSpace;
template<typename StateType>
class SymmetricPlanningSpace;
template<typename StateType>
class CollisionCheckerPlanningSpace;

/// Provides a single interface to state distance, state/edge validity, state
/// propagation, etc as used by motion planners. All sampling-based motion
/// planner methods and the path processor take a PlanningSpace.
///
/// Generally, support for OpenMP and OpenMP-like parallelism with an equivalent
/// to OpenMP's notion of thread number is provided by a single PlanningSpace.
/// Methods that are likely to require per-thread resources (e.g. sampling and
/// collision checking) take an optional thread_number parameter so that
/// planners using the PlanningSpace API can identify which thread is performing
/// the method call and implementations can identify which per-thread resources
/// to use. See HolonomicKinematicPlanningSpace for examples of the use of
/// thread_number with collision checking. If nullopt is provided, operations
/// (e.g. DrawRaw/DrawUniformUnitReal in PerThreadRandomSource, or collision
/// checks performed by CollisionChecker) generally use the OpenMP thread number
/// of the current thread.
///
/// Support for other methods of parallelism is provided by cloning the planning
/// space and using one clone per thread. See the parallel (Bi)RRT planners for
/// examples of this usage.
///
/// Note: users cannot inherit directly from PlanningSpace, and must instead
/// inherit from AsymmetricPlanningSpace, SymmetricPlanningSpace,
/// AsymmetricCollisionCheckerPlanningSpace, or
/// SymmetricCollisionCheckerPlanningSpace depending on whether or not their
/// space is symmetric (i.e. each pair of *Forward* and *Backwards* methods is
/// provided by a single implementation) and whether or not they want ownership
/// of a CollisionChecker and JointLimits to be handled for them.
// TODO(calderpg) Consider if motion cost should be wrapped into Propagate*()
// calls and/or edge evaluation? Some "expensive" planning spaces would benefit
// from this approach.
// TODO(calderpg) Add test coverage to ensure that thread_number is properly
// wired through to implementations.
template<typename StateType>
class PlanningSpace {
 public:
  // The copy constructor is protected for use in implementing Clone().
  // Does not allow copy, move, or assignment.
  PlanningSpace(PlanningSpace<StateType>&&) = delete;
  PlanningSpace& operator=(const PlanningSpace<StateType>&) = delete;
  PlanningSpace& operator=(PlanningSpace<StateType>&&) = delete;

  virtual ~PlanningSpace();

  /// Clones the current planning space. Clones *must not* share mutable state,
  /// as cloning is used to provide thread safety in parallel (Bi)RRT planners.
  std::unique_ptr<PlanningSpace<StateType>> Clone() const { return DoClone(); }

  /// Computes the "forwards" distance for use in nearest neighbors checks in
  /// (Bi)RRT planners. All nearest neighbor checks in RRT planners and nearest
  /// neighbor checks between the start tree and sampled states in BiRRT
  /// planners use this method. This method is provided to allow for faster
  /// approximate distance functions used in nearest neighbor checks.
  /// By default, the forwards state distance function is used.
  /// @param from Starting state, from the start tree of a (Bi)RRT planner.
  /// @param to Ending state, generally a sampled state.
  /// @return Distance (potentially approximate) from start state to end state.
  double NearestNeighborDistanceForwards(
      const StateType& from, const StateType& to) const {
    return DoNearestNeighborDistanceForwards(from, to);
  }

  /// Computes the "backwards" distance for use in nearest neighbors checks in
  /// BiRRT planners. Nearest neighbor checks between the goal tree and sampled
  /// states in BiRRT planners use this method. This method is provided to allow
  /// for faster approximate distance functions used in nearest neighbor checks.
  /// By default, the backwards state distance function is used.
  /// @param from Starting state, from the goal tree of a (Bi)RRT planner.
  /// @param to Ending state, generally a sampled state.
  /// @return Distance (potentially approximate) from start state to end state.
  double NearestNeighborDistanceBackwards(
      const StateType& from, const StateType& to) const {
    return DoNearestNeighborDistanceBackwards(from, to);
  }

  /// Computes the "forwards" distance between two states. In the PRM planner,
  /// this is used for both nearest neighbor and edge weighting. In the RRT
  /// planner, this is used for goal distance checks. In the BiRRT planner, this
  /// is used for connection distance checks from start tree to goal tree.
  /// @param from Starting state.
  /// @param to Ending state.
  /// @return Distance from start state to end state.
  double StateDistanceForwards(
      const StateType& from, const StateType& to) const {
    return DoStateDistanceForwards(from, to);
  }

  /// Computes the "backwards" distance between two states. In the BiRRT
  /// planner, this is used for connection distance checks from goal tree to
  /// start tree.
  /// @param from Starting state.
  /// @param to Ending state.
  /// @return Distance from start state to end state.
  double StateDistanceBackwards(
      const StateType& from, const StateType& to) const {
    return DoStateDistanceBackwards(from, to);
  }

  /// Calculates the length of the provided path, as the sum of the forward
  /// distance of each edge.
  /// @param path Provided path.
  /// @return Length of path.
  double CalcPathLength(const std::vector<StateType>& path) const;

  /// Interpolates "forwards" between the provided states. In the path
  /// processor, this is used to resample intermediate states on a path.
  /// @param from Starting state.
  /// @param to Ending state.
  /// @param ratio Interpolation ratio. @pre 0 <= ratio <= 1.
  /// @return Interpolated state.
  StateType InterpolateForwards(
      const StateType& from, const StateType& to, double ratio) const {
    DRAKE_THROW_UNLESS(ratio >= 0.0);
    DRAKE_THROW_UNLESS(ratio <= 1.0);
    return DoInterpolateForwards(from, to, ratio);
  }

  /// Interpolates "backwards" between the provided states.
  /// @param from Starting state.
  /// @param to Ending state.
  /// @param ratio Interpolation ratio. @pre 0 <= ratio <= 1.
  /// @return Interpolated state.
  StateType InterpolateBackwards(
      const StateType& from, const StateType& to, double ratio) const {
    DRAKE_THROW_UNLESS(ratio >= 0.0);
    DRAKE_THROW_UNLESS(ratio <= 1.0);
    return DoInterpolateBackwards(from, to, ratio);
  }

  /// Performs "forwards" propagation from the provided start state towards the
  /// provided end state, as used in RRT planner and forward propagations of
  /// the BiRRT planner. All propagated states must be valid.
  /// @param from Starting state.
  /// @param to Ending state.
  /// @param propagation_statistics Map<string, double> that is provided by
  /// (Bi)RRT planners for planning spaces to use in collecting statistics about
  /// propagation for use in debugging. @pre propagation_statistics != nullptr.
  /// @param thread_number Optional thread number.
  /// @return Sequence of propagated states.
  std::vector<StateType> PropagateForwards(
      const StateType& from, const StateType& to,
      std::map<std::string, double>* propagation_statistics,
      std::optional<int> thread_number = std::nullopt) {
    DRAKE_THROW_UNLESS(propagation_statistics != nullptr);
    return DoPropagateForwards(
        from, to, propagation_statistics, ResolveThreadNumber(thread_number));
  }

  /// Performs "backwards" propagation from the provided start state towards the
  /// provided end state, as used in backwards propagations of the BiRRT
  /// planner. All propagated states must be valid.
  /// @param from Starting state.
  /// @param to Ending state.
  /// @param propagation_statistics Map<string, double> that is provided by
  /// (Bi)RRT planners for planning spaces to use in collecting statistics about
  /// propagation for use in debugging. @pre propagation_statistics != nullptr.
  /// @param thread_number Optional thread number.
  /// @return Sequence of propagated states.
  std::vector<StateType> PropagateBackwards(
      const StateType& from, const StateType& to,
      std::map<std::string, double>* propagation_statistics,
      std::optional<int> thread_number = std::nullopt) {
    DRAKE_THROW_UNLESS(propagation_statistics != nullptr);
    return DoPropagateBackwards(
        from, to, propagation_statistics, ResolveThreadNumber(thread_number));
  }

  /// Computes the "forwards" cost for the motion between the provided states,
  /// as would be used in a cost-sensitive RRT or forwards propagations in a
  /// cost-sensitive BiRRT (e.g. T-RRT/BiT-RRT or RRT*).
  /// By default, the forwards state distance function is used.
  /// @param from Starting state.
  /// @param to Ending state.
  /// @return Motion cost from start state to end state.
  double MotionCostForwards(
      const StateType& from, const StateType& to) const {
    return DoMotionCostForwards(from, to);
  }

  /// Computes the "backwards" cost for the motion between the provided states,
  /// as would be used in backwards propagations in a cost-sensitive BiRRT (e.g.
  /// BiT-RRT or RRT*).
  /// By default, the backwards state distance function is used.
  /// @param from Starting state.
  /// @param to Ending state.
  /// @return Motion cost from start state to end state.
  double MotionCostBackwards(
      const StateType& from, const StateType& to) const {
    return DoMotionCostBackwards(from, to);
  }

  /// Checks the validity (e.g. if the state is collision free and within
  /// limits) of the provided state.
  /// @param state State to check.
  /// @param thread_number Optional thread number.
  /// @return true if the provided state is valid.
  bool CheckStateValidity(
      const StateType& state,
      std::optional<int> thread_number = std::nullopt) const {
    return DoCheckStateValidity(state, ResolveThreadNumber(thread_number));
  }

  /// Extracts the valid states from the provided start states. Used in some
  /// forms of the RRT planner.
  /// @param starts Potential start states.
  /// @param thread_number Optional thread number.
  /// @return Valid start states.
  ValidStarts<StateType> ExtractValidStarts(
      const std::vector<StateType>& starts,
      std::optional<int> thread_number = std::nullopt) const;

  /// Extracts the valid states from the provided start and goal states. Used in
  /// PRM and (Bi)RRT planners.
  /// @param starts Potential start states.
  /// @param goals Potential goal states.
  /// @param thread_number Optional thread number.
  /// @return Valid start and goal states.
  ValidStartsAndGoals<StateType> ExtractValidStartsAndGoals(
      const std::vector<StateType>& starts,
      const std::vector<StateType>& goals,
      std::optional<int> thread_number = std::nullopt) const;

  /// Checks the validity of the provided directed edge.
  /// @param from Starting state of edge.
  /// @param to Ending state of edge.
  /// @param thread_number Optional thread number.
  /// @return true if edge is valid.
  bool CheckEdgeValidity(
      const StateType& from, const StateType& to,
      std::optional<int> thread_number = std::nullopt) const {
    return DoCheckEdgeValidity(from, to, ResolveThreadNumber(thread_number));
  }

  /// Checks the validity of the provided path. By default, this checks the
  /// validity of each edge in the path, but may be overriden to provide a more
  /// efficient implementation (e.g. context/constraint reuse).
  /// @param path Provided path. @pre path is non-empty.
  /// @param thread_number Optional thread number.
  /// @return true if entire path is valid.
  bool CheckPathValidity(
      const std::vector<StateType>& path,
      std::optional<int> thread_number = std::nullopt) const {
    return DoCheckPathValidity(path, ResolveThreadNumber(thread_number));
  }

  /// Samples a state (not necessarily a valid state).
  /// @param thread_number Optional thread number.
  /// @return Sampled state.
  StateType SampleState(std::optional<int> thread_number = std::nullopt) {
    return DoSampleState(ResolveThreadNumber(thread_number));
  }

  /// Attempts to sample a valid state, optionally taking up to max_attempts
  /// tries.
  /// @param max_attempts Maximum number of tries to sample a valid state.
  /// @param thread_number Optional thread number.
  /// @pre max_attempts > 0.
  /// @return Valid sample state or nullopt if no valid state could be sampled.
  std::optional<StateType> MaybeSampleValidState(
      int max_attempts, std::optional<int> thread_number = std::nullopt) {
    DRAKE_THROW_UNLESS(max_attempts > 0);
    return DoMaybeSampleValidState(
        max_attempts, ResolveThreadNumber(thread_number));
  }

  /// Same as MaybeSampleValidState, but throws if no valid state could be
  /// sampled.
  /// @param max_attempts Maximum number of tries to sample a valid state.
  /// @param thread_number Optional thread number.
  /// @pre max_attempts > 0.
  /// @return Valid sample state.
  StateType SampleValidState(
      int max_attempts, std::optional<int> thread_number = std::nullopt);

  /// Retrieves the per-thread random source.
  PerThreadRandomSource& random_source() { return random_source_; }

  /// Supported parallelism of this planning space.
  Parallelism parallelism() const { return parallelism_; }

  /// Is the planning space symmetric (i.e. are *Forwards* and *Backwards*
  /// methods the same)?
  bool is_symmetric() const { return is_symmetric_; }

  /// Resolves the provided optional `thread_number`. If a value is provided,
  /// the value of `thread_number` is returned. If nullopt, the current OpenMP
  /// thread number is returned.
  static int ResolveThreadNumber(std::optional<int> thread_number);

 protected:
  /// Copy constructor for use in Clone().
  PlanningSpace(const PlanningSpace<StateType>& other);

  /// Interface to be implemented by derived classes. See public methods above
  /// for more information.

  virtual std::unique_ptr<PlanningSpace<StateType>> DoClone() const = 0;

  virtual double DoNearestNeighborDistanceForwards(
      const StateType& from, const StateType& to) const {
    return StateDistanceForwards(from, to);
  }

  virtual double DoNearestNeighborDistanceBackwards(
      const StateType& from, const StateType& to) const {
    return StateDistanceBackwards(from, to);
  }

  virtual double DoStateDistanceForwards(
      const StateType& from, const StateType& to) const = 0;

  virtual double DoStateDistanceBackwards(
      const StateType& from, const StateType& to) const = 0;

  virtual StateType DoInterpolateForwards(
      const StateType& from, const StateType& to, double ratio) const = 0;

  virtual StateType DoInterpolateBackwards(
      const StateType& from, const StateType& to, double ratio) const = 0;

  virtual std::vector<StateType> DoPropagateForwards(
      const StateType& from, const StateType& to,
      std::map<std::string, double>* propagation_statistics,
      int thread_number) = 0;

  virtual std::vector<StateType> DoPropagateBackwards(
      const StateType& from, const StateType& to,
      std::map<std::string, double>* propagation_statistics,
      int thread_number) = 0;

  virtual double DoMotionCostForwards(
      const StateType& from, const StateType& to) const {
    return StateDistanceForwards(from, to);
  }

  virtual double DoMotionCostBackwards(
      const StateType& from, const StateType& to) const {
    return StateDistanceForwards(from, to);
  }

  virtual bool DoCheckStateValidity(
      const StateType& state, int thread_number) const = 0;

  virtual bool DoCheckEdgeValidity(
      const StateType& from, const StateType& to, int thread_number) const = 0;

  virtual bool DoCheckPathValidity(
      const std::vector<StateType>& path, int thread_number) const;

  virtual StateType DoSampleState(int thread_number) = 0;

  virtual std::optional<StateType> DoMaybeSampleValidState(
      int max_attempts, int thread_number);

 private:
  friend class AsymmetricPlanningSpace<StateType>;
  friend class SymmetricPlanningSpace<StateType>;
  friend class CollisionCheckerPlanningSpace<StateType>;

  // Constructor. This is private so that only AsymmetricPlanningSpace and
  // SymmetricPlanningSpace, and CollisionCheckerPlanningSpace (declared as
  // friends above) can directly construct a PlanningSpace.
  // @param seed Seed for per-thread random source.
  // @param parallelism Supported parallelism for operations using this planning
  // space via thread numbers or OpenMP.
  // @param is_symmetric Is the planning space symmetric?
  PlanningSpace(uint64_t seed, Parallelism parallelism, bool is_symmetric);

  PerThreadRandomSource random_source_;
  const Parallelism parallelism_;
  const bool is_symmetric_;
};

}  // namespace planning
}  // namespace anzu

ANZU_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_PLANNING_STATE_TYPES(
    class ::anzu::planning::PlanningSpace)
