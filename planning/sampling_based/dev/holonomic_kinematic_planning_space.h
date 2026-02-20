#pragma once

#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <fmt/format.h>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/sampling_based/dev/joint_limits.h"
#include "drake/planning/sampling_based/dev/symmetric_collision_checker_planning_space.h"

namespace drake {
namespace planning {
/// Implementation of a "holonomic" kinematic planning space. Broadly, this
/// covers all non-constrained planning uses to date, i.e. anywhere the
/// distance, interpolation, and validity provided by a single collision checker
/// is sufficient. Notably, it *does not* necessarily cover planning using a
/// mobile base.
/// Note: three member functions, DoNearestNeighborDistance(), DoSampleState()
/// and DoMotionCost(), may be further overridden by derived classes.
// TODO(calderpg) Add test coverage to ensure that thread_number is properly
// used by all methods (e.g. sampling, collision checking, etc).
class HolonomicKinematicPlanningSpace
    : public SymmetricCollisionCheckerPlanningSpace<Eigen::VectorXd> {
 public:
  // The copy constructor is protected for use in implementing Clone().
  // Does not allow copy, move, or assignment.
  HolonomicKinematicPlanningSpace(HolonomicKinematicPlanningSpace&&) = delete;
  HolonomicKinematicPlanningSpace& operator=(
      const HolonomicKinematicPlanningSpace&) = delete;
  HolonomicKinematicPlanningSpace& operator=(
      HolonomicKinematicPlanningSpace&&) = delete;

  /// Constructor.
  /// @param collision_checker Collision checker to use.
  /// @param joint_limits Joint limits to use for sampling. @pre size of
  /// position limits must match the configuration size of collision_checker.
  /// @param propagation_step_size Step size for propagation functions.
  /// @param seed Seed for per-thread random source.
  HolonomicKinematicPlanningSpace(
      std::unique_ptr<drake::planning::CollisionChecker> collision_checker,
      const JointLimits& joint_limits, double propagation_step_size,
      uint64_t seed);

  ~HolonomicKinematicPlanningSpace() override;

  // Class-specific getters and setters.
  double propagation_step_size() const { return propagation_step_size_; }

  /// Sets new propagation step size.
  /// @param propagation_step_size New propagation step size.
  /// @pre > 0.0 and finite.
  void SetPropagationStepSize(double propagation_step_size) {
    DRAKE_THROW_UNLESS(std::isfinite(propagation_step_size));
    DRAKE_THROW_UNLESS(propagation_step_size > 0.0);
    propagation_step_size_ = propagation_step_size;
  }

 protected:
  // Copy constructor for use in Clone().
  HolonomicKinematicPlanningSpace(const HolonomicKinematicPlanningSpace& other);

  // Implement SymmetricPlanningSpace API.

  std::unique_ptr<PlanningSpace<Eigen::VectorXd>> DoClone() const override;

  bool DoCheckStateValidity(const Eigen::VectorXd& state,
                            int thread_number) const final;

  bool DoCheckEdgeValidity(const Eigen::VectorXd& from,
                           const Eigen::VectorXd& to,
                           int thread_number) const final;

  Eigen::VectorXd DoSampleState(int thread_number) override;

  double DoStateDistance(const Eigen::VectorXd& from,
                         const Eigen::VectorXd& to) const final;

  Eigen::VectorXd DoInterpolate(const Eigen::VectorXd& from,
                                const Eigen::VectorXd& to,
                                double ratio) const final;

  std::vector<Eigen::VectorXd> DoPropagate(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to,
      std::map<std::string, double>* propagation_statistics,
      int thread_number) final;

  double DoMotionCost(const Eigen::VectorXd& from,
                      const Eigen::VectorXd& to) const override;

 private:
  double propagation_step_size_ = 0.0;
};

// Separate implementation for constrained case, see if this makes sense?
}  // namespace planning
}  // namespace drake
