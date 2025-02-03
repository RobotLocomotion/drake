#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <fmt/format.h>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/planning/collision_checker.h"
#include "drake/solvers/constraint.h"
#include "planning/joint_limits.h"
#include "planning/symmetric_collision_checker_planning_space.h"

namespace anzu {
namespace planning {
/// A function type to build constraints using the provided standalone
/// CollisionCheckerContext.
///
/// Note: the only mutable operation that constraints may perform on the
/// provided context is setting positions and/or velocities.
///
/// Note: the returned Constraint objects may share (or alias) the provided
/// context; this is expected for any constraint that needs access to the plant.
/// However, the factory function itself must not share (or alias) the provided
/// context after it returns, as any later uses of the provided context may
/// cause thread safety or lifetime issues.
///
/// Note: we make no guarantees about when the factory function will be invoked
/// to create constraints, except that constraints will be created prior to use.
/// The factory function will be copied by the constrained planning space, and
/// any clones of the planning space will copy it as well, so the factory
/// function must be valid to call at any point the planning space or its clones
/// are still alive.
using ConstraintsFactoryFunction =
    std::function<std::vector<std::shared_ptr<drake::solvers::Constraint>>(
        const std::shared_ptr<drake::planning::CollisionCheckerContext>&)>;

/// "Dummy" constraints factory function that returns no constraints.
std::vector<std::shared_ptr<drake::solvers::Constraint>>
inline DummyConstraintsFactoryFunction(
    const std::shared_ptr<drake::planning::CollisionCheckerContext>&) {
  return {};
}

/// Implementation of a constrained kinematic planning space. Broadly, this
/// covers all constrained kinematic planning uses to date, i.e. anywhere the
/// distance, interpolation, and validity provided by a single collision checker
/// is sufficient.
/// Note: three member functions, DoNearestNeighborDistance(), DoSampleState()
/// and DoMotionCost(), may be further overridden by derived classes.
// Note: use of this class is covered by:
// - constrained_sampling_based_planning_test
// - parallel_rrt_planning_test
// TODO(calderpg) Improve/revise this class once we have a better approach for
// composing constraints with contexts.
// TODO(calderpg) Add test coverage to ensure that thread_number is properly
// used by all methods (e.g. sampling, collision/constraint checking, etc).
class InterimConstrainedKinematicPlanningSpace
    : public SymmetricCollisionCheckerPlanningSpace<Eigen::VectorXd> {
 public:
  // The copy constructor is protected for use in implementing Clone().
  // Does not allow copy, move, or assignment.
  InterimConstrainedKinematicPlanningSpace(
      InterimConstrainedKinematicPlanningSpace&&) = delete;
  InterimConstrainedKinematicPlanningSpace& operator=(
      const InterimConstrainedKinematicPlanningSpace&) = delete;
  InterimConstrainedKinematicPlanningSpace& operator=(
      InterimConstrainedKinematicPlanningSpace&&) = delete;

  /// Constructor.
  /// @param collision_checker Collision checker to use.
  /// @param joint_limits Joint limits to use for sampling. @pre size of
  /// position limits must match the configuration size of collision_checker.
  /// @param constraints_factory_fn Factory function to construct constraints.
  /// See documentation on ConstraintsFactoryFunction for more information.
  /// @param propagation_step_size Step size for propagation functions.
  /// @param minimum_propagation_progress Minimum progress required for
  /// propagation to continue.
  /// @param tolerance Tolerance for constraint checks.
  /// @param seed Seed for per-thread random source.
  InterimConstrainedKinematicPlanningSpace(
      std::unique_ptr<drake::planning::CollisionChecker> collision_checker,
      const JointLimits& joint_limits,
      const ConstraintsFactoryFunction& constraints_factory_fn,
      double propagation_step_size, double minimum_propagation_progress,
      double tolerance, uint64_t seed);

  ~InterimConstrainedKinematicPlanningSpace() override;

  // Convenience method for push-to-constraint operations.

  std::optional<Eigen::VectorXd> TryProjectStateToConstraints(
      const Eigen::VectorXd& state,
      std::optional<int> thread_number = std::nullopt) const;

  // Getters and setters.

  /// Sets new constraints factory function.
  /// See documentation on ConstraintsFactoryFunction for more information.
  /// @param constraints_factory_fn New constraints factory function.
  /// @pre non null.
  void SetConstraintsFactoryFunction(
      const ConstraintsFactoryFunction& constraints_factory_fn) {
    context_and_constraints_keeper_.SetConstraintsFactoryFunction(
        constraints_factory_fn);
  }

  double propagation_step_size() const { return propagation_step_size_; }

  /// Sets new propagation step size.
  /// @param propagation_step_size New propagation step size.
  /// @pre > 0.0 and propagation_step_size > minimum_propagation_progress and
  /// finite.
  void SetPropagationStepSize(double propagation_step_size) {
    DRAKE_THROW_UNLESS(std::isfinite(propagation_step_size));
    DRAKE_THROW_UNLESS(propagation_step_size > 0.0);
    DRAKE_THROW_UNLESS(propagation_step_size > minimum_propagation_progress());
    propagation_step_size_ = propagation_step_size;
  }

  double minimum_propagation_progress() const {
    return minimum_propagation_progress_;
  }

  /// Sets new minimum propagation progress.
  /// @param minimum_propagation_progress New minimum propagation progress.
  /// @pre > 0.0 and minimum_propagation_progress < propagation_step_size and
  /// finite.
  void SetMinimumPropagationProgress(double minimum_propagation_progress) {
    DRAKE_THROW_UNLESS(std::isfinite(minimum_propagation_progress));
    DRAKE_THROW_UNLESS(minimum_propagation_progress > 0.0);
    DRAKE_THROW_UNLESS(minimum_propagation_progress < propagation_step_size());
    minimum_propagation_progress_ = minimum_propagation_progress;
  }

  double tolerance() const { return tolerance_; }

  /// Sets new constraint tolerance.
  /// @param tolerance New constraint tolerance. @pre > 0 and finite.
  void SetTolerance(double tolerance) {
    DRAKE_THROW_UNLESS(std::isfinite(tolerance));
    DRAKE_THROW_UNLESS(tolerance >= 0.0);
    tolerance_ = tolerance;
  }

 protected:
  // Copy constructor for use in Clone().
  InterimConstrainedKinematicPlanningSpace(
      const InterimConstrainedKinematicPlanningSpace& other);

  // Implement SymmetricPlanningSpace API.

  std::unique_ptr<PlanningSpace<Eigen::VectorXd>> DoClone() const override;

  bool DoCheckStateValidity(
      const Eigen::VectorXd& state, int thread_number) const final;

  bool DoCheckEdgeValidity(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to,
      int thread_number) const final;

  bool DoCheckPathValidity(
      const std::vector<Eigen::VectorXd>& path,
      int thread_number) const final;

  Eigen::VectorXd DoSampleState(int thread_number) override;

  std::optional<Eigen::VectorXd> DoMaybeSampleValidState(
      int max_attempts, int thread_number) final;

  double DoStateDistance(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to) const final;

  Eigen::VectorXd DoInterpolate(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to, double ratio)
      const final;

  std::vector<Eigen::VectorXd> DoPropagate(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to,
      std::map<std::string, double>* propagation_statistics,
      int thread_number) final;

  double DoMotionCost(
      const Eigen::VectorXd& from, const Eigen::VectorXd& to) const override;

 private:
  // Stores a reference to a CollisionCheckerContext and the constraints built
  // around it.
  class ContextAndConstraints {
   public:
    // Provide only the move constructor, necessary to store this type in a
    // std::vector.
    ContextAndConstraints(ContextAndConstraints&&) = default;

    ContextAndConstraints(const ContextAndConstraints&) = delete;
    void operator=(const ContextAndConstraints&) = delete;
    void operator=(ContextAndConstraints&&) = delete;

    ContextAndConstraints(
        std::shared_ptr<drake::planning::CollisionCheckerContext> context,
        std::vector<std::shared_ptr<drake::solvers::Constraint>> constraints)
        : context_(std::move(context)), constraints_(std::move(constraints)) {
      DRAKE_THROW_UNLESS(context_ != nullptr);
    }

    const std::shared_ptr<drake::planning::CollisionCheckerContext>&
    context() const { return context_; }

    const std::vector<std::shared_ptr<drake::solvers::Constraint>>&
    constraints() const { return constraints_; }

   private:
    std::shared_ptr<drake::planning::CollisionCheckerContext> context_;
    std::vector<std::shared_ptr<drake::solvers::Constraint>> constraints_;
  };

  // Keeps per-thread ContextAndConstraints for use in planning space
  // operations.
  class ContextAndConstraintsKeeper {
   public:
    ContextAndConstraintsKeeper(
        const drake::planning::CollisionChecker* collision_checker,
        const ConstraintsFactoryFunction& constraints_factory_function)
        : collision_checker_(collision_checker),
          constraints_factory_function_(constraints_factory_function) {
      DRAKE_THROW_UNLESS(collision_checker_ != nullptr);
      DRAKE_THROW_UNLESS(constraints_factory_function_ != nullptr);
      AllocateContextAndConstraints();
    }

    // Providing a default constructor keeps the containing class copy
    // constructor simpler.
    ContextAndConstraintsKeeper() = default;

    const ContextAndConstraints& context_and_constraints(
        int thread_number) const {
      return context_and_constraints_.at(thread_number);
    }

    void SetConstraintsFactoryFunction(
        const ConstraintsFactoryFunction& constraints_factory_function) {
      DRAKE_THROW_UNLESS(constraints_factory_function != nullptr);
      constraints_factory_function_ = constraints_factory_function;
      AllocateContextAndConstraints();
    }

    const drake::planning::CollisionChecker& collision_checker() const {
      DRAKE_THROW_UNLESS(collision_checker_ != nullptr);
      return *collision_checker_;
    }

    const ConstraintsFactoryFunction& constraints_factory_function() const {
      return constraints_factory_function_;
    }

   private:
    void AllocateContextAndConstraints() {
      context_and_constraints_.clear();

      for (int context_num = 0;
           context_num < collision_checker().num_allocated_contexts();
           ++context_num) {
        // Where possible, we want to avoid creating a new standalone context
        // just for operations within a single function. Instead, in these cases
        // we can use the appropriate implicit context from the collision
        // checker (i.e. the same implicit context that would be used if we
        // performed implicit context queries on the collision checker), and
        // construct a shared_ptr to the context using the shared_ptr aliasing
        // constructor.
        // Note: to avoid unsafe use, the context shared_ptrs should never be
        // used outside a given planning space to ensure they do not outlive the
        // CollisionChecker from which they were created.
        // TODO(calderpg) Revise this approach when we have stronger protections
        // against unsafe mutation of pool and standalone
        // CollisionCheckerContexts.
        auto& mutable_context =
            const_cast<drake::planning::CollisionCheckerContext&>(
                collision_checker().model_context(context_num));
        std::shared_ptr<drake::planning::CollisionCheckerContext>
            shared_context(std::shared_ptr<void>{}, &mutable_context);

        auto constraints = constraints_factory_function_(shared_context);

        context_and_constraints_.emplace_back(
            std::move(shared_context), std::move(constraints));
      }
    }

    const drake::planning::CollisionChecker* collision_checker_ = nullptr;
    ConstraintsFactoryFunction constraints_factory_function_;
    std::vector<ContextAndConstraints> context_and_constraints_;
  };

  ContextAndConstraintsKeeper context_and_constraints_keeper_;

  double propagation_step_size_ = 0.0;
  double minimum_propagation_progress_ = 0.0;
  double tolerance_ = 0.0;
};

}  // namespace planning
}  // namespace anzu
