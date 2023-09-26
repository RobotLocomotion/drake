#pragma once

#include <memory>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/minimum_value_constraint.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
/** Constraint min(d) <= ub. Namely at least one of the signed distance between
 all candidate pairs of geometries is below ub. When ub=0, it means the robot
 is in collision.
 */
class InCollisionConstraint final : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InCollisionConstraint);

  /**
   @param minimum_distance_upper The upper bound on the minimum distance.
   @param normalizer A positive scalar. This normalizer should be roughly in the
   same magnitude as the range of distances between all pairs of geometries. It
   doesn't need to be precise.
   */
  InCollisionConstraint(const multibody::MultibodyPlant<double>* const plant,
                        double minimum_distance_upper, double normalizer,
                        systems::Context<double>* plant_context);

  /** Overloaded constructor with AutoDiff scalar. */
  InCollisionConstraint(
      const multibody::MultibodyPlant<AutoDiffXd>* const plant,
      double minimum_distance_upper, double normalizer,
      systems::Context<AutoDiffXd>* plant_context);

  ~InCollisionConstraint() override {}

  [[nodiscard]] double minimum_distance_upper() const {
    return minimum_distance_upper_;
  }

  [[nodiscard]] double normalizer() const { return normalizer_; }

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "InCollisionConstraint::DoEval() does not work for symbolic "
        "variables.");
  }

  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<T>* y) const;

  template <typename T>
  void Initialize(const MultibodyPlant<T>& plant,
                  systems::Context<T>* plant_context,
                  double minimum_distance_upper, double nomalizer);

  const multibody::MultibodyPlant<double>* const plant_double_;
  systems::Context<double>* const plant_context_double_;
  std::unique_ptr<solvers::MinimumValueUpperBoundConstraint>
      minimum_value_constraint_;

  const multibody::MultibodyPlant<AutoDiffXd>* const plant_autodiff_;
  systems::Context<AutoDiffXd>* const plant_context_autodiff_;

  double minimum_distance_upper_;
  double normalizer_;
};
}  // namespace multibody
}  // namespace drake
