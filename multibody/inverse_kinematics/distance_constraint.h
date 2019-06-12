#pragma once

#include <memory>

#include "drake/common/sorted_pair.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
class DistanceConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DistanceConstraint)

  DistanceConstraint(const multibody::MultibodyPlant<double>* const plant,
                     SortedPair<geometry::GeometryId> geometry_pair,
                     systems::Context<double>* plant_context,
                     double distance_lower, double distance_upper);

  ~DistanceConstraint() override {}

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "DistanceConstraint::DoEval does not work for symbolic variables.");
  }

  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<T>* y) const;

  const multibody::MultibodyPlant<double>& plant_;
  systems::Context<double>* const plant_context_;
  SortedPair<geometry::GeometryId> geometry_pair_;
};
}  // namespace multibody
}  // namespace drake
