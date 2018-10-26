#pragma once

#include <memory>

#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace multibody {
namespace internal {
class MinimalDistanceConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MinimalDistanceConstraint)

  /**
   * Constructs the minimal distance constraint, that the pairwise distance
   * between the objects in the world should be greater than some threshold.
   * We consider the distance between
   * 1. A dynamic object and a static (anchored) object.
   * 2. Two dynamic objects.
   * TODO(hongkai.dai): filter out between the pair of dynamic objects, that
   * one is the parent of the other (connected by a mechanical joint).
   * @param diagram The diagram that contains both the MultibodyPlant and the
   * SceneGraph. TODO(hongkai.dai): replace this diagram with the
   * MultiBodySceneGraph class when it is in drake.
   * @param num_positions The number of generalized positions in MultibodyPlant.
   * TODO(hongkai.dai): remove this argument when MultibodySceneGraph is ready.
   * @param minimal_distance The minimal distance allowed between the pair of
   * objects.
   * @param diagram_context The context that has been allocated for this
   * diagram. We will update the context when evaluating the constraint.
   */
  MinimalDistanceConstraint(
      const multibody::multibody_plant::MultibodyPlant<AutoDiffXd>& plant,
      double minimal_distance, systems::Context<AutoDiffXd>* plant_context);

  ~MinimalDistanceConstraint() override {}

  /** Getter for the minimal distance. */
  double minimal_distance() const { return minimal_distance_; }

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "MinimalDistanceConstraint::DoEval() does not work for symbolic "
        "variables.");
  }

  const multibody::multibody_plant::MultibodyPlant<AutoDiffXd>& plant_;
  const double minimal_distance_;
  systems::Context<AutoDiffXd>* const plant_context_;
};
}  // namespace internal
}  // namespace multibody
}  // namespace drake
