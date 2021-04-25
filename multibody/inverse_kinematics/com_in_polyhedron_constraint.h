#pragma once

#include <optional>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
/**
 * Constrains the center of mass to lie within a polyhedron
 * lb <= A * p_EC <= ub
 * where p_EC is the position of the center-of-mass (C) expressed in a frame E.
 *
 * For example, if you set A as identity matrix, then this constraint enforces a
 * box-region on the CoM position p_EC. If you set the expressed frame E as the
 * robot foot frame, and choose A to describe the foot support polygon, this
 * constraint could enforce the projection of CoM to be within the foot support
 * polygon, which is commonly used to ensure static equilibrium.
 */
class ComInPolyhedronConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ComInPolyhedronConstraint)

  /**
   * Constructs a %ComInPolyhedronConstraint object.
   * @param plant The MultibodyPlant on which the constraint is imposed. `plant`
   * must be alive during the lifetime of this constraint.
   * @param model_instances The CoM of these model instances are computed. If
   * model_instances = std::nullopt, then we compute the CoM of all model
   * instances (except the world). Currently if model_instances is not
   * std::nullopt, we will throw an error. After github issue #14916 is
   * resolved, we will accept model_instances not equal to std::nullopt.
   * @param expressed_frame The frame in which the CoM is expressed.
   * @param A The CoM position p_EC satisfies lb <= A * p_EC <= ub
   * @param lb The CoM position p_EC satisfies lb <= A * p_EC <= ub
   * @param ub The CoM position p_EC satisfies lb <= A * p_EC <= ub
   * @param plant_context The Context that has been allocated for this
   * `plant`. We will update the context when evaluating the constraint.
   * `plant_context` must be alive during the lifetime of this constraint.
   */
  ComInPolyhedronConstraint(
      const MultibodyPlant<double>* plant,
      std::optional<std::vector<ModelInstanceIndex>> model_instances,
      const Frame<double>& expressed_frame,
      const Eigen::Ref<const Eigen::MatrixX3d>& A,
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub,
      systems::Context<double>* plant_context);

  /**
   * Overloaded constructor. Same as the constructor with the double version
   * (using MultibodyPlant<double> and Context<double>. Except the gradient of
   * the constraint is computed from autodiff.
   * @pre if model_instances is not std::nullopt, then all indices in
   * `model_instances` refer to valid model instances in `plant`.
   * @exclude_from_pydrake_mkdoc{Suppressed due to ambiguity in mkdoc.
   * Documentation string is manually recreated in Python.}
   */
  ComInPolyhedronConstraint(
      const MultibodyPlant<AutoDiffXd>* plant,
      std::optional<std::vector<ModelInstanceIndex>> model_instances,
      const Frame<AutoDiffXd>& expressed_frame,
      const Eigen::Ref<const Eigen::MatrixX3d>& A,
      const Eigen::Ref<const Eigen::VectorXd>& lb,
      const Eigen::Ref<const Eigen::VectorXd>& ub,
      systems::Context<AutoDiffXd>* plant_context);

  ~ComInPolyhedronConstraint() override {}

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "ComInPolyhedronConstraint::DoEval() does not work for symbolic "
        "variables.");
  }

  bool use_autodiff() const { return plant_autodiff_; }

  const MultibodyPlant<double>* const plant_double_;
  std::optional<std::vector<ModelInstanceIndex>> model_instances_;
  const FrameIndex expressed_frame_index_;
  Eigen::MatrixX3d A_;
  systems::Context<double>* const context_double_{};

  const MultibodyPlant<AutoDiffXd>* const plant_autodiff_;
  systems::Context<AutoDiffXd>* const context_autodiff_{};
};
}  // namespace multibody
}  // namespace drake
