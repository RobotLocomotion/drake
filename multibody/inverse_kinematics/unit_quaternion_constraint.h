#pragma once

#include <memory>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace multibody {
/**
 * Constrains the quaternion to have a unit length.
 *
 * @ingroup solver_evaluators
 */
class UnitQuaternionConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UnitQuaternionConstraint)

  UnitQuaternionConstraint();

  ~UnitQuaternionConstraint() override {}

 private:
  template <typename T, typename S>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<S>* y) const {
    y->resize(1);
    (*y)(0) = x(0) * x(0) + x(1) * x(1) + x(2) * x(2) + x(3) * x(3);
  }

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override;
};

/**
 * Add unit length constraints to all the variables representing quaternion in
 * `q_vars`. Namely the quaternions for floating base joints in `plant` will be
 * enforced to have a unit length.
 * @param plant The plant on which we impose the unit quaternion constraints.
 * @param q_vars The decision variables for the generalized position of the
 * plant.
 * @param prog The unit quaternion constraints are added to this prog.
 * @tparam_default_scalar
 */
template <typename T>
void AddUnitQuaternionConstraintOnPlant(
    const MultibodyPlant<T>& plant,
    const Eigen::Ref<const VectorX<symbolic::Variable>>& q_vars,
    solvers::MathematicalProgram* prog);
}  // namespace multibody
}  // namespace drake
