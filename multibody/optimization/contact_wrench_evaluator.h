#pragma once

#include <utility>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/evaluator_base.h"

namespace drake {
namespace multibody {
// An abstract class that computes the contact wrench between a pair of
// geometry. The input to the Eval function is λ, the user-specified
// parameterization of the contact wrench. The output is the Fapp_B_W, namely
// the wrench (torque/force) applied at the witness point of geometry B from
// geometryA, expressed in the world frame.
class ContactWrenchEvaluator : public solvers::EvaluatorBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactWrenchEvaluator)

  /**
   * Set the value of the variable `x` in Eval(x, &y) function, based on the
   * context and value of lambda.
   */
  template <typename T, typename Derived>
  typename std::enable_if<std::is_same<T, typename Derived::Scalar>::value,
                          VectorX<T>>::type
  SetVariableValues(const systems::Context<T>& context,
                    const Derived& lambda_value) const {
    VectorX<T> x(num_vars());
    x.template head(plant_->num_positions()) = plant_->GetPositions(context);
    x.template tail(num_lambda_) = lambda_value;
    return x;
  }

  /**
   * Returns the size of lambda.
   */
  int num_lambda() const { return num_lambda_; }

  const std::pair<geometry::GeometryId, geometry::GeometryId>&
  geometry_id_pair() const {
    return geometry_id_pair_;
  }

 protected:
  ContactWrenchEvaluator(
      const MultibodyPlant<AutoDiffXd>* plant,
      systems::Context<AutoDiffXd>* context, int num_lambda,
      const std::pair<geometry::GeometryId, geometry::GeometryId>&
          geometry_id_pair)
      : solvers::EvaluatorBase(6, plant->num_positions() + num_lambda),
        plant_(plant),
        context_(context),
        geometry_id_pair_{geometry_id_pair},
        num_lambda_{num_lambda} {}

 protected:
  template <typename Derived>
  Eigen::VectorBlock<const Derived> q(const Derived& x) const {
    return x.head(plant_->num_positions());
  }

  template <typename Derived>
  Eigen::VectorBlock<const Derived> lambda(const Derived& x) const {
    return x.tail(num_lambda_);
  }

  systems::Context<AutoDiffXd>* get_mutable_context() const { return context_; }

 private:
  const MultibodyPlant<AutoDiffXd>* plant_;
  systems::Context<AutoDiffXd>* context_;
  const std::pair<geometry::GeometryId, geometry::GeometryId> geometry_id_pair_;
  int num_lambda_;
};

/**
 * The contact wrench is τ_B_W = 0, f_B_W = λ
 */
class ContactWrenchFromForceInWorldFrameEvaluator
    : public ContactWrenchEvaluator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactWrenchFromForceInWorldFrameEvaluator)

  ContactWrenchFromForceInWorldFrameEvaluator(
      const MultibodyPlant<AutoDiffXd>* plant,
      systems::Context<AutoDiffXd>* context,
      const std::pair<geometry::GeometryId, geometry::GeometryId>&
          geometry_id_pair)
      : ContactWrenchEvaluator(plant, context, 3, geometry_id_pair) {}

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

  template <typename T, typename U>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<U>* y) const;
};
}  // namespace multibody
}  // namespace drake
