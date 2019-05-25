#pragma once

#include <string>
#include <utility>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/evaluator_base.h"

namespace drake {
namespace multibody {
// An abstract class that computes the contact wrench between a pair of
// geometries. The inputs to the Eval function are q and λ, where λ is the
// user-specified parameterization of the contact wrench. The output is the
// Fapp_AB_W, namely the 6 x 1 wrench (torque/force) applied at the witness
// point of geometry B from geometry A, expressed in the world frame.
class ContactWrenchEvaluator : public solvers::EvaluatorBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactWrenchEvaluator)

  /**
   * @anchor ComposeVariableValues
   * @name compose variable values.
   * Composes the value of the variable `x` in Eval(x, &y) function, based on
   * the context and value of lambda, x = [q, λ].
   * @pre q has size plant.num_positions() x 1
   * @pre lambda has size num_lambda_ x 1.
   */
  //@{
  /**
   * Overloads @ref ComposeVariableValues
   */
  template <typename T, typename Derived>
  typename std::enable_if<std::is_same<T, typename Derived::Scalar>::value,
                          VectorX<T>>::type
  ComposeVariableValues(const systems::Context<T>& context,
                        const Derived& lambda_value) const {
    DRAKE_ASSERT(lambda_value.rows() == num_lambda_ &&
                 lambda_value.cols() == 1);
    VectorX<T> x(num_vars());
    x.head(plant_->num_positions()) = plant_->GetPositions(context);
    x.tail(num_lambda_) = lambda_value;
    return x;
  }

  /**
   * Overloads @ref ComposeVariableValues with q, λ as the input instead of
   * context, λ.
   */
  template <typename DerivedQ, typename DerivedLambda>
  typename std::enable_if<std::is_same<typename DerivedQ::Scalar,
                                       typename DerivedLambda::Scalar>::value,
                          VectorX<typename DerivedQ::Scalar>>::type
  ComposeVariableValues(
      const Eigen::MatrixBase<DerivedQ>& q_value,
      const Eigen::MatrixBase<DerivedLambda>& lambda_value) const {
    DRAKE_ASSERT(q_value.rows() == plant_->num_positions() &&
                 q_value.cols() == 1);
    DRAKE_ASSERT(lambda_value.rows() == num_lambda_ &&
                 lambda_value.cols() == 1);
    VectorX<typename DerivedQ::Scalar> x(num_vars());
    x.head(plant_->num_positions()) = q_value;
    x.tail(num_lambda_) = lambda_value;
    return x;
  }
  //@}

  /**
   * Returns the size of lambda.
   */
  int num_lambda() const { return num_lambda_; }

  /**
   * Returns the pair of geometry IDs.
   */
  const std::pair<geometry::GeometryId, geometry::GeometryId>&
  geometry_id_pair() const {
    return geometry_id_pair_;
  }

  const MultibodyPlant<AutoDiffXd>& plant() const { return *plant_; }

  /** Getter for const context */
  const systems::Context<AutoDiffXd>& context() const { return *context_; }

  /** Getter for the mutable context */
  systems::Context<AutoDiffXd>& get_mutable_context() { return *context_; }

 protected:
  /**
   * Each derived class should call this constructor.
   * @param plant The MultibodyPlant on which the contact wrench is computed.
   * The lifetime of plant should outlive this object.
   * @param context The context of @p plant. The lifetime of context should
   * outlive this object.
   * @param num_lambda The size of lambda.
   * @param geometry_id_pair The pair of geometries for which the contact wrench
   * is computed. Notice that the order of the geometries in the pair should
   * match with that in SceneGraphInspector::GetCollisionCandidates().
   */
  ContactWrenchEvaluator(
      const MultibodyPlant<AutoDiffXd>* plant,
      systems::Context<AutoDiffXd>* context, int num_lambda,
      const std::pair<geometry::GeometryId, geometry::GeometryId>&
          geometry_id_pair,
      const std::string& description)
      : solvers::EvaluatorBase(6, plant->num_positions() + num_lambda,
                               description),
        plant_(plant),
        context_(context),
        geometry_id_pair_{geometry_id_pair},
        num_lambda_{num_lambda} {
    DRAKE_DEMAND(plant);
    DRAKE_DEMAND(context);
    DRAKE_DEMAND(num_lambda >= 0);
  }

  /**
   * Extract the generalized configuration q from x (x is used in Eval(x, &y)).
   */
  template <typename Derived>
  Eigen::VectorBlock<const Derived> q(const Derived& x) const {
    return x.head(plant_->num_positions());
  }

  /**
   * Extract lambda from x (x is used in Eval(x, &y)).
   */
  template <typename Derived>
  Eigen::VectorBlock<const Derived> lambda(const Derived& x) const {
    return x.tail(num_lambda_);
  }

 private:
  const MultibodyPlant<AutoDiffXd>* plant_;
  // The derived class might need to modify the position value stored inside
  // this context, so we don't keep context_ as a pointer to const.
  systems::Context<AutoDiffXd>* context_;
  const std::pair<geometry::GeometryId, geometry::GeometryId> geometry_id_pair_;
  int num_lambda_;
};

/**
 * The contact wrench is τ_AB_W = 0, f_AB_W = λ
 * Namely we assume that λ is the contact force from A to B, applied directly
 * at B's witness point.
 */
class ContactWrenchFromForceInWorldFrameEvaluator final
    : public ContactWrenchEvaluator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactWrenchFromForceInWorldFrameEvaluator)

  /**
   * @param plant The MultibodyPlant on which the contact wrench is computed.
   * The lifetime of @p plant should outlive this object.
   * @param context The context of the MultibodyPlant.
   * The lifetime of @p context should outlive this object.
   * @param geometry_id_pair The pair of geometries for which the contact wrench
   * is computed. Notice that the order of the geometries in the pair should
   * match with that in SceneGraphInspector::GetCollisionCandidates().
   * @param description The description of this evaluator. Default to none.
   */
  ContactWrenchFromForceInWorldFrameEvaluator(
      const MultibodyPlant<AutoDiffXd>* plant,
      systems::Context<AutoDiffXd>* context,
      const std::pair<geometry::GeometryId, geometry::GeometryId>&
          geometry_id_pair)
      : ContactWrenchEvaluator(
            plant, context, 3, geometry_id_pair,
            "contact_wrench_from_pure_force_in_world_frame") {}

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const final;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const final;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const final;

  template <typename T, typename U>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<U>* y) const;
};
}  // namespace multibody
}  // namespace drake
