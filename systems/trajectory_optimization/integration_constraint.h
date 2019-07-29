#pragma once

#include "drake/solvers/constraint.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
/**
 * Implements the midpoint integration
 *
 *     (ẋₗ + ẋᵣ)/2  * dt = xᵣ - xₗ
 * where the bounded variables are (xᵣ, xₗ, ẋᵣ, ẋₗ, dt)
 */
class MidPointIntegrationConstraint : public solvers::Constraint {
 public:
  explicit MidPointIntegrationConstraint(int dim);

  ~MidPointIntegrationConstraint() override {}

  /**
   * Compose x for the Eval input from individual variables.
   */
  template <typename T>
  void ComposeX(const Eigen::Ref<const VectorX<T>>& x_r,
                const Eigen::Ref<const VectorX<T>>& x_l,
                const Eigen::Ref<const VectorX<T>>& xdot_r,
                const Eigen::Ref<const VectorX<T>>& xdot_l, const T& dt,
                VectorX<T>* x) const {
    x->resize(num_vars());
    x->head(dim_) = x_r;
    x->segment(dim_, dim_) = x_l;
    x->segment(2 * dim_, dim_) = xdot_r;
    x->segment(3 * dim_, dim_) = xdot_l;
    (*x)(num_vars() - 1) = dt;
  }

 private:
  template <typename T>
  void DecomposeX(const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* x_r,
                  VectorX<T>* x_l, VectorX<T>* xdot_r, VectorX<T>* xdot_l,
                  T* dt) const {
    *x_r = x.head(dim_);
    *x_l = x.segment(dim_, dim_);
    *xdot_r = x.segment(2 * dim_, dim_);
    *xdot_l = x.segment(3 * dim_, dim_);
    *dt = x(num_vars() - 1);
  }

  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x,
                     VectorX<T>* y) const;

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

  int dim_;
};
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
