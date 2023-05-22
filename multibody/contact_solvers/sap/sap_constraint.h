#pragma once

#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/common/value.h"
#include "drake/multibody/contact_solvers/matrix_block.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint_jacobian.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// TODO(amcastro-tri): SapConstraintData is only temporary to aid the migration
// towards #19392 and it will soon be removed.
/* Class to store data needed for SapConstraint computations. */
template <typename T>
class SapConstraintData {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapConstraintData);

  /* Constructs data for a SapLimitConstraint.
     @param R Regularization parameters.
     @param v_hat Bias term.
     @warning Data stored is left uninitialized to avoid the cost of an
     unnecessary initialization. */
  SapConstraintData(VectorX<T> R, VectorX<T> v_hat) {
    const int nk = R.size();
    parameters_.R.resize(nk);
    parameters_.R_inv.resize(nk);
    parameters_.v_hat.resize(nk);
    parameters_.R = R;
    parameters_.R_inv = R.cwiseInverse();
    parameters_.v_hat = v_hat;
    vc_.resize(nk);
    y_.resize(nk);
    gamma_.resize(nk);
    dPdy_.resize(nk, nk);
  }

  /* Regularization R. */
  const VectorX<T>& R() const { return parameters_.R; }

  /* Inverse of the regularization, R⁻¹. */
  const VectorX<T>& R_inv() const { return parameters_.R_inv; }

  /* Constraint bias. */
  const VectorX<T>& v_hat() const { return parameters_.v_hat; }

  /* Const access. */
  const VectorX<T>& vc() const { return vc_; }
  const VectorX<T>& y() const { return y_; }
  const VectorX<T>& gamma() const { return gamma_; }
  const MatrixX<T>& dPdy() const { return dPdy_; }

  /* Mutable access. */
  VectorX<T>& mutable_vc() { return vc_; }
  VectorX<T>& mutable_y() { return y_; }
  VectorX<T>& mutable_gamma() { return gamma_; }
  MatrixX<T>& mutable_dPdy() { return dPdy_; }

 private:
  // This struct stores parameters that remain const after construction.
  struct ConstParameters {
    VectorX<T> R;      // Regularization R.
    VectorX<T> R_inv;  // Inverse of regularization R.
    VectorX<T> v_hat;  // Constraint velocity bias.
  };
  ConstParameters parameters_;

  VectorX<T> vc_;
  VectorX<T> y_;      // Un-projected impulse y = −R⁻¹⋅(vc−v̂)
  VectorX<T> gamma_;  // Impulse.
  MatrixX<T> dPdy_;   // Gradient of the projection γ = P(y) w.r.t. y.
};

/* This class serves to represent constraints supported by the SapSolver as
described in [Castro et al., 2021].

All SAP constraints are described by a convex cost function ℓ(vc), where vc is
the constraint velocity. The constraint velocity (along with sign conventions
used by specific constraints) is defined at construction given the constraint
Jacobian J such that vc = J⋅v, where v are the generalized velocities of the
SapContactProblem. The size of vc defines a fixed number nₑ of constraint
equations, see num_constraint_equations() (E.g. 3 for contact constraints, 1 for
a distance constraint).

In this formulation, constraint impulses are simply defined by:
  γ(vc) = −∂ℓ/∂vc
where γ is also a vector of size nₑ. The Hessian of the constraint is then
given by:
  G(vc) = −∂γ/∂vc = ∂²ℓ/∂vc²
of size nₑ×nₑ, it is positive semi-definite given the cost function ℓ(vc) is
convex.

Specific constraint types will provide a definition for ℓ(vc), γ(vc) and G(vc)
with the implementation of DoCalcCost(), DoCalcImpulse(), and DoCalcHessian(),
respectively.

@tparam_nonsymbolic_scalar */
template <typename T>
class SapConstraint {
 public:
  /* We do not allow copy, move, or assignment generally to avoid slicing.
   Protected copy construction is enabled for sub-classes to use in their
   implementation of DoClone(). */
  //@{
  SapConstraint& operator=(const SapConstraint&) = delete;
  SapConstraint(SapConstraint&&) = delete;
  SapConstraint& operator=(SapConstraint&&) = delete;
  //@}

  /* Constructor for a SAP constraint given its Jacobian J.
   SAP constraints are defined by their convex cost ℓ(vc) function of the
   constraint velocity vc. The constraint velocity in turn is defined by the
   constraint's Jacobian J such that vc = J⋅v, where v are the generalized
   velocities of the contact problem.

   @throws if J.rows() equals zero. */
  explicit SapConstraint(SapConstraintJacobian<T> J);

  virtual ~SapConstraint() = default;

  /* Number of constraint equations. */
  int num_constraint_equations() const { return J_.rows(); }

  /* Number of participating cliques. It will always return either one (1) or
   two (2). */
  int num_cliques() const { return J_.num_cliques(); }

  int num_velocities(int clique) const {
    DRAKE_THROW_UNLESS(0 <= clique && clique < num_cliques());
    return J_.clique_jacobian(clique).cols();
  }

  /* Index of the first (and maybe only) participating clique. Always a
   non-negative number. */
  int first_clique() const { return J_.clique(0); }

  /* Index of the second participating clique. It throws an exception if
   num_cliques() == 1. */
  int second_clique() const {
    if (num_cliques() == 1)
      throw std::logic_error("This constraint only involves a single clique.");
    return J_.clique(1);
  }

  const SapConstraintJacobian<T>& jacobian() const { return J_; }

  /* Returns the Jacobian with respect to the DOFs of the first clique. */
  const MatrixBlock<T>& first_clique_jacobian() const {
    return J_.clique_jacobian(0);
  }

  /* Returns the Jacobian with respect to the DOFs of the second clique.
   It throws an exception if num_cliques() == 1. */
  const MatrixBlock<T>& second_clique_jacobian() const {
    if (num_cliques() == 1)
      throw std::logic_error("This constraint only involves a single clique.");
    return J_.clique_jacobian(1);
  }

  /* Makes data used by this constraint to perform computations. Different
   constraints can opt to use `time_step` and a diagonal approximation of the
   Delassus operator in `delassus_estimation` to pre-compute scale quantities to
   condition the problem better.
   N.B. Specific constraints can choose to ignore `time_step` and
   `delassus_estimation` making a new data that does not depend on either of
   them. Refer to the documentation for specific constraint types.
   @pre delassus_estimation.size() equals num_constraint_equations(). */
  std::unique_ptr<AbstractValue> MakeData(
      const T& time_step,
      const Eigen::Ref<const VectorX<T>>& delassus_estimation) const;

  /* Computes constraint data as a function of constraint velocities `vc`.
   This method can be as simple as a copy of `vc` into `data`, though specific
   constraints might want to compute commonly occurring terms in the computation
   of the cost, impulses and Hessian into `data` to be reused.
   @pre vc.size() equals num_constraint_equations().
   @pre data does not equal nullptr.
   @pre data was created via MakeData() on `this` object and therefore the type
   wrapped by AbstractValue is consistent with the concrete subclass. */
  void CalcData(const Eigen::Ref<const VectorX<T>>& vc,
                AbstractValue* data) const;

  /* Computes the constraint cost ℓ(vc) function of constraint velocities vc.
   @post The cost ℓ(vc) is a convex function of vc, as required by the SAP
   formulation.
   @pre data was created via MakeData() on `this` object and therefore the type
   wrapped by AbstractValue is consistent with the concrete subclass. */
  T CalcCost(const AbstractValue& data) const;

  /* Computes the impulse according to:
       γ(vc) = −∂ℓ/∂vc.
   @pre gamma does not equal nullptr.
   @pre data was created via MakeData() on `this` object and therefore the type
   wrapped by AbstractValue is consistent with the concrete subclass. */
  void CalcImpulse(const AbstractValue& data, EigenPtr<VectorX<T>> gamma) const;

  /* Computes the constraint Hessian as:
       G(vc) = ∂²ℓ/∂vc² = -∂γ/∂vc.
   @post G is of size nₑ×nₑ, with nₑ equal to num_constraint_equations().
   @post The constraint Hessian G(vc) is
   symmetric positive semi-definite, since ℓ(vc) is a convex function of vc as
   required by the SAP formulation.
   @pre G does not equal nullptr.
   @pre data was created via MakeData() on `this` object and therefore the type
   wrapped by AbstractValue is consistent with the concrete subclass. */
  void CalcCostHessian(const AbstractValue& data, MatrixX<T>* G) const;

  /* Polymorphic deep-copy into a new instance. */
  std::unique_ptr<SapConstraint<T>> Clone() const { return DoClone(); }

 protected:
  /* Protected copy construction is enabled for sub-classes to use in their
   implementation of DoClone(). */
  SapConstraint(const SapConstraint&) = default;

  /* Helper to pack `data` of type `U` into an AbstractValue.
   Derived constraint classes can use this helper to implement DoMakeData(). */
  template <typename U>
  static std::unique_ptr<AbstractValue> MoveAndMakeAbstractValue(U&& data) {
    auto owned_data = std::make_unique<U>(std::move(data));
    return std::make_unique<Value<U>>(std::move(owned_data));
  }

  // @group NVI implementations. Specific constraints must implement these
  // methods. Refer to the specific NVI documentation for details.
  // Proper argument sizes and valid non-null pointers are already guaranteed by
  // checks in the correspondng NVIs.
  // TODO(amcastro-tri): Make these pure virtual per #19392.
  // @{
  virtual std::unique_ptr<AbstractValue> DoMakeData(
      const T& time_step,
      const Eigen::Ref<const VectorX<T>>& delassus_estimation) const;
  virtual void DoCalcData(const Eigen::Ref<const VectorX<T>>& vc,
                          AbstractValue* data) const;
  virtual T DoCalcCost(const AbstractValue& data) const;
  virtual void DoCalcImpulse(const AbstractValue& data,
                             EigenPtr<VectorX<T>> gamma) const;
  virtual void DoCalcCostHessian(const AbstractValue& data,
                                 MatrixX<T>* G) const;

  /* Clone() implementation. Derived classes must override to provide
   polymorphic deep-copy into a new instance. */
  virtual std::unique_ptr<SapConstraint<T>> DoClone() const = 0;

  // @}

  // TODO(amcastro-tri): Remove this group of methods below per #19392.
  // N.B. To aid migration towards #19392, the default implementations all
  // abort. This will guarantee that all current (and future) derived classes
  // have overridden these methods; failure to do so would lead to a program
  // abort. This allows older implementations in terms of these methods to
  // coexist with newly migrated constraints that implement the new DoCalcFoo()
  // APIs. Once the migration is complete, these virtual APIs will be removed
  // and the DoCalcFoo() APIs will be made pure virtual.

  /* Computes the projection γ = P(y) onto the convex set specific to a
   constraint in the norm defined by the diagonal positive matrix R, i.e. the
   norm ‖x‖ = sqrt(xᵀ⋅R⋅x). Refer to [Castro et al., 2021] for details.
   @param[in] y Impulse to be projected, of size num_constraint_equations().
   @param[in] R Specifies the diagonal components of matrix R, of size
   num_constraint_equations().
   @param[in,out] gamma On output, the projected impulse γ. On input it must be
   of size num_constraint_equations().
   @param[in,out] dPdy Not used if nullptr. Otherwise it must be a squared
   matrix of size num_constraint_equations() to store the Jacobian dP/dy on
   output. */
  virtual void Project(const Eigen::Ref<const VectorX<T>>& y,
                       const Eigen::Ref<const VectorX<T>>& R,
                       EigenPtr<VectorX<T>> gamma,
                       MatrixX<T>* dPdy = nullptr) const {
    unused(y);
    unused(R);
    unused(gamma);
    unused(dPdy);
    DRAKE_UNREACHABLE();
  }

  /* Computes the bias term v̂ used to compute the constraint impulses before
   projection y = −R⁻¹⋅(vc − v̂).
   @param[in] time_step The time step used in the temporal discretization.
   @param[in] wi Approximation of the inverse of the mass for this
   constraint. Specific constraints can use this information to estimate
   stabilization terms in the "near-rigid" regime. Refer to [Castro et al.,
   2021] for details.
   @note Parameter wi provides a scale factor, with units of inverse of mass.
   Thus far we are assuming the same scalar factor can be used for all
   constraint equations effectively.
   TODO(amcastro-tri): Consider making wi a vector quantity. */
  virtual VectorX<T> CalcBiasTerm(const T& time_step, const T& wi) const {
    unused(time_step);
    unused(wi);
    DRAKE_UNREACHABLE();
  }

  /* Computes the regularization R used to compute the constraint impulses
   before projection as y = −R⁻¹⋅(vc − v̂).
   @param[in] time_step The time step used in the temporal discretization.
   @param[in] wi Approximation of the inverse of the mass for this
   constraint. Specific constraints can use this information to estimate
   stabilization terms in the "near-rigid" regime. Refer to [Castro et al.,
   2021] for details.
   @note Parameter wi provides a scale factor, with units of inverse of mass.
   Thus far we are assuming the same scalar factor can be used for all
   constraint equations effectively.
   TODO(amcastro-tri): Consider making wi a vector quantity. */
  virtual VectorX<T> CalcDiagonalRegularization(const T& time_step,
                                                const T& wi) const {
    unused(time_step);
    unused(wi);
    DRAKE_UNREACHABLE();
  }

 private:
  SapConstraintJacobian<T> J_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
