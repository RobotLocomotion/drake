#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/common/value.h"
#include "drake/multibody/contact_solvers/matrix_block.h"
#include "drake/multibody/contact_solvers/sap/partial_permutation.h"

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

All SAP constraints are compliant. That is, SAP does not impose the hard
constraint g(q, v, t) = 0 but rather applies a compliant force that drives the
state towards g(q, v, t) = 0. More specifically, the impulse γ applied by a
constraint in the SAP formulation admits the analytical form:
  γ/δt = P(−k⋅g−cġ)                                                         (1)
where δt is the discrete time step used in the formulation, k is the
constraint's stiffness, c is its linear dissipation and P is a projection
operator. For instance, for contact constraints, P projects the compliant
impulse y = −k⋅g−cġ onto the friction cone, see the documentation of
SapConstraint::Project() for details. We parameterize the linear dissipation c
in terms of a "dissipation time scale" τ (in seconds) as c=τ⋅k.

A particular constraint defines a fixed number nᵢ of constraint equations, see
num_constraint_equations() (E.g. 3 for contact constraints, 1 for a distance
constraint). Then γ, g, ġ in Eq. (1) are all vectors of size nᵢ and stiffness k
and damping c are positive diagonal matrices of size nᵢ×nᵢ. Then the projection
P projects vectors in the real nᵢ-space into vectors in the same space. Refer to
Project()'s documentation for further details.

The constraint's Jacobian J is defined such that ġ = J⋅v + b where v are the
generalized velocities of the system and b is the bias term b = ∂g/∂t. When a
constraint is instantiated, constraint function g and Jacobian J are provided at
construction for the state of the mechanical system at the previous (current)
time step. Therefore an instance of this class is a function of the state of the
dynamical system.

In general a constraint will couple the degrees of freedom of two cliques
(please refer to SapContactProblem's documentation for a definition of
cliques). Therefore ġ = J⋅v = J₁⋅v₁ + J₂⋅v₂ where v₁ and v₂ correspond to the
vector of generalized velocities for the first and second clique respectively
(which clique is designated as first or second is arbitrary). Similarly, J₁ and
J₂ are the non-zero blocks of the full Jacobian J that correspond to
contributions from the first and second cliques respectively. A constraint can
also involve only a single clique. Examples include a coupler constraint between
DOFs in a single clique or a contact constraint with the world (the world has no
DOFs and therefore has no clique.) For the case of a single clique constraint,
only one clique and a single block J₁ needs to be provided at construction. For
details, refer to the documentation for this class's constructors.

TODO(amcastro-tri): consider extension to support constraints among more than
two cliques, see issue #16575.

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

  /* Constructor for a constraint among DOFs within a single `clique`.
   @param[in] clique
     Index of a clique in the SapContactProblem where this constraint will be
     added. It must be non-negative or an exception is thrown.
   @param[in] g
     Value of the constraint function (see this class's documentation) evaluated
     at the previous time step state of the mechanical system. g.size()
     corresponds to the number of constraint equations, see
     num_constraint_equations(). The size must be strictly positive or an
     exception is thrown.
   @param[in] J
     Jacobian of g with respect to the DOFs for `clique` only. J.rows() must
     equal g.size() or an exception is thrown. J.cols() must match the number of
     generalized velocities for `clique`, see
     SapContactProblem::num_velocities(). This condition is enforced when the
     constraint is added to the contact problem instead of during construction
     here, see SapContactProblem::AddConstraint(). */
  SapConstraint(int clique, VectorX<T> g, MatrixBlock<T> J);

  /* Alternative signature for the constructor for a constraint within a single
   clique. that takes a dense Jacobian. */
  SapConstraint(int clique, VectorX<T> g, MatrixX<T> J)
      : SapConstraint(clique, std::move(g), MatrixBlock<T>(std::move(J))) {}

  /* Constructor for a constraint among DOFs between two cliques.
   @param[in] first_clique
     Index of a clique in the SapContactProblem where this constraint will be
     added. It must be non-negative or an exception is thrown.
   @param[in] second_clique
     Index of a clique in the SapContactProblem where this constraint will be
     added. It must be non-negative and different from first_clique or an
     exception is thrown.
   @param[in] g
     Value of the constraint function (see this class's documentation) evaluated
     at the previous time step state of the mechanical system. g.size()
     corresponds to the number of constraint equations, see
     num_constraint_equations(). The size must be strictly positive or an
     exception is thrown.
   @param[in] J_first_clique
     Jacobian of g with respect to the DOFs of the first clique only.
     J_first_clique.rows() must equal g.size() or an exception is thrown.
     J_first_clique.cols() must match the number of generalized velocities for
     `first_clique`, see SapContactProblem::num_velocities(). This condition is
     enforced when the constraint is added to the contact problem instead of
     during construction here, see SapContactProblem::AddConstraint()
   @param[in] J_second_clique
     Jacobian of g with respect to the DOFs of the second clique only.
     J_second_clique.rows() must equal g.size() or an exception is thrown.
     J_second_clique.cols() must match the number of generalized velocities for
     `second_clique`, see SapContactProblem::num_velocities(), though this
     condition is not enforced at construction but when the constraint is added
     to the contact problem, see SapContactProblem::AddConstraint(). */
  SapConstraint(int first_clique, int second_clique, VectorX<T> g,
                MatrixBlock<T> J_first_clique, MatrixBlock<T> J_second_clique);

  /* Alternative signature for the constructor for constraint between two
   cliques that takes dense Jacobians. */
  SapConstraint(int first_clique, int second_clique, VectorX<T> g,
                MatrixX<T> J_first_clique, MatrixX<T> J_second_clique)
      : SapConstraint(first_clique, second_clique, std::move(g),
                      MatrixBlock<T>(std::move(J_first_clique)),
                      MatrixBlock<T>(std::move(J_second_clique))) {}

  virtual ~SapConstraint() = default;

  /* Number of constraint equations. */
  int num_constraint_equations() const { return g_.size(); }

  /* Number of participating cliques. It will always return either one (1) or
   two (2). */
  int num_cliques() const { return second_clique_ < 0 ? 1 : 2; }

  /* Index of the first (and maybe only) participating clique. Always a
   non-negative number. */
  int first_clique() const { return first_clique_; }

  /* Index of the second participating clique. It throws an exception if
   num_cliques() == 1. */
  int second_clique() const {
    if (num_cliques() == 1)
      throw std::logic_error("This constraint only involves a single clique.");
    return second_clique_;
  }

  const VectorX<T>& constraint_function() const { return g_; }

  /* Returns the Jacobian with respect to the DOFs of the first clique. */
  const MatrixBlock<T>& first_clique_jacobian() const {
    return first_clique_jacobian_;
  }

  /* Returns the Jacobian with respect to the DOFs of the second clique.
   It throws an exception if num_cliques() == 1. */
  const MatrixBlock<T>& second_clique_jacobian() const {
    if (num_cliques() == 1)
      throw std::logic_error("This constraint only involves a single clique.");
    return second_clique_jacobian_;
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
   @pre data does not equal nullptr. */
  void CalcData(const Eigen::Ref<const VectorX<T>>& vc,
                AbstractValue* data) const;

  /* Computes the constraint cost ℓ(vc) function of constraint velocities vc.
   @post The cost ℓ(vc) is a convex function of vc, as required by the SAP
   formulation. */
  T CalcCost(const AbstractValue& data) const;

  /* Computes the impulse according to:
       γ(vc) = −∂ℓ/∂vc.
   @pre gamma does not equal nullptr. */
  void CalcImpulse(const AbstractValue& data, EigenPtr<VectorX<T>> gamma) const;

  /* Computes the constraint Hessian as:
       G(vc) = ∂²ℓ/∂vc² = -∂γ/∂vc.
   @post G is of size nₑ×nₑ, with nₑ equal to num_constraint_equations().
   @post The constraint Hessian G(vc) is
   symmetric positive semi-definite, since ℓ(vc) is a convex function of vc as
   required by the SAP formulation.
   @pre G does not equal nullptr. */
  void CalcCostHessian(const AbstractValue& data, MatrixX<T>* G) const;

  /* Polymorphic deep-copy into a new instance. */
  std::unique_ptr<SapConstraint<T>> Clone() const { return DoClone(); }

  /* Creates a clone, cᵣ, of this constraint, c, modified by the clique and dof
    permutation given by `clique_permutation` and `per_clique_locked_dofs`. The
    following table breaks down how the returned constraint relates to this
    constraint where participates(x) is true iff index x participates in the
    partial permutation given by `clique_permutation` and permuted(x) is its
    permuted index. For convenience c.first_clique = i and c.second_clique = j
    and cᵣ.first_clique = iᵣ and cᵣ.second_clique = jᵣ:
     ________________________________________________________________________
    | participates(i) | participates(j) |        iᵣ       |         jᵣ       |
    |------------------------------------------------------------------------|
    |     FALSE       |     FALSE       |         No constraint made.        |
    |     TRUE        |     FALSE       |   permuted(i)   |    permuted(j)   |
    |     FALSE       |     TRUE        |   permuted(j)   |        -1        |
    |     TRUE        |     TRUE        |   permuted(i)   |    permuted(j)   |
    |------------------------------------------------------------------------|

    The constraint Jacobian for a participating clique is modified by removing
    all columns that correspond to dofs included in
    `per_clicked_locked_dofs[clique_index]`.

    Note: if both cliques do not participate, this function returns `nullptr`.
  */
  std::unique_ptr<SapConstraint<T>> MakeReduced(
      const PartialPermutation& clique_permutation,
      const std::vector<std::vector<int>>& per_clique_locked_dofs) const;

 protected:
  /* Protected copy construction is enabled for sub-classes to use in their
   implementation of DoClone(). */
  SapConstraint(const SapConstraint&) = default;

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
  int first_clique_{-1};
  int second_clique_{-1};
  VectorX<T> g_;
  MatrixBlock<T> first_clique_jacobian_;
  MatrixBlock<T> second_clique_jacobian_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
