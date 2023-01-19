#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

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
  SapConstraint(int clique, VectorX<T> g, MatrixX<T> J);

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
                MatrixX<T> J_first_clique, MatrixX<T> J_second_clique);

  virtual ~SapConstraint() = default;

  /* Number of constraint equations. */
  int num_constraint_equations() const {
    return g_.size();
  }

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
      throw std::logic_error(
          "This constraint only involves a single clique.");
    return second_clique_;
  }

  const VectorX<T>& constraint_function() const { return g_; }

  /* Returns the Jacobian with respect to the DOFs of the first clique. */
  const MatrixX<T>& first_clique_jacobian() const {
    return first_clique_jacobian_;
  }

  /* Returns the Jacobian with respect to the DOFs of the second clique.
   It throws an exception if num_cliques() == 1. */
  const MatrixX<T>& second_clique_jacobian() const {
    if (num_cliques() == 1)
      throw std::logic_error(
          "This constraint only involves a single clique.");
    return second_clique_jacobian_;
  }

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
                       MatrixX<T>* dPdy = nullptr) const = 0;

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
  virtual VectorX<T> CalcBiasTerm(const T& time_step, const T& wi) const = 0;

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
                                                const T& wi) const = 0;

  /* Derived classes must override to provide polymorphic deep-copy into a new
   instance. */
  virtual std::unique_ptr<SapConstraint<T>> Clone() const = 0;

 protected:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapConstraint);

 private:
  int first_clique_{-1};
  int second_clique_{-1};
  VectorX<T> g_;
  MatrixX<T> first_clique_jacobian_;
  MatrixX<T> second_clique_jacobian_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
