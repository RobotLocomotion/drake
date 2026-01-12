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
#include "drake/multibody/contact_solvers/sap/sap_constraint_jacobian.h"
#include "drake/multibody/math/spatial_algebra.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

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

  /* Constructor for a SAP constraint given its Jacobian J and the objects it
   constraints, see @ref sap_physical_forces.

   SAP constraints are defined by their convex cost ℓ(vc) function
   of the constraint velocity vc. The constraint velocity in turn is defined by
   the constraint's Jacobian J such that vc = J⋅v, where v are the generalized
   velocities of the contact problem.

   Specific constraint types might lead to generalized forces on the cliques
   participating in J, to spatial forces on the physical objects being
   constrained, or both. See @ref sap_physical_forces for details. For
   constraints among physical objects, object indices are provided in `objects`.

   @param[in] J The contact Jacobian. This defines the cliques involved in the
   constraint (see SapConstraintJacobian) and the constraint velocity as vc =
   J⋅v.
   @param[in] objects Indices to the physical objects involved in this
   constraint. It can be empty for constraints that only lead to generalized
   forces, see @ref sap_physical_forces. num_objects() will equal objects.size()
   after construction.

   @throws if J.rows() equals zero. */
  SapConstraint(SapConstraintJacobian<T> J, std::vector<int> objects);

  virtual ~SapConstraint();

  /* Number of constraint equations. */
  int num_constraint_equations() const { return J_.rows(); }

  /* Number of participating cliques. It will always return either one (1) or
   two (2). */
  int num_cliques() const { return J_.num_cliques(); }

  /* Returns the clique index for the first clique (c = 0) or for the
   second clique (c = 1).
   @throws if c < 0 or c >= num_cliques().*/
  int clique(int c) const {
    DRAKE_THROW_UNLESS(0 <= c && c < num_cliques());
    return J_.clique(c);
  }

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
   This method can be as simple as a copy of `vc` into `abstract_data`, though
   specific constraints might want to compute commonly occurring terms in the
   computation of the cost, impulses and Hessian into `data` to be reused.
   @pre vc.size() equals num_constraint_equations().
   @pre abstract_data does not equal nullptr.
   @pre abstract_data was created via MakeData() on `this` object and therefore
   the type wrapped by AbstractValue is consistent with the concrete subclass.
  */
  void CalcData(const Eigen::Ref<const VectorX<T>>& vc,
                AbstractValue* abstract_data) const;

  /* Computes the constraint cost ℓ(vc) function of constraint velocities vc.
   @post The cost ℓ(vc) is a convex function of vc, as required by the SAP
   formulation.
   @pre abstract_data was created via MakeData() on `this` object and therefore
   the type wrapped by AbstractValue is consistent with the concrete subclass.
  */
  T CalcCost(const AbstractValue& abstract_data) const;

  /* Computes the impulse according to:
       γ(vc) = −∂ℓ/∂vc.
   @pre gamma does not equal nullptr.
   @pre abstract_data was created via MakeData() on `this` object and therefore
   the type wrapped by AbstractValue is consistent with the concrete subclass.
  */
  void CalcImpulse(const AbstractValue& abstract_data,
                   EigenPtr<VectorX<T>> gamma) const;

  /* Computes the constraint Hessian as:
       G(vc) = ∂²ℓ/∂vc² = -∂γ/∂vc.
   @post G is of size nₑ×nₑ, with nₑ equal to num_constraint_equations().
   @post The constraint Hessian G(vc) is
   symmetric positive semi-definite, since ℓ(vc) is a convex function of vc as
   required by the SAP formulation.
   @pre G does not equal nullptr.
   @pre abstract_data was created via MakeData() on `this` object and therefore
   the type wrapped by AbstractValue is consistent with the concrete subclass.
  */
  void CalcCostHessian(const AbstractValue& abstract_data, MatrixX<T>* G) const;

  /* @name Physical forces
   @anchor sap_physical_forces
   Each SAP constraint leads to a set of impulses in the solution of their
   corresponding SapContactProblem. These impulses map to physical generalized
   and/or spatial forces on the physical objects being constrained. As an
   example, a distance constraint leads to a single scalar impulse value,
   however the actual physical forces are vectors that act along the direction
   of the two points being constrained. These forces are applied on the objects
   being constrained, e.g. a rigid body, deformable body or particle. It is the
   specific constraint type that knows how to map these impulses to physical
   generalized and/or spatial forces. In the constraints framework to report
   forces, client code (in Drake, most likely the SapDriver) will label these
   physical objects with indices, arbitrarily defined to uniquely identify
   distinct physical objects. With these labels defined, the constraint will be
   able to report physical forces (or rather impulses) on these objects via
   AccumulateSpatialImpulses().

   Specific constraint types might require these object indices at construction
   (e.g. the contact contraint between two objects). Object indices, if any, are
   provided at construction. Refer to this class's constructor for details.

   Generalized impulses are reported on a per-clique basis via
   AccumulateGeneralizedImpulses(). */
  //@{

  /* Number of physical objects constrained by `this` constraint.
   Number of objects and their indices are provided at construction. */
  int num_objects() const { return objects_.size(); }

  /* Returns the vector of object indices provided at construction. */
  const std::vector<int>& objects() const { return objects_; }

  /* Returns the index for the o-the object constrained by `this` constraint.
   @throws if o < 0 > or if o >= num_objects(). */
  int object(int o) const {
    DRAKE_THROW_UNLESS(0 <= o && o < num_objects());
    return objects_[o];
  }
  //@}

  /* Accumulates generalized impulses applied by this constraint on the c-th
   clique.
   @param[in] c The c-th clique of index clique(c).
   @param[in] gamma Impulses for this constraint, of size
   num_constraint_equations().
   @param[out] tau On output this function will accumulate the generalized
   impulses applied by this constraint on the c-th clique.

   @throws if c < 0 or if c >= num_cliques().
   @throws if gamma.size() != num_constraint_equations().
   @throws if tau is the nullptr.
   @throws if tau.size() != num_velocities(c). */
  void AccumulateGeneralizedImpulses(int c,
                                     const Eigen::Ref<const VectorX<T>>& gamma,
                                     EigenPtr<VectorX<T>> tau) const;

  /* Accumulates spatial impulses applied by this constraint the o-th object.
   @param[in] o The o-th object of index object(o).
   @param[in] gamma Impulses for this constraint, of size
   num_constraint_equations().
   @param[out] F On output this function will accumulate the spatial impulse
   applied by this constraint on the o-th object.

   @throws if o < 0 or if o >= num_objects().
   @throws if gamma.size() != num_constraint_equations().
   @throws if F is the nullptr. */
  void AccumulateSpatialImpulses(int o,
                                 const Eigen::Ref<const VectorX<T>>& gamma,
                                 SpatialForce<T>* F) const;

  /* Polymorphic deep-copy into a new instance. */
  std::unique_ptr<SapConstraint<T>> Clone() const { return DoClone(); }

  /* When T = double, this method is equivalent to Clone() and returns a
   deep-copy of `this` jacobian. When T = AutoDiffXd this method returns a copy
   where gradients were discarded. */
  std::unique_ptr<SapConstraint<double>> ToDouble() const {
    return DoToDouble();
  }

  /* Creates a "reduced" clone of this constraint by removing known DoFs from
   the constraint's Jacobian. That is, the newly reduced constraint will have
   this constraint's Jacobian excluding columns for known DoFs. The following
   table breaks down how the returned constraint relates to this constraint
   where:
     participates(clique_index) = clique_permutation.participates(clique_index)
     permuted(clique_index) = clique_permutation.permuted_index(clique_index)
   For convenience: c.first_clique  = i   c.second_clique  = j
                    cᵣ.first_clique = iᵣ  cᵣ.second_clique = jᵣ
     ________________________________________________________________________
    | participates(i) | participates(j) |        iᵣ       |         jᵣ       |
    |------------------------------------------------------------------------|
    |     FALSE       |     FALSE       |       N/A       |       N/A        |
    |     TRUE        |     FALSE       |   permuted(i)   |       N/A        |
    |     FALSE       |     TRUE        |   permuted(j)   |       N/A        |
    |     TRUE        |     TRUE        |   permuted(i)   |    permuted(j)   |
    |------------------------------------------------------------------------|
    Note: if both cliques do not participate, this function returns `nullptr`.
    Note: per_clique_known_dofs.size() can be smaller than
    clique_permutation.domain_size() (i.e. the number of cliques in the problem)
    for cases in which the last set of cliques have zero known DoFs and it is
    therefore unnecessary to pad per_clique_known_dofs with empty vectors.
    @pre per_clique_known_dofs.size() <= clique_permutation.domain_size() .
    @pre first_clique() < clique_permutation.domain_size().
    @pre if num_cliques() > 1,
      second_clique() < clique_permutation.domain_size().
  */
  std::unique_ptr<SapConstraint<T>> MakeReduced(
      const PartialPermutation& clique_permutation,
      const std::vector<std::vector<int>>& per_clique_known_dofs) const;

 protected:
  /* Protected copy construction is enabled for sub-classes to use in their
   implementation of DoClone(). */
  SapConstraint(const SapConstraint&) = default;

  /* Helper to pack `data` of type `U` into an AbstractValue. Derived constraint
   classes can use this helper to implement DoMakeData(). */
  template <typename U>
  static std::unique_ptr<AbstractValue> MoveAndMakeAbstractValue(U&& data) {
    auto owned_data = std::make_unique<U>(std::move(data));
    return std::make_unique<Value<U>>(std::move(owned_data));
  }

  // @name NVI implementations. Specific constraints must implement these
  // methods. Refer to the specific NVI documentation for details.
  // Proper argument sizes and valid non-null pointers are already guaranteed by
  // checks in the correspondng NVIs.
  // @{
  virtual std::unique_ptr<AbstractValue> DoMakeData(
      const T& time_step,
      const Eigen::Ref<const VectorX<T>>& delassus_estimation) const = 0;
  virtual void DoCalcData(const Eigen::Ref<const VectorX<T>>& vc,
                          AbstractValue* data) const = 0;
  virtual T DoCalcCost(const AbstractValue& data) const = 0;
  virtual void DoCalcImpulse(const AbstractValue& data,
                             EigenPtr<VectorX<T>> gamma) const = 0;
  virtual void DoCalcCostHessian(const AbstractValue& data,
                                 MatrixX<T>* G) const = 0;
  virtual void DoAccumulateGeneralizedImpulses(
      int, const Eigen::Ref<const VectorX<T>>&, EigenPtr<VectorX<T>>) const {
    // TODO(amcastro-tri): Temporarily, the default implementation throws until
    // the full resolution of #19435. Once all constraints report forces, this
    // function will be pure virtual.
    throw std::logic_error(
        "Constraints must implement this function. See #19435. ");
  }
  virtual void DoAccumulateSpatialImpulses(int,
                                           const Eigen::Ref<const VectorX<T>>&,
                                           SpatialForce<T>*) const {
    // TODO(amcastro-tri): Temporarily, the default implementation throws until
    // the full resolution of #19435. Once all constraints report forces, this
    // function will be pure virtual.
    throw std::logic_error(
        "Constraints must implement this function. See #19435. ");
  }
  /* Clone() implementation. Derived classes must override to provide
   polymorphic deep-copy into a new instance. */
  virtual std::unique_ptr<SapConstraint<T>> DoClone() const = 0;

  /* ToDouble() implementation. Derived classes must override to provide
   polymorphic scalar conversion. */
  virtual std::unique_ptr<SapConstraint<double>> DoToDouble() const = 0;
  // @}

 private:
  SapConstraintJacobian<T> J_;
  std::vector<int> objects_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
