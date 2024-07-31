#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/sap_holonomic_constraint.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* Struct to store the kinematics of the fixed constraint in its current
 configuration, when it gets constructed. */
template <typename T>
struct FixedConstraintKinematics {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FixedConstraintKinematics);

  /* Constructor for fixed constraints where only a single body has degrees of
     freedom.
     @param[in] objectA
                Index of the physical object A on which point P attaches.
                Must be non-negative.
     @param[in] p_APs_W
                Positions of point Ps in A, expressed in the world frame.
     @param[in] p_PQs_W
                Displacements from point Ps to point Qs, expressed in the
                world frame.
     @param[in] J_ApBq_W
                Jacobian for the relative velocity v_ApBq_W.
     @pre object_A >= 0
     @pre p_APs_W_in.size() == p_PQs_W_in.size()
     @pre p_APs_W_in.size() == J_ApBq_W.rows()
     @pre p_APs_W_in.size() % 3 == 0 */
  FixedConstraintKinematics(int objectA_in, VectorX<T> p_APs_W_in,
                            VectorX<T> p_PQs_W_in,
                            SapConstraintJacobian<T> J_ApBq_W)
      : objectA(objectA_in),
        p_APs_W(std::move(p_APs_W_in)),
        p_PQs_W(std::move(p_PQs_W_in)),
        J(std::move(J_ApBq_W)) {
    DRAKE_THROW_UNLESS(objectA >= 0);
    const int num_constrained_dofs = p_APs_W.size();
    DRAKE_THROW_UNLESS(num_constrained_dofs % 3 == 0);
    DRAKE_THROW_UNLESS(p_PQs_W.size() == num_constrained_dofs);
    DRAKE_THROW_UNLESS(J.rows() == num_constrained_dofs);
  }

  /* Constructor for fixed constraints where both bodies have degrees of
     freedom.
     @param[in] objectA
                Index of the physical object A on which point P attaches.
                Must be non-negative.
     @param[in] p_APs_W
                Positions of point Ps in A, expressed in the world frame.
     @param[in] objectB
                Index of the physical object B on which point Q attaches.
                Must be non-negative.
     @param[in] p_BQs_W
                Positions of point Qs in B, expressed in the world frame.
     @param[in] p_PQs_W
                Displacements from point Ps to point Qs, expressed in the
                world frame.
     @param[in] J_ApBq_W
                Jacobian for the relative velocity v_ApBq_W.
     @pre object_A >= 0 && object_B >= 0
     @pre p_APs_W_in.size() == p_BQs_W_in.size()
     @pre p_APs_W_in.size() == p_PQs_W_in.size()
     @pre p_APs_W_in.size() == J_ApBq_W.rows()
     @pre p_APs_W_in.size() % 3 == 0 */
  FixedConstraintKinematics(int objectA_in, VectorX<T> p_APs_W_in,
                            int objectB_in, VectorX<T> p_BQs_W_in,
                            VectorX<T> p_PQs_W_in,
                            SapConstraintJacobian<T> J_ApBq_W)
      : objectA(objectA_in),
        p_APs_W(std::move(p_APs_W_in)),
        objectB(objectB_in),
        p_BQs_W(std::move(p_BQs_W_in)),
        p_PQs_W(std::move(p_PQs_W_in)),
        J(std::move(J_ApBq_W)) {
    DRAKE_THROW_UNLESS(objectA >= 0);
    const int num_constrained_dofs = p_APs_W.size();
    DRAKE_THROW_UNLESS(num_constrained_dofs % 3 == 0);
    DRAKE_THROW_UNLESS(objectB >= 0);
    DRAKE_THROW_UNLESS(p_BQs_W->size() == num_constrained_dofs);
    DRAKE_THROW_UNLESS(p_PQs_W.size() == num_constrained_dofs);
    DRAKE_THROW_UNLESS(J.rows() == num_constrained_dofs);
  }

  int objectA{};
  VectorX<T> p_APs_W;
  std::optional<int> objectB;
  std::optional<VectorX<T>> p_BQs_W;
  VectorX<T> p_PQs_W;
  SapConstraintJacobian<T> J;
};

/* Implements a SAP fixed constraint between two objects.

 Given two objects A and B with at least one of which being deformable, consider
 n constrained point pairs: point Pᵢ on object A (assumed to be deformable) and
 point Qᵢ on object B, for i = 0, 1, ..., n-1. Working in the world frame W,
 this constraint penalizes non-coincident Pᵢ and Qᵢ, for each
 i = 0, 1, ..., n-1, with the constraint functions:

   gᵢ = p_WQᵢ - p_WPᵢ = 0

 with corresponding constraint velocity:

   ġᵢ = vcᵢ(v) = v_W_PᵢQᵢ

 N.B. Only when Pᵢ and Qᵢ are coincident is their relative velocity independent
 of which frame it is measured in. This means that the way we model this
 constraint lacks frame-invariance does not conserve angular momentum! It is
 unclear at this time if choosing a particular frame leads to a better
 approximation, conditioning, etc. We choose the world frame for convenience.

 This leads to 3n holonomic constraint equations. The i-th
 constrained point pair produces constraint impulse γᵢ ∈ ℝ³. The i-th impulse on
 B applied at Qᵢ is:

   γ_BQᵢ_W = γᵢ

 and the reaction force on A applied at P is:

   γ_APᵢ_W = -γᵢ

 DoAccumulateSpatialImpulses() shifts these impulses to the body frame origins,
 reporting the total spatial impulses Γ_Bo_W and Γ_Ao_W from all n point pairs,
 respectively.

 @tparam_nonsymbolic_scalar */
template <typename T>
class SapFixedConstraint final : public SapHolonomicConstraint<T> {
 public:
  /* We do not allow copy, move, or assignment generally to avoid slicing. */
  //@{
  SapFixedConstraint& operator=(const SapFixedConstraint&) = delete;
  SapFixedConstraint(SapFixedConstraint&&) = delete;
  SapFixedConstraint& operator=(SapFixedConstraint&&) = delete;
  //@}

  /* Constructs a fixed constraint given its kinematics in a particular
   configuration. */
  explicit SapFixedConstraint(FixedConstraintKinematics<T> kinematics);

  int num_constrained_point_pairs() const {
    return num_constrained_point_pairs_;
  }

 private:
  /* Private copy construction is enabled to use in the implementation of
   DoClone(). */
  SapFixedConstraint(const SapFixedConstraint&) = default;

  /* no-op for this constraint. */
  void DoAccumulateGeneralizedImpulses(int, const Eigen::Ref<const VectorX<T>>&,
                                       EigenPtr<VectorX<T>>) const final {}

  /* Accumulates spatial forces applied by this constraint on the i-th object.
   @param[in] i        Object index. As defined at construction i = 0
                       corresponds to object A and i = 1 corresponds to object
                       B.
   @param[in] gamma A  vector of size equation to the number of constraint
                       equations.
   @param[out] F       On output, this method accumulates the total spatial
                       impulse applied by this constraint on the i-th object.
   @pre 0 ≤ i < 2.
   @pre gamma is a vector of size equal to the number of constraint equations.
   @pre F is not nullptr. */
  void DoAccumulateSpatialImpulses(int i,
                                   const Eigen::Ref<const VectorX<T>>& gamma,
                                   SpatialForce<T>* F) const final;

  /* Helper used at construction. This method makes the parameters needed by the
   base class SapHolonomicConstraint. */
  static typename SapHolonomicConstraint<T>::Parameters
  MakeSapHolonomicConstraintParameters(int num_constraint_equations);

  /* Helper used at construction. This method makes the object indices needed by
   the base class SapHolonomicConstraint. */
  static std::vector<int> MakeObjectIndices(
      const FixedConstraintKinematics<T>& kinematics);

  std::unique_ptr<SapConstraint<T>> DoClone() const final {
    return std::unique_ptr<SapFixedConstraint<T>>(
        new SapFixedConstraint<T>(*this));
  }

  // We do not yet support scalar conversion for constraints used for
  // deformables.
  std::unique_ptr<SapConstraint<double>> DoToDouble() const final {
    throw std::runtime_error(
        "SapFixedConstraint: Scalar conversion to double not supported.");
  }

  int num_constrained_point_pairs_{};
  VectorX<T> p_APs_W_;
  std::optional<VectorX<T>> p_BQs_W_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
