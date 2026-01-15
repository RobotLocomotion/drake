#pragma once

#include <span>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/block_sparse_lower_triangular_or_symmetric_matrix.h"
#include "drake/multibody/contact_solvers/icf/eigen_pool.h"
#include "drake/multibody/contact_solvers/icf/icf_data.h"
#include "drake/multibody/contact_solvers/icf/patch_constraints_data_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

// Forward declaration to break circular dependencies.
template <typename T>
class IcfModel;

/* A pool of contact constraints organized by patches. Each patch involves two
bodies A and B, with one or more contact pairs per patch. We can think of each
patch as corresponding to a physical "contact surface" between the two bodies.

Each constraint (patch) adds a cost term ℓ(v) to the overall ICF cost. The
gradient of this cost term is ∇ℓ = -Jᵀγ, where J = J_AbB is the Jacobian for the
relative spatial velocity of body B with respect to the spatial velocity of body
A shifted to B's center. That is, V_AbB_W = J⋅v, with v the generalized
velocities of the model. With this Jacobian definition, impulses γ correspond to
the spatial impulse on body B, expressed in the world frame.

This constraint type includes several optimizations for speeding up contact-rich
simulations:

  - All spatial quantities (relative contact velocities, normals, tangents,
    spatial velocities, etc.) are expressed in the world frame. This allows us
    to avoid expensive intermediate frame computations and reduce FLOP counts.

  - Computations are optimized for the common case of a single clique (a moving
    body interacts with an anchored body). The two-clique case (two moving
    bodies) is handled by shifting single-body quantities, allowing reuse of
    expensive-to-compute spatial quantities.

  - Contact pairs between the same two bodies are grouped into a single patch
    constraint. This improves memory locality by allowing the solver to process
    an entire patch with a coherent access pattern, reducing cache misses and
    allowing us to load per-body-pair data only once.

@tparam_nonsymbolic_scalar */
template <typename T>
class PatchConstraintsPool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PatchConstraintsPool);

  using JacobianView = typename EigenPool<Matrix6X<T>>::MatrixView;
  using ConstJacobianView = typename EigenPool<Matrix6X<T>>::ConstMatrixView;

  /* Constructs an empty pool. */
  explicit PatchConstraintsPool(const IcfModel<T>* parent_model);

  ~PatchConstraintsPool();

  /* Returns a reference to the parent model. */
  const IcfModel<T>& model() const { return *model_; }

  /* Returns the number of patches (pairs of contacting bodies) in the pool. */
  int num_patches() const { return ssize(num_pairs_); }

  /* Returns the number of pairs in the given patch. */
  int num_pairs(int patch_index) const {
    DRAKE_ASSERT(0 <= patch_index && patch_index < num_patches());
    return num_pairs_[patch_index];
  }

  /* Returns the number of pairs in each patch. */
  std::span<const int> patch_sizes() const {
    return std::span<const int>(num_pairs_);
  }

  /* Returns the total number of pairs across all patches. */
  int total_num_pairs() const { return ssize(fe0_); }

  /* Returns the number of constraints in the pool. We introduce one constraint
  for each patch. */
  int num_constraints() const { return num_patches(); }

  /* Sets the stiction tolerance used for friction regularization.

  This is a velocity, in m/s. Objects in stiction may slide tangentially at
  velocities up to this value. Additionally, we transition smoothly from static
  to dynamic friction coefficients at around 10× this velocity. See
  [Kurtz and Castro, 2025] for details. */
  void set_stiction_tolerance(double stiction_tolerance) {
    DRAKE_DEMAND(stiction_tolerance > 0.0);
    stiction_tolerance_ = stiction_tolerance;
  }

  /* Resizes this constraint pool to store the given patches and pairs.

  @param num_pairs_per_patch Number of contact pairs for each patch.

  @warning After resizing, constraints may hold invalid data until SetPatch()
  and SetPair() are called for each patch and pair. */
  void Resize(std::span<const int> num_pairs_per_patch);

  /* Sets the contact patch between bodies A and B.

  @param patch_index The index of the patch within the pool.
  @param bodyA The index of body A (may be anchored).
  @param bodyB The index of body B (must be dynamic).
  @param dissipation The Hunt & Crossley dissipation coefficient for the patch.
  @param static_friction The static friction coefficient for the patch.
  @param dynamic_friction The dynamic friction coefficient for the patch.
  @param p_AB_W The position of body B relative to body A.

  @pre B is always dynamic (not anchored). */
  void SetPatch(int patch_index, int bodyA, int bodyB, const T& dissipation,
                const T& static_friction, const T& dynamic_friction,
                const Vector3<T>& p_AB_W);

  /* Sets the contact pair data for the given patch and pair index.

  @param patch_index The index of the patch within the pool.
  @param pair_index The index of the contact pair within the patch.
  @param p_BoC_W The position of the contact point C relative to body B.
  @param normal_W Contact normal, from A into B by convention.
  @param fn0 The previous-step normal contact force (lagged for convexity.)
  @param stiffness Contact stiffness, in N/m. See [Masterjohn et al., 2022].

  @note As required by SetPatch(), body B is always dynamic (not anchored).
        Therefore we provide the position of the contact point relative to B
        and, if needed, the position relative to A is inferred from p_AB_W
        provided to SetPatch(). */
  void SetPair(const int patch_index, const int pair_index,
               const Vector3<T>& p_BoC_W, const Vector3<T>& normal_W,
               const T& fn0, const T& stiffness);

  /* Computes the sparsity pattern for the pool. That is, cliques i is connected
  to clique j > i iff sparsity[i] contains j. */
  void CalcSparsityPattern(std::vector<std::vector<int>>* sparsity) const;

  /* Computes constraint data for each patch, given the body spatial velocities
  V_WB. */
  void CalcData(const EigenPool<Vector6<T>>& V_WB,
                PatchConstraintsDataPool<T>* patch_data) const;

  /* Adds the gradient contribution of the patch constraints, ∇ℓ = −Jᵀ⋅γ, to the
  overall gradient. */
  void AccumulateGradient(const IcfData<T>& data, VectorX<T>* gradient) const;

  /* Adds the Hessian contribution of the patch constraints to the overall
  Hessian. */
  void AccumulateHessian(
      const IcfData<T>& data,
      contact_solvers::internal::BlockSparseSymmetricMatrix<MatrixX<T>>*
          hessian) const;

  /* Computes the first and second derivatives of the constraint cost
  ℓ̃(α) = ℓ(v + α⋅w).

  @param patch_data The pre-computed patch constraint data at v + α⋅w.
  @param U_WB Body spatial velocity when the generalized velocities equal w.
         I.e., U_WB = J⋅w.
  @param U_AbB_W_pool Scratch space for patch spatial velocities.
  @param[out] dcost the first derivative dℓ̃ /dα on output.
  @param[out] d2cost the second derivative d²ℓ̃ /dα² on output. */
  void CalcCostAlongLine(const PatchConstraintsDataPool<T>& patch_data,
                         const EigenPool<Vector6<T>>& U_WB_pool,
                         EigenPool<Vector6<T>>* U_AbB_W_pool, T* dcost,
                         T* d2cost) const;

  /* Testing only access. */
  const std::vector<std::pair<int, int>>& bodies() { return bodies_; }

 private:
  /* Returns the index into data per patch and per contact pair.
  @param p Patch index p < num_pairs().
  @param k Pair index k < num_pairs(p). */
  int patch_pair_index(int p, int k) const { return pair_data_start_[p] + k; }

  /* Computes cost, gradient, and Hessian contributions for the given pair.

  @param p Patch index.
  @param k Pair index within patch.
  @param v_AcBc_W Relative contact point velocity.
  @param[out] gamma_Bc_W Contact spatial impulse on body B
  @param[out] G Contact spatial Hessian for the pair. */
  T CalcLaggedHuntCrossleyModel(int p, int k, const Vector3<T>& v_AcBc_W,
                                Vector3<T>* gamma_Bc_W, Matrix3<T>* G) const;

  /* Computes relative spatial velocities V_AbB_W for each patch, given body
  spatial velocities V_WB. When A is anchored, V_WA = 0 and V_AbB_W = V_WB. */
  void CalcConstraintVelocities(const EigenPool<Vector6<T>>& V_WB_pool,
                                EigenPool<Vector6<T>>* V_AbB_W_pool) const;

  /* Computes contact velocities for each contact pair from body spatial
  velocities V_WB. */
  void CalcContactVelocities(const EigenPool<Vector6<T>>& V_WB_pool,
                             EigenPool<Vector3<T>>* v_AcBc_W_pool) const;

  /* Computes cost, gradient, and Hessian contributions for each patch. */
  void CalcPatchQuantities(const EigenPool<Vector3<T>>& v_AcBc_W_pool,
                           std::vector<T>* cost_pool,
                           EigenPool<Vector6<T>>* spatial_impulses_pool,
                           EigenPool<Matrix6<T>>* patch_hessians_pool) const;

  // The parent model.
  const IcfModel<T>* const model_;

  // The stiction tolerance vₛ (m/s) for regularized friction.
  double stiction_tolerance_{1.0e-4};

  // SAP regularization parameter σ. Each frictional contact is regularized by
  // εₛ = max(vₛ, μ⋅σ⋅wₜ⋅n₀), where vₛ is the stiction tolerance, μ is the
  // friction coefficient, wₜ is the Delassus operator approximation, and n₀ is
  // the normal impulse from the previous time step. See the ICF paper [Castro
  // et al., 2023] for further details.
  double sigma_{1.0e-3};

  // Data per patch. Indexed by patch index p < num_patches().
  std::vector<int> num_pairs_;    // Number of pairs per patch.
  std::vector<int> num_cliques_;  // Number of cliques (one or two).
  std::vector<T> Rt_;             // Friction regularization Rₜ = σ⋅wₜ

  // Bodies involved in each patch. The first is always body B, which always
  // corresponds to a valid (positive) clique. The second is always body A,
  // which might correspond to an anchored body with invalid (negative) clique.
  std::vector<std::pair<int, int>> bodies_;

  EigenPool<Vector3<T>> p_AB_W_;  // Position of body B relative to A.
  std::vector<T> dissipation_;    // Hunt & Crossley dissipation.

  // Friction coefficients per patch.
  std::vector<T> static_friction_;
  std::vector<T> dynamic_friction_;

  // Starting index for each patch. Holds the global pair index of the first
  // pair in each patch, of size num_patches().
  std::vector<int> pair_data_start_;

  // Data per pair. Indexed by patch_pair_index(p, k).
  EigenPool<Vector3<T>> p_BC_W_;    // Position of contact point from body B.
  EigenPool<Vector3<T>> normal_W_;  // Contact normals.
  std::vector<T> stiffness_;        // Linear stiffness, N/m.
  // Elastic component of the normal force at the previous step, in N.
  std::vector<T> fe0_;
  // Normal force (elastic and damping) at the previous step, in N.
  std::vector<T> fn0_;

  // The net friction coefficient for each pair. Computed by interpolating
  // between static and dynamic coefficients based on the relative contact
  // velocity of this pair at the previous time step.
  std::vector<T> net_friction_;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::contact_solvers::icf::internal::
        PatchConstraintsPool);
