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

using internal::BlockSparseSymmetricMatrixT;

namespace icf {
namespace internal {

// Forward declaration to break circular dependencies.
template <typename T>
class IcfModel;

/* A pool of contact constraints organized by patches. Each patch involves two
bodies and one (point contact) or more (hydroelastic) contact pairs. */
template <typename T>
class PatchConstraintsPool {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PatchConstraintsPool);

  using JacobianView = Eigen::Map<Matrix6X<T>>;
  using ConstJacobianView = typename EigenPool<Matrix6X<T>>::ConstMatrixView;

  /* Constructs an empty pool. */
  explicit PatchConstraintsPool(const IcfModel<T>* parent_model)
      : model_(parent_model) {
    DRAKE_ASSERT(parent_model != nullptr);
  }

  /* Clears the constraints without freeing memory (capacity remains the same).
  After this call num_patches() and num_pairs() are zero. */
  void Clear();

  /* Resizes to store the given patches and pairs.

  @param num_pairs_per_patch Number of contact pairs for each patch.*/
  void Resize(std::span<const int> num_pairs_per_patch);

  /* Sets the contact patch between bodies A and B.

  @param patch_index The index of the patch within the pool.
  @param bodyA The index of body A (may be anchored).
  @param bodyB The index of body B (must be dynamic).
  @param dissipation The dissipation coefficient for the patch.
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
  @param stiffness The contact stiffness for the pair.

  @note As required by SetPatch(), body B is always dynamic (not anchored).
        Therefore we provide the position of the contact point relative to B
        and, if needed, the position relative to A is inferred from p_AB_W
        provided to SetPatch(). */
  void SetPair(const int patch_index, const int pair_index,
               const Vector3<T>& p_BoC_W, const Vector3<T>& normal_W,
               const T& fn0, const T& stiffness);

  /* Returns the number of patches (pairs of contacting bodies) in the pool. */
  int num_patches() const { return ssize(num_pairs_); }

  /* Returns the number of constraints in the pool. We introduce one constraint
   * for each patch. */
  int num_constraints() const { return num_patches(); }

  /* Returns a reference to the parent model. */
  const IcfModel<T>& model() const { return *model_; }

  /* Returns the number of pairs in each patch. */
  std::span<const int> patch_sizes() const {
    return std::span<const int>(num_pairs_);
  }

  /* Returns the total number of pairs across all patches. */
  int total_num_pairs() const { return ssize(fn0_); }

  /* Returns the number of pairs in the given patch. */
  int num_pairs(int patch_index) const {
    DRAKE_ASSERT(0 <= patch_index && patch_index < num_patches());
    return num_pairs_[patch_index];
  }

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
  void AccumulateHessian(const IcfData<T>& data,
                         BlockSparseSymmetricMatrixT<T>* hessian) const;

  /* Computes the first and second derivatives of ℓ(α) = ℓ(v + αw) at α = 0.
  Used for exact line search.

  @param patch_data The pre-computed patch constraint data.
  @param U_WB Body spatial velocities associated with u = v + αw.
  @param U_AbB_W_pool Scratch space for body spatial velocities.
  @param dcost The first derivative, dℓ/dα at α = 0.
  @param d2cost The second derivative, d²ℓ/dα² at α = 0. */
  void ProjectAlongLine(const PatchConstraintsDataPool<T>& patch_data,
                        const EigenPool<Vector6<T>>& U_WB_pool,
                        EigenPool<Vector6<T>>* U_AbB_W_pool, T* dcost,
                        T* d2cost) const;

  /* Sets the stiction tolerance used for friction regularization. */
  void set_stiction_tolerance(double stiction_tolerance) {
    DRAKE_DEMAND(stiction_tolerance > 0.0);
    stiction_tolerance_ = stiction_tolerance;
  }

 private:
  using VectorXView = Eigen::VectorBlock<VectorX<T>>;

  double stiction_tolerance_{1.0e-4};
  double sigma_{1.0e-3};

  /* Computes cost, gradient, and Hessian contributions for the given pair.

  @param p Patch index.
  @param k Pair index within patch.
  @param v_AcBc_W Relative contact point velocity.
  @param[out] gamma_Bc_W Contact spatial impulse on body B
  @param[out] G Contact spatial Hessian for the pair. */
  T CalcLaggedHuntCrossleyModel(int p, int k, const Vector3<T>& v_AcBc_W,
                                Vector3<T>* gamma_Bc_W, Matrix3<T>* G) const;

  /* Computes cost, gradient, and Hessian contributions for each patch. */
  void CalcPatchQuantities(const EigenPool<Vector3<T>>& v_AcBc_W_pool,
                           std::vector<T>* cost_pool,
                           EigenPool<Vector6<T>>* spatial_impulses_pool,
                           EigenPool<Matrix6<T>>* patch_hessians_pool) const;

  /* Computes contact velocities for each contact pair from body spatial
  velocities V_WB. */
  void CalcContactVelocities(const EigenPool<Vector6<T>>& V_WB_pool,
                             EigenPool<Vector3<T>>* v_AcBc_W_pool) const;

  /* Computes relative spatial velocities V_AbB_W for each patch, given body
  spatial velocities V_WB. When A is anchored, V_WA = 0 and V_AbB_W = V_WB. */
  void CalcConstraintVelocities(const EigenPool<Vector6<T>>& V_WB_pool,
                                EigenPool<Vector6<T>>* V_AbB_W_pool) const;

  /* Returns the index into data per patch and per contact pair.
  @param p Patch index p < num_pairs().
  @param k Pair index k < num_pairs(p). */
  int patch_pair_index(int p, int k) const { return pair_data_start_[p] + k; }

  const IcfModel<T>* model_{nullptr};  // The parent model.

  // Data per patch. Indexed by patch index p < num_patches()
  std::vector<int> num_pairs_;    // Number of pairs per patch.
  std::vector<int> num_cliques_;  // Num cliques. One or two.
  std::vector<T> Rt_;             // Friction regularization

  // .first is always body B, which always corresponds to a valid (positive)
  // clique.
  // .second is always body A, which might corresponds to an anchored body with
  // invalid (negative) clique.
  std::vector<std::pair<int, int>> bodies_;
  EigenPool<Vector3<T>> p_AB_W_;    // Position of body B relative to A.
  std::vector<T> dissipation_;      // Hunt & Crossley dissipation.
  std::vector<T> static_friction_;  // Friction coefficients
  std::vector<T> dynamic_friction_;

  // Data per patch and per pair. Indexed by patch_pair_index(p, k).
  std::vector<int> pair_data_start_;  // Starting index for each patch.
  EigenPool<Vector3<T>> p_BC_W_;      // Position of contact point from body A.
  EigenPool<Vector3<T>> normal_W_;    // Contact normals.
  std::vector<T> stiffness_;          // Linear stiffness, N/m.
  std::vector<T> fn0_;                // Previous time step normal force.
  std::vector<T> n0_;                 // Previous time step impulse.
  std::vector<T> net_friction_;       // Friction coefficients mu(s)
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::contact_solvers::icf::internal::
        PatchConstraintsPool);
