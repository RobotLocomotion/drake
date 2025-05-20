#pragma once

#ifndef DRAKE_POOLED_SAP_INCLUDED
#error Do not include this file. Use "drake/multibody/contact_solvers/pooled_sap/pooled_sap.h" // NOLINT
#endif

#include <algorithm>
#include <numeric>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/pooled_sap/eigen_pool.h"
#include "drake/multibody/contact_solvers/pooled_sap/patch_constraints_data_pool.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap.h"
#include "drake/multibody/contact_solvers/pooled_sap/sap_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

/* A pool of contact constraints organized by patches. Each patch involves two
bodies. */
template <typename T>
class PooledSapModel<T>::PatchConstraintsPool {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PatchConstraintsPool);

  using JacobianView = Eigen::Map<Matrix6X<T>>;

  int num_constraints() const { return ssize(num_pairs_); }

  int num_constraint_equations() const { return 3 * total_num_pairs(); }

  /* Constructor for an empty pool. */
  PatchConstraintsPool(const PooledSapModel<T>* parent_model)
      : model_(parent_model) {
    DRAKE_ASSERT(parent_model != nullptr);
  }

  /* Returns reference to the parent model. */
  const PooledSapModel<T>& model() const { return *model_; }

  // Reset to work in a model with the given cliques.
  void Reset(const T& time_step, const std::vector<int>& clique_start,
             const std::vector<int>& clique_size) {
    time_step_ = time_step;
    clique_start_ = clique_start;
    clique_size_ = clique_size;
    Clear();
  }

  /* Resizes to store patch constraint data. No memory allocation performed if
     the current capcity is enough to store this data size.
   @param num_patches The number of patches.
   @param num_pairs_capcity Capacity for the total number of pairs.
   @param max_clique_size Used to estimate storage for spatial velocity
     Jacobians.  */
  void Resize(int num_patches, int num_pairs_capacity, int max_clique_size) {
    unused(max_clique_size);
    // per-patch data.
    num_pairs_.resize(num_patches);
    num_cliques_.resize(num_patches);
    bodies_.resize(num_patches);
    p_AB_W_.Resize(num_patches);
    dissipation_.resize(num_patches);
    friction_.resize(num_patches);

    // per-pair data.
    normal_W_.Resize(num_pairs_capacity);
    p_BC_W_.Resize(num_pairs_capacity);
    stiffness_.resize(num_pairs_capacity);
    fn0_.resize(num_pairs_capacity);
    n0_.resize(num_pairs_capacity);
    epsilon_soft_.resize(num_pairs_capacity);
  }

  /* Reserve to store patch constraint data. No memory allocation performed if
     the current capcity is enough to store this data size.
   @param num_patches The number of patches.
   @param num_pairs_capcity Capacity for the total number of pairs.
   @param max_clique_size Used to estimate storage for spatial velocity
     Jacobians.  */
  void Reserve(int num_patches, int num_pairs_capacity, int max_clique_size) {
    unused(max_clique_size);

    // Data per patch.
    num_pairs_.reserve(num_patches);
    num_cliques_.reserve(num_patches);
    bodies_.reserve(num_patches);
    p_AB_W_.Reserve(num_patches);
    dissipation_.reserve(num_patches);
    friction_.reserve(num_patches);

    // Data per patch and per pair.
    normal_W_.Reserve(num_pairs_capacity);
    p_BC_W_.Reserve(num_pairs_capacity);
    stiffness_.reserve(num_pairs_capacity);
    fn0_.reserve(num_pairs_capacity);
    n0_.reserve(num_pairs_capacity);
    epsilon_soft_.reserve(num_pairs_capacity);
  }

  /* Adds a contact patch between bodies A and B.
   @pre B is always dynamic (not anchored).
   @returns the index into the new patch. */
  int AddPatch(int bodyA, int bodyB, const T& dissipation, const T& friction,
               const Vector3<T>& p_AB_W) {
    DRAKE_DEMAND(bodyA != bodyB);               // Same body never makes sense.
    DRAKE_DEMAND(!model().is_anchored(bodyB));  // B is never anchored.
    const int index = num_patches();

    bodies_.emplace_back(bodyB, bodyA);  // Dynamic body B always first.
    dissipation_.push_back(dissipation);
    friction_.push_back(friction);
    p_AB_W_.PushBack(p_AB_W);

    const int num_cliques =
        (model().is_anchored(bodyA) || model().is_anchored(bodyB)) ? 1 : 2;
    num_cliques_.push_back(num_cliques);

    // Start indexes.
    if (index == 0) {  // the very first patch.
      pair_data_start_.push_back(0);
    } else {
      pair_data_start_.push_back(pair_data_start_.back() + num_pairs_.back());
    }
    num_pairs_.push_back(0);

    return index;
  }

  /* Adds per-pair data associated with the patch last added with AddPatch(),
   that is, the patch with index equal to num_patches()-1.

   @note As required by AddPatch(), body B is always dynamics (not anchored).
   Therefore we provide the position of the contact point relative to B and, if
   needed, the position relative to A is inferred from p_AB_W provided to
   AddPatch().

   @param[in] normal_W Contact normal, from A into B by convention. */
  void AddPair(const Vector3<T>& p_BoC_W, const Vector3<T>& normal_W,
               const T& fn0, const T& stiffness) {
    const int p = num_patches() - 1;
    ++num_pairs_[p];

    p_BC_W_.PushBack(p_BoC_W);
    normal_W_.PushBack(normal_W);
    fn0_.push_back(fn0);
    stiffness_.push_back(stiffness);

    // Pre-computed quantities.
    const int num_cliques = num_cliques_[p];

    // First clique.
    const Vector6<T>& V_WB = model().V_WB(bodies_[p].first);
    const auto w_WB = V_WB.template head<3>();
    const auto v_WB = V_WB.template tail<3>();
    Vector3<T> v_AcBc_W = v_WB + w_WB.cross(p_BoC_W);

    // Second clique.
    if (num_cliques == 2) {
      const Vector6<T>& V_WA = model().V_WB(bodies_[p].second);
      const Vector3<T> p_AC_W = p_AB_W_[p] + p_BoC_W;
      const auto w_WA = V_WA.template head<3>();
      const auto v_WA = V_WA.template tail<3>();
      v_AcBc_W -= (v_WA + w_WA.cross(p_AC_W));
    }

    using std::max;
    const T& d = dissipation_[p];
    const T vn0 = v_AcBc_W.dot(normal_W);
    const T damping = max(0.0, 1.0 - d * vn0);
    const T n0 = max(0.0, time_step_ * fn0) * damping;
    n0_.push_back(n0);

#if 0
    const T& mu = friction_[p];
    const T Rt = CalcRegularizationOfFriction(p, p_BoC_W);
    const T sap_stiction_tolerance = mu * Rt * n0;
    const T eps = max(stiction_tolerance_, sap_stiction_tolerance);

    fmt::print("vs: {}. vs_sap: {}. eps: {}\n", stiction_tolerance_,
               sap_stiction_tolerance, eps);

    epsilon_soft_.push_back(eps);
#endif
    epsilon_soft_.push_back(stiction_tolerance_);
  }

  /* Clears memory, no memory is freed, and the capacity remains the same.
   After this call num_patches() and num_pairs() equal zero. */
  void Clear() {
    // Data per patch.
    num_pairs_.clear();
    num_cliques_.clear();
    bodies_.clear();
    p_AB_W_.Clear();
    dissipation_.clear();
    friction_.clear();
    pair_data_start_.clear();

    // Data per patch and per pair.
    p_BC_W_.Clear();
    normal_W_.Clear();
    stiffness_.clear();
    fn0_.clear();
    n0_.clear();
    epsilon_soft_.clear();
  }

  int num_patches() const { return ssize(num_pairs_); }

  /* Returns the sizes of all patches in the pool. */
  const std::vector<int>& patch_sizes() const { return num_pairs_; }

  /* Total number of pairs across all patches. */
  int total_num_pairs() const { return ssize(fn0_); }

  int num_pairs(int patch_index) const {
    DRAKE_ASSERT(0 <= patch_index && patch_index < num_patches());
    return num_pairs_[patch_index];
  }

  void CalcData(const SapData<T>& data,
                PatchConstraintsDataPool<T>* patch_data) const;

  // TODO(amcastro-tri): factor out this method into a BodyConstraintsPool
  // parent class, along with other common functionality to all body-constraint
  // pools.
  void AccumulateGradientAndHessian(const SapData<T>& data,
                                    VectorX<T>* gradient,
                                    MatrixX<T>* hessian) const;

 private:
  using ConstJacobianView =
      typename PooledSapModel<T>::ConstSpatialVelocityJacobianView;
  using ConstVectorXView = Eigen::VectorBlock<const VectorX<T>>;
  using VectorXView = Eigen::VectorBlock<VectorX<T>>;

  ConstVectorXView clique_segment(int clique, const VectorX<T>& x) const {
    return x.segment(clique_start_[clique], clique_size_[clique]);
  }

  VectorXView clique_segment(int clique, VectorX<T>* x) const {
    DRAKE_ASSERT(x != nullptr);
    return x->segment(clique_start_[clique], clique_size_[clique]);
  }

  double stiction_tolerance_{1.0e-4};
  double vs2_{stiction_tolerance_ * stiction_tolerance_};
  double sigma_{1.0e-3};

  /* Computes Rt. */
  T CalcRegularizationOfFriction(int p, const Vector3<T>& p_BoC_W) const;

  T CalcLaggedHuntCrossleyModel(int p, int k, const Vector3<T>& v_AcBc_W,
                                Vector3<T>* gamma_Bc_W, Matrix3<T>* G) const;
  void CalcContactVelocities(const EigenPool<Vector6<T>>& V_WB_pool,
                             EigenPool<Vector3<T>>* v_AcBc_W_pool) const;
  void CalcPatchQuantities(const EigenPool<Vector3<T>>& v_AcBc_W_pool,
                           std::vector<T>* cost_pool,
                           EigenPool<Vector6<T>>* spatial_impulses_pool,
                           EigenPool<Matrix6<T>>* patch_hessians_pool) const;

  const PooledSapModel<T>* model_{nullptr};  // The parent model.

  T time_step_{0.0};

  /* Per-clique data. Indexed by c < num_cliques()  */
  std::vector<int> clique_start_;  // Velocity start index for the c-th clique.
  std::vector<int> clique_size_;   // Num. velocities for the c-th clique.

  /* Data per patch. Indexed by patch index p < num_patches() */
  std::vector<int> num_pairs_;    // Number of pairs per patch.
  std::vector<int> num_cliques_;  // Num cliques. One or two.

  // .first is always body B, which always corresponds to a valid (positive)
  // clique.
  // .second is always body A, which might corresponds to an anchored body with
  // invalid (negative) clique.
  std::vector<std::pair<int, int>> bodies_;
  EigenPool<Vector3<T>> p_AB_W_;  // Position of body B relative to A.
  std::vector<T> dissipation_;    // Hunt & Crossley dissipation.
  std::vector<T> friction_;       // Friction coefficient.

  /* Data per patch and per pair. Indexed by patch_pair_index(p, k). */
  std::vector<int> pair_data_start_;  // Start into arrays indexed by
                                      // patch_pair_index(p, k).
  /* Returns index into data per patch and per contact pair.
   @param p Patch index p < num_pairs().
   @param k Pair index k < num_pairs(p). */
  int patch_pair_index(int p, int k) const { return pair_data_start_[p] + k; }

  EigenPool<Vector3<T>> p_BC_W_;    // Position of contact point from body A.
  EigenPool<Vector3<T>> normal_W_;  // Contact normals/
  std::vector<T> stiffness_;        // Linear stiffness, N/m.
  std::vector<T> fn0_;              // Previous time step normal force.
  std::vector<T> n0_;               // Previous time step impulse.
  std::vector<T> epsilon_soft_;     // Regularized stiction tolerance.

  // Scratch used during construction to compute Delassus approximation.
  mutable EigenPool<MatrixX<T>> MatrixX_pool_;
};

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

// DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
//     class ::drake::multibody::contact_solvers::pooled_sap::
//         PatchConstraintsPool);
