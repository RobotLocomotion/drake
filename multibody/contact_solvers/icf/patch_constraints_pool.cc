#include "drake/multibody/contact_solvers/icf/patch_constraints_pool.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>
#include <vector>

#include "drake/math/cross_product.h"
#include "drake/multibody/contact_solvers/icf/icf_model.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

using contact_solvers::internal::BlockSparseSymmetricMatrix;
using math::VectorToSkewSymmetric;

// Computes the soft norm ‖x‖ₛ = sqrt(xᵀx + ε²) - ε.
template <typename T>
T SoftNorm(const Vector3<T>& x, const T& eps) {
  using std::sqrt;
  return sqrt(x.squaredNorm() + eps * eps) - eps;
}

// Given spatial force F_Bo applied at B and the relative position p_AB of B
// from A, computes the spatial force F_Ao shifted to A. Mathematically, F_Ao =
// ϕ(p_AB)ᵀ⋅F_Bo, where ϕ(p) = [-pₓ; 𝕀₃]. All quantities must be expressed in
// the same common frame.
//
// @param F The spatial force F_Bo.
// @param p The relative position p_AB.
// @returns The shifted spatial force F_Ao.
template <typename T>
Vector6<T> ShiftSpatialForce(const Vector6<T>& F, const Vector3<T>& p) {
  const auto t = F.template head<3>();
  const auto f = F.template tail<3>();
  Vector6<T> result;
  result.template head<3>() = t + p.cross(f);
  result.template tail<3>() = f;
  return result;
}

// Shifts a second-order tensor G_Bo computed about B to tensor G_Ao computed
// about A, given the relative position p_AB of B from A. Mathematically, G_Ao =
// ϕ(p_AB)ᵀ⋅G_Bo⋅ϕ(p_AB), where ϕ(p) = [-pₓ; 𝕀₃]. All quantities must be
// expressed in the same common frame.
//
// @param G The second-order tensor G_Bo.
// @param p The relative position p_AB.
// @returns The shifted second-order tensor G_Ao.
template <typename T>
Matrix6<T> ShiftSecondOrderTensor(const Matrix3<T>& G, const Vector3<T>& p) {
  const Matrix3<T> px = VectorToSkewSymmetric(p);
  const Matrix3<T> pxG = px * G;
  Matrix6<T> Gp;
  Gp.template topLeftCorner<3, 3>() = -pxG * px;
  Gp.template topRightCorner<3, 3>() = pxG;
  Gp.template bottomLeftCorner<3, 3>() = pxG.transpose();
  Gp.template bottomRightCorner<3, 3>() = G;
  return Gp;
}

// Returns G⋅Φ(p), where Φ(p) = [𝕀₃, 0; -pₓ, 𝕀₃].
template <typename T>
Matrix6<T> ShiftFromTheRight(const Matrix6<T>& G, const Vector3<T>& p) {
  const auto Gw = G.template topLeftCorner<3, 3>();
  const auto Gwv = G.template topRightCorner<3, 3>();
  const auto Gvw = G.template bottomLeftCorner<3, 3>();
  const auto Gv = G.template bottomRightCorner<3, 3>();
  const Matrix3<T> px = VectorToSkewSymmetric(p);

  Matrix6<T> R;
  R.template topLeftCorner<3, 3>() = Gw - Gwv * px;
  R.template topRightCorner<3, 3>() = Gwv;
  R.template bottomLeftCorner<3, 3>() = Gvw - Gv * px;
  R.template bottomRightCorner<3, 3>() = Gv;

  return R;
}

// Returns Φ(p)ᵀ⋅G, where Φ(p) = [𝕀₃, 0; -pₓ, 𝕀₃]
template <typename T>
Matrix6<T> ShiftFromTheLeft(const Matrix6<T>& G, const Vector3<T>& p) {
  const auto Gw = G.template topLeftCorner<3, 3>();
  const auto Gwv = G.template topRightCorner<3, 3>();
  const auto Gvw = G.template bottomLeftCorner<3, 3>();
  const auto Gv = G.template bottomRightCorner<3, 3>();
  const Matrix3<T> px = VectorToSkewSymmetric(p);

  Matrix6<T> R;
  R.template topLeftCorner<3, 3>() = Gw + px * Gvw;
  R.template topRightCorner<3, 3>() = Gwv + px * Gv;
  R.template bottomLeftCorner<3, 3>() = Gvw;
  R.template bottomRightCorner<3, 3>() = Gv;

  return R;
}

/* Computes the normal impulse and derivative associated with an individual
contact using a discrete Hunt-Crossley model. The normal impulse is

  γₙ(vₙ) = δt⋅(k⋅(-ϕ₀ − δt⋅vₙ))₊⋅(1 - d⋅vₙ)₊
         = δt⋅(fₑ₀ - δt⋅k⋅vₙ)₊⋅(1 - d⋅vₙ)₊,

where (x)₊ = max(0, x), vₙ is the normal contact velocity, ϕ₀ is the initial
signed distance, fₑ₀ = -k⋅ϕ₀ is the previous-step elastic force contribution, k
is the contact stiffness, d is the dissipation coefficient, and δt is the time
step.

@param dt Time step δt.
@param vn Normal contact velocity vₙ.
@param fe0 Previous time step normal force elastic contribution fₑ₀.
@param k Contact stiffness.
@param d Dissipation coefficient.

@returns A pair with the normal impulse γₙ and its derivative dγₙ/dvₙ. */
template <typename T>
std::pair<T, T> CalcDiscreteHuntCrossleyImpulseAndDerivative(
    const T& dt, const T& vn, const T& fe0, const T& k, const T& d) {
  // Elastic force contribution.
  const T fe = fe0 - dt * k * vn;

  // Quick exit if either of the terms is non-positive.
  if (fe <= 0.0) {
    return std::make_pair(0.0, 0.0);
  }
  const T damping = 1.0 - d * vn;
  if (damping <= 0.0) {
    return std::make_pair(0.0, 0.0);
  }

  // Normal impulse γₙ.
  const T gn = dt * fe * damping;

  // Derivative dγₙ/dvₙ = -δt⋅[k⋅δt⋅(1 - d⋅vₙ) + d⋅(fₑ₀ - δt⋅k⋅vₙ)].
  const T dgn_dvn = -dt * (k * dt * damping + d * fe);

  return std::make_pair(gn, dgn_dvn);
}

/* Computes the antiderivative N(vₙ) of the normal impulse associated with an
individual contact using a discrete Hunt-Crossley model.

The anti-derivative N(vₙ) is defined such that dN/dvₙ = γₙ(vₙ) and is used to
define the constraint cost ℓ_c(v).

@param dt Time step δt.
@param vn Normal contact velocity vₙ.
@param fe0 Previous time step normal force elastic contribution fₑ₀.
@param k Contact stiffness.
@param d Dissipation coefficient.

@returns The anti-derivative N(vₙ).
*/
template <typename T>
T CalcDiscreteHuntCrossleyAntiderivative(const T& dt, const T& vn, const T& fe0,
                                         const T& k, const T& d) {
  using std::min;

  // The discrete impulse is modeled as:
  //   γₙ(v) = δt⋅(fₑ₀ - δt⋅k⋅v)₊⋅(1 - d⋅v)₊.
  // We see that γₙ(v) = 0 for v ≥ v̂, with v̂ = min(vx, vd) and:
  //  vx = -ϕ₀/δt = fₑ₀/(δt⋅k),
  //  vd = 1/d.
  // Then for v < v̂, γₙ(v) is positive and we can verify that:
  //   N⁺(v) = δt⋅[v⋅(fₑ₀ + 1/2⋅Δf)-d⋅v²/2⋅(fₑ₀ + 2/3⋅Δf)],
  // is its antiderivative with Δf = -δt⋅k⋅v.
  // Since γₙ(v) = 0 for v ≥ v̂, then N(v) must be constant for v ≥ v̂.
  // Therefore we define it as:
  //   N(v) = N⁺(min(vn, v̂)).

  // We define the "dissipation" velocity vd at which the dissipation term
  // vanishes using a small tolerance so that vd goes to a very large number in
  // the limit to d = 0.
  const T vd = 1.0 / (d + 1.0e-20);

  // Similarly, we define vx as the velocity at which the elastic term goes
  // to zero. We use a small tolerance so that it goes to a very large
  // number in the limit of k = 0 (e.g., from discrete hydroelastics).
  const T vx = fe0 / dt / (k + 1.0e-20);

  // With the tolerances above in vd and vx, we can define a v̂ that goes to a
  // large number in the limit of either d = 0 or k = 0.
  const T v_hat = min(vx, vd);

  // Clamp vn to v̂.
  const T vn_clamped = min(vn, v_hat);

  // From the derivation above, N(v) = N⁺(vn_clamped).
  const T& v = vn_clamped;  // Alias to shorten notation.
  const T df = -dt * k * v;
  const T N = dt * (v * (fe0 + 1.0 / 2.0 * df) -
                    d * v * v / 2.0 * (fe0 + 2.0 / 3.0 * df));

  return N;
}

template <typename T>
PatchConstraintsPool<T>::PatchConstraintsPool(const IcfModel<T>* parent_model)
    : model_(parent_model) {
  DRAKE_ASSERT(parent_model != nullptr);
}

template <typename T>
PatchConstraintsPool<T>::~PatchConstraintsPool() = default;

template <typename T>
void PatchConstraintsPool<T>::Resize(std::span<const int> num_pairs_per_patch) {
  num_pairs_.assign(num_pairs_per_patch.begin(), num_pairs_per_patch.end());

  const int num_patches = num_pairs_.size();

  // Identity list of patch indices, used to drive the full-problem code paths.
  all_patches_.resize(num_patches);
  std::iota(all_patches_.begin(), all_patches_.end(), 0);
  const int num_pairs =
      std::accumulate(num_pairs_.begin(), num_pairs_.end(), 0);

  // Per-patch data.
  num_cliques_.resize(num_patches);
  bodies_.resize(num_patches);
  p_AB_W_.Resize(num_patches, 3, 1);
  dissipation_.resize(num_patches);
  static_friction_.resize(num_patches);
  dynamic_friction_.resize(num_patches);
  Rt_.resize(num_patches);

  // Per-pair data.
  p_BC_W_.Resize(num_pairs, 3, 1);
  normal_W_.Resize(num_pairs, 3, 1);
  stiffness_.resize(num_pairs);
  fe0_.resize(num_pairs);
  fn0_.resize(num_pairs);
  net_friction_.resize(num_pairs);

  // Start indexes for each patch.
  pair_data_start_.resize(num_patches);
  int previous_total = 0;
  for (int i = 0; i < num_patches; ++i) {
    pair_data_start_[i] = previous_total;
    previous_total += num_pairs_[i];
  }
}

template <typename T>
void PatchConstraintsPool<T>::SetPatch(int patch_index, int bodyA, int bodyB,
                                       const T& dissipation,
                                       const T& static_friction,
                                       const T& dynamic_friction,
                                       const Vector3<T>& p_AB_W) {
  DRAKE_ASSERT(0 <= patch_index && patch_index < num_patches());
  DRAKE_ASSERT(bodyA != bodyB);               // Same body never makes sense.
  DRAKE_ASSERT(!model().is_anchored(bodyB));  // B is never anchored.

  bodies_[patch_index] =
      std::make_pair(bodyB, bodyA);  // Dynamic body B always first.
  dissipation_[patch_index] = dissipation;
  static_friction_[patch_index] = static_friction;
  dynamic_friction_[patch_index] = dynamic_friction;
  p_AB_W_[patch_index] = p_AB_W;

  const int num_cliques =
      (model().is_anchored(bodyA) || model().is_anchored(bodyB)) ? 1 : 2;
  num_cliques_[patch_index] = num_cliques;

  // Compute per-patch regularization of friction. We use a "spherical body
  // approximation" for the estimation of the Delassus operator. A sphere has
  // gyration radius of g = 5/2 R (with R the radius). A contact will happen
  // at distance R from the CoM.
  // Thus the Delassus operator will be:
  //   W = 1/m⋅[I₃   0
  //           [ 0   R²/g²]
  // It's RMS norm will be w = sqrt(7)/m ≈ 2.65/m.
  T w = 2.65 / model().body_mass(bodies_[patch_index].first);
  if (num_cliques == 2) {
    w += 2.65 / model().body_mass(bodies_[patch_index].second);
  }
  Rt_[patch_index] = sigma_ * w;
}

template <typename T>
void PatchConstraintsPool<T>::SetPair(const int patch_index,
                                      const int pair_index,
                                      const Vector3<T>& p_BoC_W,
                                      const Vector3<T>& normal_W, const T& fe0,
                                      const T& stiffness) {
  using std::max;
  DRAKE_ASSERT(0 <= patch_index && patch_index < num_patches());
  DRAKE_ASSERT(0 <= pair_index && pair_index < num_pairs_[patch_index]);
  const int i = patch_pair_index(patch_index, pair_index);

  p_BC_W_[i] = p_BoC_W;
  normal_W_[i] = normal_W;
  fe0_[i] = fe0;
  stiffness_[i] = stiffness;

  // Pre-computed quantities.
  const int num_cliques = num_cliques_[patch_index];

  // First clique.
  const Vector6<T>& V_WB = model().V_WB0(bodies_[patch_index].first);
  const auto w_WB = V_WB.template head<3>();
  const auto v_WB = V_WB.template tail<3>();
  Vector3<T> v_AcBc_W = v_WB + w_WB.cross(p_BoC_W);

  // Second clique.
  if (num_cliques == 2) {
    const Vector6<T>& V_WA = model().V_WB0(bodies_[patch_index].second);
    const Vector3<T> p_AC_W = p_AB_W_[patch_index] + p_BoC_W;
    const auto w_WA = V_WA.template head<3>();
    const auto v_WA = V_WA.template tail<3>();
    v_AcBc_W -= (v_WA + w_WA.cross(p_AC_W));
  }

  // N.B. the normal impulse is n₀ = (δt⋅fₑ₀))₊(1−dvₙ₀)₊, where (·)₊ = max(0,·).
  // However, model.time_step() may change between when the constraint is set
  // and when the problem is solved. Thus we only store the normal force fₙ₀ =
  // (fₑ₀)₊(1−dvₙ₀)₊ here and compute the impulse n₀ = δt⋅fₙ₀ later in
  // CalcData().
  const T& d = dissipation_[patch_index];
  const T vn0 = v_AcBc_W.dot(normal_W);
  const T damping = max(0.0, 1.0 - d * vn0);
  const T fn0 = max(0.0, fe0) * damping;
  fn0_[i] = fn0;

  // Coefficient of friction is determined based on previous velocity. This
  // allows us to consider a Streibeck-like curve while maintaining a convex
  // formulation.
  const T vt0 = (v_AcBc_W - vn0 * normal_W).norm();
  const T s = vt0 / stiction_tolerance_;
  const T& mu_s = static_friction_[patch_index];
  const T& mu_d = dynamic_friction_[patch_index];

  auto sigmoid = [](const T& x) -> T {
    return x / sqrt(1 + x * x);
  };

  const T mu =
      (mu_s - mu_d) * 0.5 * (1 - (sigmoid(s - 10) / sigmoid(10))) + mu_d;
  net_friction_[i] = mu;
}

template <typename T>
void PatchConstraintsPool<T>::CalcSparsityPattern(
    std::vector<std::vector<int>>* sparsity) const {
  DRAKE_ASSERT(sparsity != nullptr);
  for (int p = 0; p < num_patches(); ++p) {
    // We only need to add to the sparsity if body_a is not anchored.
    const int body_a = bodies_[p].second;
    if (!model().is_anchored(body_a)) {
      const int body_b = bodies_[p].first;
      DRAKE_ASSERT(!model().is_anchored(body_b));  // Body B is never anchored.
      const int c_a = model().body_to_clique(body_a);
      const int c_b = model().body_to_clique(body_b);
      if (c_a == c_b) continue;  // we do not add diagonal blocks here.
      const int c_min = std::min(c_a, c_b);
      const int c_max = std::max(c_a, c_b);
      sparsity->at(c_min).push_back(c_max);  // j > i blocks.
    }
  }
}

template <typename T>
void PatchConstraintsPool<T>::CalcData(
    const EigenPool<Vector6<T>>& V_WB,
    PatchConstraintsDataPool<T>* patch_data) const {
  DRAKE_ASSERT(patch_data != nullptr);
  patch_data->mutable_cost() = CalcData(V_WB, all_patches_, patch_data);
}

template <typename T>
T PatchConstraintsPool<T>::CalcData(
    const EigenPool<Vector6<T>>& V_WB, std::span<const int> patches,
    PatchConstraintsDataPool<T>* patch_data) const {
  DRAKE_ASSERT(patch_data != nullptr);
  CalcContactVelocities(V_WB, patches, &patch_data->mutable_v_AcBc_W_pool());
  return CalcPatchQuantities(patch_data->mutable_v_AcBc_W_pool(), patches,
                             &patch_data->mutable_cost_pool(),
                             &patch_data->mutable_Gamma_Bo_W_pool(),
                             &patch_data->mutable_G_Bp_pool());
}

template <typename T>
void PatchConstraintsPool<T>::AccumulateGradient(const IcfData<T>& data,
                                                 VectorX<T>* gradient) const {
  AccumulateGradient(data, all_patches_, gradient);
}

template <typename T>
void PatchConstraintsPool<T>::AccumulateGradient(const IcfData<T>& data,
                                                 std::span<const int> patches,
                                                 VectorX<T>* gradient) const {
  DRAKE_ASSERT(gradient != nullptr);
  const PatchConstraintsDataPool<T>& patch_data = data.patch_constraints_data();
  const EigenPool<Vector6<T>>& Gamma_Bo_W_pool = patch_data.Gamma_Bo_W_pool();

  for (int p : patches) {
    const int body_a = bodies_[p].second;
    const int body_b = bodies_[p].first;
    const int c_b = model().body_to_clique(body_b);
    const int c_a = model().body_to_clique(body_a);  // negative if anchored.

    // First clique, body B.
    DRAKE_ASSERT(!model().is_anchored(body_b));  // Body B is never anchored.

    const Vector6<T>& Gamma_Bo_W = Gamma_Bo_W_pool[p];

    Eigen::VectorBlock<VectorX<T>> gradient_b =
        model().mutable_clique_segment(c_b, gradient);
    if (model().is_floating(body_b)) {
      gradient_b.noalias() -= Gamma_Bo_W;
    } else {
      ConstJacobianView J_WB = model().J_WB(body_b);
      gradient_b.noalias() -= J_WB.transpose() * Gamma_Bo_W;
    }

    // Second clique, for body A, only contributes if not anchored.
    if (!model().is_anchored(body_a)) {
      const Vector3<T>& p_AB_W = p_AB_W_[p];
      const Vector6<T> minus_Gamma_Ao_W = ShiftSpatialForce(Gamma_Bo_W, p_AB_W);
      Eigen::VectorBlock<VectorX<T>> gradient_a =
          model().mutable_clique_segment(c_a, gradient);

      if (model().is_floating(body_a)) {
        gradient_a.noalias() += minus_Gamma_Ao_W;
      } else {
        ConstJacobianView J_WA = model().J_WB(body_a);
        gradient_a.noalias() += J_WA.transpose() * minus_Gamma_Ao_W;
      }
    }
  }
}

template <typename T>
void PatchConstraintsPool<T>::AccumulateHessian(
    const IcfData<T>& data,
    BlockSparseSymmetricMatrix<MatrixX<T>>* hessian) const {
  DRAKE_ASSERT(hessian != nullptr);

  const PatchConstraintsDataPool<T>& patch_data = data.patch_constraints_data();

  auto& H_BB_pool = data.scratch().H_BB_pool;
  auto& H_AA_pool = data.scratch().H_AA_pool;
  auto& H_AB_pool = data.scratch().H_AB_pool;
  auto& H_BA_pool = data.scratch().H_BA_pool;
  auto& GJa_pool = data.scratch().GJa_pool;
  auto& GJb_pool = data.scratch().GJb_pool;

  const EigenPool<Matrix6<T>>& G_Bp_pool = patch_data.G_Bp_pool();

  for (int p = 0; p < num_patches(); ++p) {
    const int body_a = bodies_[p].second;
    const int body_b = bodies_[p].first;
    const int c_b = model().body_to_clique(body_b);
    const int c_a = model().body_to_clique(body_a);  // negative if anchored.
    const int nv_b = model().clique_size(c_b);
    const int nv_a = model().clique_size(c_a);  // zero if anchored.

    // First clique, body B.
    DRAKE_ASSERT(!model().is_anchored(body_b));  // Body B is never anchored.

    // Accumulate Hessian.
    H_BB_pool.Resize(1, nv_b, nv_b);
    auto H_BB = H_BB_pool[0];

    const Matrix6<T>& G_Bp = G_Bp_pool[p];
    ConstJacobianView J_WB = model().J_WB(body_b);
    if (model().is_floating(body_b)) {
      H_BB.noalias() = G_Bp;
    } else {
      GJb_pool.Resize(1, 6, nv_b);
      auto GJb = GJb_pool[0];
      GJb.noalias() = G_Bp * J_WB;
      H_BB.noalias() = J_WB.transpose() * GJb;
    }
    hessian->AddToBlock(c_b, c_b, H_BB);

    // Second clique, for body A, only contributes if not anchored.
    if (!model().is_anchored(body_a)) {
      GJa_pool.Resize(1, 6, nv_a);
      auto GJa = GJa_pool[0];

      const Vector3<T>& p_AB_W = p_AB_W_[p];
      ConstJacobianView J_WA = model().J_WB(body_a);

      // Accumulate (upper triangular) Hessian.
      H_AA_pool.Resize(1, nv_a, nv_a);
      auto H_AA = H_AA_pool[0];
      const Matrix6<T> G_Phi = ShiftFromTheRight(G_Bp, p_AB_W);  // = Gₚ⋅Φ
      const Matrix6<T> G_Ap = ShiftFromTheLeft(G_Phi, p_AB_W);   // = Φᵀ⋅Gₚ⋅Φ

      // When c_a != c_b, we only write the lower triangular portion.
      // If c_a == c_b, we must compute both terms.
      GJa.noalias() = G_Phi * J_WA;
      H_BA_pool.Resize(1, nv_b, nv_a);
      auto H_BA = H_BA_pool[0];
      if (model().is_floating(body_b)) {
        H_BA.noalias() = -GJa;
      } else {
        H_BA.noalias() = -J_WB.transpose() * GJa;
      }
      if (c_b > c_a) {
        hessian->AddToBlock(c_b, c_a, H_BA);
      }
      if (c_a > c_b) {
        // N.B. Doing:
        // AddToBlock(c_a, c_b, H_BA.transpose()) allocates temp memory!!!.
        H_AB_pool.Resize(1, nv_a, nv_b);
        auto H_AB = H_AB_pool[0];
        H_AB = H_BA.transpose();
        hessian->AddToBlock(c_a, c_b, H_AB);
      }
      if (c_b == c_a) {
        // We must add both. Additionally, AddToBlock assumes symmetric blocks
        // for diagonal blocks.
        H_BB.noalias() = H_BA + H_BA.transpose();  // Re-use H_BB.
        hessian->AddToBlock(c_a, c_b, H_BB);
      }

      if (model().is_floating(body_a)) {
        H_AA.noalias() = G_Ap;
      } else {
        GJa.noalias() = G_Ap * J_WA;
        H_AA.noalias() = J_WA.transpose() * GJa;
      }
      hessian->AddToBlock(c_a, c_a, H_AA);
    }
  }
}

template <typename T>
void PatchConstraintsPool<T>::AccumulateHessian(
    const IcfData<T>& data, std::span<const int> patches,
    std::span<const int> clique_to_block, int island,
    BlockSparseSymmetricMatrix<MatrixX<T>>* hessian) const {
  DRAKE_ASSERT(hessian != nullptr);

  const PatchConstraintsDataPool<T>& patch_data = data.patch_constraints_data();

  auto& H_BB_pool = data.scratch(island).H_BB_pool;
  auto& H_AA_pool = data.scratch(island).H_AA_pool;
  auto& H_AB_pool = data.scratch(island).H_AB_pool;
  auto& H_BA_pool = data.scratch(island).H_BA_pool;
  auto& GJa_pool = data.scratch(island).GJa_pool;
  auto& GJb_pool = data.scratch(island).GJb_pool;

  const EigenPool<Matrix6<T>>& G_Bp_pool = patch_data.G_Bp_pool();

  for (int p : patches) {
    const int body_a = bodies_[p].second;
    const int body_b = bodies_[p].first;
    const int c_b = model().body_to_clique(body_b);
    const int c_a = model().body_to_clique(body_a);  // negative if anchored.
    const int nv_b = model().clique_size(c_b);
    const int nv_a = model().clique_size(c_a);  // zero if anchored.

    // Block indices in the island's local sub-Hessian. Because island cliques
    // are sorted ascending, local indices preserve the global clique ordering,
    // so the c_b/c_a comparisons below remain valid for lower-triangular
    // writes.
    const int b_b = clique_to_block[c_b];

    // First clique, body B.
    DRAKE_ASSERT(!model().is_anchored(body_b));  // Body B is never anchored.

    H_BB_pool.Resize(1, nv_b, nv_b);
    auto H_BB = H_BB_pool[0];

    const Matrix6<T>& G_Bp = G_Bp_pool[p];
    ConstJacobianView J_WB = model().J_WB(body_b);
    if (model().is_floating(body_b)) {
      H_BB.noalias() = G_Bp;
    } else {
      GJb_pool.Resize(1, 6, nv_b);
      auto GJb = GJb_pool[0];
      GJb.noalias() = G_Bp * J_WB;
      H_BB.noalias() = J_WB.transpose() * GJb;
    }
    hessian->AddToBlock(b_b, b_b, H_BB);

    // Second clique, for body A, only contributes if not anchored.
    if (!model().is_anchored(body_a)) {
      const int b_a = clique_to_block[c_a];
      GJa_pool.Resize(1, 6, nv_a);
      auto GJa = GJa_pool[0];

      const Vector3<T>& p_AB_W = p_AB_W_[p];
      ConstJacobianView J_WA = model().J_WB(body_a);

      H_AA_pool.Resize(1, nv_a, nv_a);
      auto H_AA = H_AA_pool[0];
      const Matrix6<T> G_Phi = ShiftFromTheRight(G_Bp, p_AB_W);  // = Gₚ⋅Φ
      const Matrix6<T> G_Ap = ShiftFromTheLeft(G_Phi, p_AB_W);   // = Φᵀ⋅Gₚ⋅Φ

      GJa.noalias() = G_Phi * J_WA;
      H_BA_pool.Resize(1, nv_b, nv_a);
      auto H_BA = H_BA_pool[0];
      if (model().is_floating(body_b)) {
        H_BA.noalias() = -GJa;
      } else {
        H_BA.noalias() = -J_WB.transpose() * GJa;
      }
      if (c_b > c_a) {
        hessian->AddToBlock(b_b, b_a, H_BA);
      }
      if (c_a > c_b) {
        H_AB_pool.Resize(1, nv_a, nv_b);
        auto H_AB = H_AB_pool[0];
        H_AB = H_BA.transpose();
        hessian->AddToBlock(b_a, b_b, H_AB);
      }
      if (c_b == c_a) {
        H_BB.noalias() = H_BA + H_BA.transpose();  // Re-use H_BB.
        hessian->AddToBlock(b_a, b_b, H_BB);
      }

      if (model().is_floating(body_a)) {
        H_AA.noalias() = G_Ap;
      } else {
        GJa.noalias() = G_Ap * J_WA;
        H_AA.noalias() = J_WA.transpose() * GJa;
      }
      hessian->AddToBlock(b_a, b_a, H_AA);
    }
  }
}

template <typename T>
void PatchConstraintsPool<T>::CalcCostAlongLine(
    const PatchConstraintsDataPool<T>& patch_data,
    const EigenPool<Vector6<T>>& U_WB_pool,
    EigenPool<Vector6<T>>* U_AbB_W_pool_ptr, T* dcost, T* d2cost) const {
  CalcCostAlongLine(patch_data, U_WB_pool, all_patches_, U_AbB_W_pool_ptr,
                    dcost, d2cost);
}

template <typename T>
void PatchConstraintsPool<T>::CalcCostAlongLine(
    const PatchConstraintsDataPool<T>& patch_data,
    const EigenPool<Vector6<T>>& U_WB_pool, std::span<const int> patches,
    EigenPool<Vector6<T>>* U_AbB_W_pool_ptr, T* dcost, T* d2cost) const {
  DRAKE_ASSERT(U_AbB_W_pool_ptr != nullptr);
  DRAKE_ASSERT(dcost != nullptr);
  DRAKE_ASSERT(d2cost != nullptr);
  auto& U_AbB_W_pool = *U_AbB_W_pool_ptr;
  DRAKE_ASSERT(U_AbB_W_pool.size() == num_patches());
  CalcConstraintVelocities(U_WB_pool, patches, &U_AbB_W_pool);

  const auto& Gamma_pool = patch_data.Gamma_Bo_W_pool();
  const EigenPool<Matrix6<T>>& G_Bp_pool = patch_data.G_Bp_pool();
  *dcost = 0;
  *d2cost = 0;
  Vector6<T> dGamma;
  for (int p : patches) {
    // N.B. All this is contiguous, so we could use a single dot product.
    const Vector6<T>& U = U_AbB_W_pool[p];
    const Vector6<T>& Gamma = Gamma_pool[p];
    const Matrix6<T>& G = G_Bp_pool[p];
    *dcost -= U.dot(Gamma);  // -= Uᵀ⋅Γ

    dGamma = G * U;
    *d2cost += U.dot(dGamma);  // += Uᵀ⋅Gₚ⋅U
  }
}

template <typename T>
T PatchConstraintsPool<T>::CalcLaggedHuntCrossleyModel(
    int p, int k, const Vector3<T>& v_AcBc_W, Vector3<T>* gamma_Bc_W,
    Matrix3<T>* G) const {
  using std::max;
  const int pk = patch_pair_index(p, k);
  const Vector3<T>& normal_W = normal_W_[pk];
  const T& dt = model().time_step();

  const T& mu = net_friction_[pk];
  const T& d = dissipation_[p];
  const T& stiffness = stiffness_[pk];
  const T& n0 = fn0_[pk] * dt;
  const T& fe0 = fe0_[pk];

  // Regularization for the stiction tolerance
  const T sap_stiction_tolerance = mu * Rt_[p] * n0;
  const T vs = max(stiction_tolerance_, sap_stiction_tolerance);

  // Normal velocity. Positive when bodies move apart.
  const T vn = v_AcBc_W.dot(normal_W);
  const Vector3<T> vt_AcBc_W = v_AcBc_W - vn * normal_W;
  const T vt_soft = SoftNorm(vt_AcBc_W, vs);
  const Vector3<T> t_hat_W = vt_AcBc_W / (vt_soft + vs);

  // Normal impulse γₙ(vₙ), derivative dγₙ/dvₙ, and antiderivative N(vₙ).
  const auto [gn, dgn_dvn] =
      CalcDiscreteHuntCrossleyImpulseAndDerivative(dt, vn, fe0, stiffness, d);
  const T N = CalcDiscreteHuntCrossleyAntiderivative(dt, vn, fe0, stiffness, d);

  // Constraint cost and impulse (negative gradient), including both friction
  // and normal force contributions.
  const T cost = mu * vt_soft * n0 - N;
  *gamma_Bc_W = -mu * t_hat_W * n0 + gn * normal_W;

  // Pn is a SPD projection matrix with eigenvalues {1, 0, 0}.
  const Matrix3<T> Pn = normal_W * normal_W.transpose();

  // Pt is SPD with eigenvalues {‖t̂‖², 0, 0}. Note that since ‖t̂‖² < 1, Pt is
  // not a projection matrix.
  const Matrix3<T> Pt = t_hat_W * t_hat_W.transpose();

  // M = Pperp(t) * Pperp(n) is SPD with eigenvalues {1 - ‖t̂‖², 0, 1}, all
  // positive since ‖t̂‖ < 1.
  const Matrix3<T> M = Matrix3<T>::Identity() - Pt - Pn;

  // Constraint Hessian, including both friction and normal force contributions.
  *G = mu * n0 / (vt_soft + vs) * M - dgn_dvn * Pn;

  return cost;
}

template <typename T>
void PatchConstraintsPool<T>::CalcConstraintVelocities(
    const EigenPool<Vector6<T>>& V_WB_pool, std::span<const int> patches,
    EigenPool<Vector6<T>>* V_AbB_W_pool) const {
  DRAKE_ASSERT(V_WB_pool.size() == model().num_bodies());
  DRAKE_ASSERT(V_AbB_W_pool->size() == num_patches());

  for (int p : patches) {
    const int num_cliques = num_cliques_[p];

    const int bodyB = bodies_[p].first;
    // If A is anchored, bodyA, will reference a zero velocity that won't
    // get used.
    const int bodyA = bodies_[p].second;
    const Vector6<T>& V_WB = V_WB_pool[bodyB];
    const Vector6<T>& V_WA = V_WB_pool[bodyA];

    Vector6<T>& V_AbB_W = (*V_AbB_W_pool)[p];
    V_AbB_W = V_WB;

    if (num_cliques == 2) {
      const Vector3<T>& p_AB_W = p_AB_W_[p];

      const auto w_WA = V_WA.template head<3>();
      const auto v_WA = V_WA.template tail<3>();
      auto w_AbB_W = V_AbB_W.template head<3>();
      auto v_AbB_W = V_AbB_W.template tail<3>();

      w_AbB_W -= w_WA;
      v_AbB_W -= (v_WA + w_WA.cross(p_AB_W));
    }
  }
}

template <typename T>
void PatchConstraintsPool<T>::CalcContactVelocities(
    const EigenPool<Vector6<T>>& V_WB_pool, std::span<const int> patches,
    EigenPool<Vector3<T>>* v_AcBc_W_pool) const {
  for (int p : patches) {
    const int num_cliques = num_cliques_[p];
    const int num_pairs = num_pairs_[p];

    const int bodyB = bodies_[p].first;
    // If A is anchored, bodyA = 0, will reference a zero velocity that won't
    // get used.
    const int bodyA = bodies_[p].second;
    const Vector6<T>& V_WB = V_WB_pool[bodyB];
    const Vector6<T>& V_WA = V_WB_pool[bodyA];

    for (int k = 0; k < num_pairs; ++k) {
      const int pk = patch_pair_index(p, k);
      Vector3<T>& v_AcBc_W = (*v_AcBc_W_pool)[pk];

      // First clique, always corresponding to body B.
      const Vector3<T>& p_BC_W = p_BC_W_[pk];
      const auto w_WB = V_WB.template head<3>();
      const auto v_WB = V_WB.template tail<3>();
      v_AcBc_W = v_WB + w_WB.cross(p_BC_W);

      // Second clique. Always corresponding to body A.
      if (num_cliques == 2) {
        const Vector3<T> p_AC_W = p_AB_W_[p] + p_BC_W_[pk];
        const auto w_WA = V_WA.template head<3>();
        const auto v_WA = V_WA.template tail<3>();
        v_AcBc_W -= (v_WA + w_WA.cross(p_AC_W));
      }
    }
  }
}

template <typename T>
T PatchConstraintsPool<T>::CalcPatchQuantities(
    const EigenPool<Vector3<T>>& v_AcBc_W_pool, std::span<const int> patches,
    std::vector<T>* cost_pool, EigenPool<Vector6<T>>* spatial_impulses_pool,
    EigenPool<Matrix6<T>>* patch_hessians_pool) const {
  EigenPool<Vector6<T>>& Gamma_Bo_W_pool = *spatial_impulses_pool;
  EigenPool<Matrix6<T>>& G_Bp_pool = *patch_hessians_pool;

  T total_cost = 0.0;
  for (int p : patches) {
    const int num_pairs = num_pairs_[p];

    // Accumulate impulses on the patch for the first clique only. This is
    // always body B. Zero per patch (rather than the whole pool) so that
    // islands writing disjoint patches do not interfere.
    Vector6<T>& Gamma_Bo_W = Gamma_Bo_W_pool[p];
    Matrix6<T>& G_Bp = G_Bp_pool[p];
    Gamma_Bo_W.setZero();
    G_Bp.setZero();

    cost_pool->at(p) = 0.0;
    for (int k = 0; k < num_pairs; ++k) {
      const int pk = patch_pair_index(p, k);
      const Vector3<T>& v_AcBc_W = v_AcBc_W_pool[pk];

      Vector3<T> gamma_Bc_W;
      Matrix3<T> Gk;
      cost_pool->at(p) +=
          CalcLaggedHuntCrossleyModel(p, k, v_AcBc_W, &gamma_Bc_W, &Gk);

      // Shift from Ck to B and accumulate.
      const Vector3<T>& p_BC_W = p_BC_W_[pk];
      Gamma_Bo_W.template head<3>() += p_BC_W.cross(gamma_Bc_W);
      Gamma_Bo_W.template tail<3>() += gamma_Bc_W;

      // Accumulate onto the path Hessian Gp.
      G_Bp += ShiftSecondOrderTensor(Gk, p_BC_W);
    }
    total_cost += cost_pool->at(p);
  }
  return total_cost;
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::multibody::contact_solvers::icf::internal::
        PatchConstraintsPool);
