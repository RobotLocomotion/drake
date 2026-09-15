#include "drake/multibody/contact_solvers/icf/holonomic_constraints_pool.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "drake/common/autodiff.h"
#include "drake/common/extract_double.h"
#include "drake/multibody/contact_solvers/icf/ball_constraints_pool.h"
#include "drake/multibody/contact_solvers/icf/distance_constraints_pool.h"
#include "drake/multibody/contact_solvers/icf/icf_model.h"
#include "drake/multibody/contact_solvers/icf/weld_constraints_pool.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

template <typename T, int N, typename Derived>
HolonomicConstraintsPool<T, N, Derived>::HolonomicConstraintsPool(
    const IcfModel<T>* parent_model)
    : model_(parent_model) {
  DRAKE_DEMAND(parent_model != nullptr);
}

template <typename T, int N, typename Derived>
HolonomicConstraintsPool<T, N, Derived>::~HolonomicConstraintsPool() = default;

template <typename T, int N, typename Derived>
void HolonomicConstraintsPool<T, N, Derived>::ResizeCommon(
    const int num_constraints) {
  ResizeAllConstraints(num_constraints);
  body_pairs_.resize(num_constraints);
  g0_.Resize(num_constraints, N, 1);
  stiffness_.resize(num_constraints);
  damping_.resize(num_constraints);
  R_.resize(num_constraints);
  v_hat_.Resize(num_constraints, N, 1);
}

template <typename T, int N, typename Derived>
void HolonomicConstraintsPool<T, N, Derived>::Resize(
    const int num_constraints) {
  ResizeCommon(num_constraints);
  mutable_derived().ResizeGeometry(num_constraints);
}

template <typename T, int N, typename Derived>
void HolonomicConstraintsPool<T, N, Derived>::SetCommon(
    int index, int bodyA, int bodyB, const ConstraintVector& g0,
    const T& stiffness, const T& damping) {
  DRAKE_ASSERT(0 <= index && index < num_constraints());
  DRAKE_ASSERT(!model().is_anchored(bodyB));
  DRAKE_ASSERT(stiffness > 0);
  DRAKE_ASSERT(damping >= 0);
  body_pairs_[index] = std::make_pair(bodyA, bodyB);
  g0_[index] = g0;
  stiffness_[index] = stiffness;
  damping_[index] = damping;
}

template <typename T, int N, typename Derived>
void HolonomicConstraintsPool<T, N, Derived>::CalcSparsityPattern(
    std::vector<std::vector<int>>* sparsity) const {
  DRAKE_ASSERT(sparsity != nullptr);

  for (const auto& [bodyA, bodyB] : body_pairs_) {
    if (!model().is_anchored(bodyA)) {
      const int c_A = model().body_to_clique(bodyA);
      const int c_B = model().body_to_clique(bodyB);
      if (c_A == c_B) continue;  // No off-diagonal block for same-clique.
      sparsity->at(std::min(c_A, c_B)).push_back(std::max(c_A, c_B));
    }
  }
}

template <typename T, int N, typename Derived>
void HolonomicConstraintsPool<T, N, Derived>::CalcData(
    const EigenPool<Vector6<T>>& V_WB, DataPool* data) const {
  DRAKE_ASSERT(ssize(all_constraints_) == num_constraints());
  data->mutable_cost() = CalcData(V_WB, all_constraints_, data);
}

template <typename T, int N, typename Derived>
T HolonomicConstraintsPool<T, N, Derived>::CalcData(
    const EigenPool<Vector6<T>>& V_WB, std::span<const int> constraints,
    DataPool* data) const {
  DRAKE_ASSERT(data != nullptr);
  T& cost = data->mutable_cost();
  cost = 0;
  for (int k : constraints) {
    const int bodyA = body_pairs_[k].first;
    const int bodyB = body_pairs_[k].second;

    // vc = constraint velocity, computed by the derived. The A
    // term is absent (null) when A is anchored.
    const ConstraintVector vc = derived().CalcConstraintVelocity(
        k, V_WB[bodyB], model().is_anchored(bodyA) ? nullptr : &V_WB[bodyA]);

    // γ = R⁻¹⋅(v̂ − vc), cost = ½(v̂ − vc)ᵀ⋅γ.
    const ConstraintVector dv = v_hat_[k] - vc;
    const ConstraintVector gamma = dv / R_[k];
    data->mutable_gamma(k) = gamma;
    cost += 0.5 * dv.dot(gamma);
  }
  return cost;
}

template <typename T, int N, typename Derived>
void HolonomicConstraintsPool<T, N, Derived>::AccumulateGradient(
    const IcfData<T>& data, VectorX<T>* gradient) const {
  DRAKE_ASSERT(ssize(all_constraints_) == num_constraints());
  AccumulateGradient(data, all_constraints_, gradient);
}

template <typename T, int N, typename Derived>
void HolonomicConstraintsPool<T, N, Derived>::AccumulateGradient(
    const IcfData<T>& data, std::span<const int> constraints,
    VectorX<T>* gradient) const {
  DRAKE_ASSERT(gradient != nullptr);
  const DataPool& data_pool = derived().GetDataPool(data);

  for (int k : constraints) {
    const int bodyA = body_pairs_[k].first;
    const int bodyB = body_pairs_[k].second;
    const int c_B = model().body_to_clique(bodyB);
    const bool A_is_dynamic = !model().is_anchored(bodyA);
    const ConstraintVector& gamma = data_pool.gamma(k);

    // The derived shifts the constraint impulse to the body
    // origins: Γ_Bo on B, and Γ_Ao when A is dynamic.
    Vector6<T> Gamma_Bo, Gamma_Ao;
    derived().CalcSpatialImpulses(k, gamma, &Gamma_Bo,
                                  A_is_dynamic ? &Gamma_Ao : nullptr);

    Eigen::VectorBlock<VectorX<T>> gradient_b =
        model().mutable_clique_segment(c_B, gradient);
    if (model().is_floating(bodyB)) {
      gradient_b.noalias() -= Gamma_Bo;
    } else {
      auto J_WB = model().J_WB(bodyB);
      gradient_b.noalias() -= J_WB.transpose() * Gamma_Bo;
    }

    if (A_is_dynamic) {
      const int c_A = model().body_to_clique(bodyA);
      Eigen::VectorBlock<VectorX<T>> gradient_a =
          model().mutable_clique_segment(c_A, gradient);
      if (model().is_floating(bodyA)) {
        gradient_a.noalias() -= Gamma_Ao;
      } else {
        auto J_WA = model().J_WB(bodyA);
        gradient_a.noalias() -= J_WA.transpose() * Gamma_Ao;
      }
    }
  }
}

template <typename T, int N, typename Derived>
void HolonomicConstraintsPool<T, N, Derived>::PrecomputeHessianBlocks() {
  hessian_blocks_.resize(num_constraints());

  using std::isinf;
  // Near-rigid parameter β.
  const double kBeta = IcfModel<T>::kBeta;
  const T dt = model().time_step();
  const T dt_eff = model().effective_time_step();
  const T taud = kBeta * dt_eff / M_PI;
  // R⁻¹ = K·dt·(dt + τd) where K = 4π²/(β²·dt_eff²·w), so
  // R_diag = w / (K·dt·(dt + τd)) = β²·dt_eff²·w / (4π²·dt·(dt + τd)).
  const T r_scale = (kBeta * kBeta * dt_eff * dt_eff) /
                    (4.0 * M_PI * M_PI * dt * (dt + taud));

  for (int k = 0; k < num_constraints(); ++k) {
    const int bodyA = body_pairs_[k].first;
    const int bodyB = body_pairs_[k].second;
    const int c_B = model().body_to_clique(bodyB);
    const int c_A = model().body_to_clique(bodyA);  // negative if anchored.

    HessianBlock& hb = hessian_blocks_[k];
    hb.c_B = c_B;
    hb.c_A = c_A;
    hb.A_is_dynamic = !model().is_anchored(bodyA);

    // Approximate W_B = J_WB⋅diag(M_B)⁻¹⋅J_WBᵀ using body mass.
    // For a single rigid body, this is approximately 1/mass for translational
    // DOFs. We use a scalar approximation: w ≈ 1/m_B (+ 1/m_A if not anchored).
    T w = 1.0 / model().body_mass(bodyB);
    if (hb.A_is_dynamic) w += 1.0 / model().body_mass(bodyA);
    const T R_near_rigid = r_scale * w;

    // Compliant regularization for the user supplied stiffness/damping:
    // R_compliant = 1/(dt·(dt + τ)·k). Infinite stiffness ⇒ near-rigid.
    const T& stiffness = stiffness_[k];
    DRAKE_DEMAND(stiffness > 0);
    T R_compliant, tau;
    if (isinf(ExtractDoubleOrThrow(stiffness))) {
      tau = 0.0;
      R_compliant = 0.0;
    } else {
      tau = damping_[k] / stiffness;
      R_compliant = 1.0 / (dt * (dt + tau) * stiffness);
    }

    // Use the softer of the two.
    T R, tau_eff;
    if (R_compliant < R_near_rigid) {
      R = R_near_rigid;
      tau_eff = taud;
    } else {
      R = R_compliant;
      tau_eff = tau;
    }
    R_[k] = R;
    v_hat_[k] = -g0_[k] / (dt + tau_eff);
    const T R_inv = 1.0 / R;

    // The derived builds the 6×6 body-space Hessian blocks
    // G_Xp = Φ(p_X)ᵀ⋅G⋅Φ(p_X). G_Ap/G_cross are only needed when
    // A is dynamic.
    Matrix6<T> G_Bp, G_Ap, G_cross;
    derived().CalcHessianBlocks(k, R_inv, &G_Bp,
                                hb.A_is_dynamic ? &G_Ap : nullptr,
                                hb.A_is_dynamic ? &G_cross : nullptr);

    // Body B contribution.
    // H_BB = J_WBᵀ⋅G_Bp⋅J_WB
    DRAKE_ASSERT(!model().is_anchored(bodyB));
    auto J_WB = model().J_WB(bodyB);
    if (model().is_floating(bodyB)) {
      hb.H_BB = G_Bp;
    } else {
      const int nv_b = model().clique_size(c_B);
      Matrix6X<T> GJb(6, nv_b);
      GJb.noalias() = G_Bp * J_WB;
      hb.H_BB.resize(nv_b, nv_b);
      hb.H_BB.noalias() = J_WB.transpose() * GJb;
    }

    // Body A contribution, only if not anchored.
    // H_AA = J_WAᵀ⋅G_Ap⋅J_WA
    if (hb.A_is_dynamic) {
      auto J_WA = model().J_WB(bodyA);
      if (model().is_floating(bodyA)) {
        hb.H_AA = G_Ap;
      } else {
        const int nv_a = model().clique_size(c_A);
        Matrix6X<T> GJa(6, nv_a);
        GJa.noalias() = G_Ap * J_WA;
        hb.H_AA.resize(nv_a, nv_a);
        hb.H_AA.noalias() = J_WA.transpose() * GJa;
      }

      // Cross term: H_BA = −J_WBᵀ⋅G_cross⋅J_WA
      const int nv_a = model().clique_size(c_A);
      const int nv_b = model().clique_size(c_B);
      MatrixX<T> H_BA(nv_b, nv_a);
      {
        Matrix6X<T> GJa(6, nv_a);
        GJa.noalias() = G_cross * J_WA;
        if (model().is_floating(bodyB)) {
          H_BA = GJa;
        } else {
          H_BA.noalias() = J_WB.transpose() * GJa;
        }
      }

      // Store the final cross block in the correct orientation for AddToBlock.
      if (c_B > c_A) {
        hb.cross_row = c_B;
        hb.cross_col = c_A;
        hb.H_cross = H_BA;
      } else if (c_A > c_B) {
        hb.cross_row = c_A;
        hb.cross_col = c_B;
        hb.H_cross = H_BA.transpose();
      } else {
        hb.cross_row = c_A;
        hb.cross_col = c_B;
        hb.H_cross = H_BA + H_BA.transpose();
      }
    }
  }
}

template <typename T, int N, typename Derived>
void HolonomicConstraintsPool<T, N, Derived>::AccumulateHessian(
    const IcfData<T>& data,
    contact_solvers::internal::BlockSparseSymmetricMatrix<MatrixX<T>>* hessian)
    const {
  DRAKE_ASSERT(ssize(all_constraints_) == num_constraints());
  AccumulateHessian(data, all_constraints_, {}, 0, hessian);
}

template <typename T, int N, typename Derived>
void HolonomicConstraintsPool<T, N, Derived>::AccumulateHessian(
    const IcfData<T>&, std::span<const int> constraints,
    std::span<const int> clique_to_block, int /* island */,
    contact_solvers::internal::BlockSparseSymmetricMatrix<MatrixX<T>>* hessian)
    const {
  DRAKE_ASSERT(hessian != nullptr);

  auto c2b = [&](int clique) {
    if (ssize(clique_to_block)) {
      return clique_to_block[clique];
    }
    return clique;
  };

  for (int k : constraints) {
    const HessianBlock& hb = hessian_blocks_[k];
    const int b_b = c2b(hb.c_B);
    hessian->AddToBlock(b_b, b_b, hb.H_BB);

    if (hb.A_is_dynamic) {
      const int b_a = c2b(hb.c_A);
      hessian->AddToBlock(b_a, b_a, hb.H_AA);
      hessian->AddToBlock(c2b(hb.cross_row), c2b(hb.cross_col), hb.H_cross);
    }
  }
}

template <typename T, int N, typename Derived>
void HolonomicConstraintsPool<T, N, Derived>::CalcCostAlongLine(
    const DataPool& data, const EigenPool<Vector6<T>>& U_WB, T* dcost,
    T* d2cost) const {
  DRAKE_ASSERT(ssize(all_constraints_) == num_constraints());
  CalcCostAlongLine(data, U_WB, all_constraints_, dcost, d2cost);
}

template <typename T, int N, typename Derived>
void HolonomicConstraintsPool<T, N, Derived>::CalcCostAlongLine(
    const DataPool& data, const EigenPool<Vector6<T>>& U_WB,
    std::span<const int> constraints, T* dcost, T* d2cost) const {
  DRAKE_ASSERT(dcost != nullptr);
  DRAKE_ASSERT(d2cost != nullptr);
  *dcost = 0.0;
  *d2cost = 0.0;
  for (int k : constraints) {
    const int bodyA = body_pairs_[k].first;
    const int bodyB = body_pairs_[k].second;

    // The constraint velocity in the search direction w reuses the hook.
    const ConstraintVector uc = derived().CalcConstraintVelocity(
        k, U_WB[bodyB], model().is_anchored(bodyA) ? nullptr : &U_WB[bodyA]);

    const ConstraintVector& gamma = data.gamma(k);
    (*dcost) -= gamma.dot(uc);        // dℓ̃/dα = −γᵀ⋅uc
    (*d2cost) += uc.dot(uc) / R_[k];  // d²ℓ̃/dα² = ucᵀ⋅G⋅uc
  }
}

template <typename T, int N, typename Derived>
void HolonomicConstraintsPool<T, N, Derived>::ReduceInto(
    const ReducedMapping& mapping, Derived* reduced_pool) const {
  // (Over-)allocate then clear so the appends below do not allocate.
  reduced_pool->Resize(num_constraints());
  reduced_pool->Resize(0);

  for (int k = 0; k < num_constraints(); ++k) {
    const int body_a = body_pairs_[k].first;
    const int body_b = body_pairs_[k].second;
    const int c_B = model().body_to_clique(body_b);
    const int c_A = model().body_to_clique(body_a);  // negative if anchored.
    const bool have_r_c_B = mapping.clique_subsequence.participates(c_B);
    const bool have_r_c_A =
        (c_A >= 0 && mapping.clique_subsequence.participates(c_A));
    const int r_num_cliques = have_r_c_B + have_r_c_A;
    if (r_num_cliques == 0) continue;

    // If only body A's clique survives, flip A/B to keep "B is dynamic". The
    // derived flips its own geometry so the constraint velocity vc stays the
    // same; g₀ negates or not per Derived::FlipNegatesG0().
    const bool need_flip = (r_num_cliques == 1 && have_r_c_A);
    if (need_flip) {
      reduced_pool->body_pairs_.emplace_back(body_b, body_a);
      if (Derived::FlipNegatesG0()) {
        reduced_pool->g0_.Add(N, 1) = -g0_[k];
      } else {
        reduced_pool->g0_.Add(N, 1) = g0_[k];
      }
    } else {
      reduced_pool->body_pairs_.push_back(body_pairs_[k]);
      reduced_pool->g0_.Add(N, 1) = g0_[k];
    }
    reduced_pool->stiffness_.push_back(stiffness_[k]);
    reduced_pool->damping_.push_back(damping_[k]);
    derived().ReduceGeometryInto(reduced_pool, k, need_flip);
  }
  const int reduced_size = ssize(reduced_pool->body_pairs_);
  reduced_pool->ResizeAllConstraints(reduced_size);

  // R, v̂, and the Hessian blocks are (re)computed by PrecomputeHessianBlocks();
  // just size them here.
  reduced_pool->R_.resize(reduced_size);
  reduced_pool->v_hat_.Resize(reduced_size, N, 1);
  reduced_pool->hessian_blocks_.resize(reduced_size);
}

template <typename T, int N, typename Derived>
void HolonomicConstraintsPool<T, N, Derived>::ResizeAllConstraints(
    int num_constraints) {
  all_constraints_.resize(num_constraints);
  std::iota(all_constraints_.begin(), all_constraints_.end(), 0);
}

// Explicit template instantiations.
template class HolonomicConstraintsPool<double, 6, WeldConstraintsPool<double>>;
template class HolonomicConstraintsPool<AutoDiffXd, 6,
                                        WeldConstraintsPool<AutoDiffXd>>;
template class HolonomicConstraintsPool<double, 3, BallConstraintsPool<double>>;
template class HolonomicConstraintsPool<AutoDiffXd, 3,
                                        BallConstraintsPool<AutoDiffXd>>;
template class HolonomicConstraintsPool<double, 1,
                                        DistanceConstraintsPool<double>>;
template class HolonomicConstraintsPool<AutoDiffXd, 1,
                                        DistanceConstraintsPool<AutoDiffXd>>;

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
