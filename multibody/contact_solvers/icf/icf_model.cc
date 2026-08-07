#include "drake/multibody/contact_solvers/icf/icf_model.h"

#include <span>
#include <utility>
#include <vector>

#include "drake/common/unused.h"
#include "drake/multibody/plant/slicing_and_indexing.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

using contact_solvers::internal::BlockSparseSymmetricMatrix;
using contact_solvers::internal::BlockSparsityPattern;
using Eigen::VectorBlock;
using multibody::internal::DemandIndicesValid;
using multibody::internal::SelectRows;
using multibody::internal::SelectRowsCols;

template <typename T>
IcfModel<T>::IcfModel()
    : params_{std::make_unique<IcfParameters<T>>()},
      ball_constraints_pool_(this),
      coupler_constraints_pool_(this),
      distance_constraints_pool_(this),
      gain_constraints_pool_(this),
      limit_constraints_pool_(this),
      patch_constraints_pool_(this),
      weld_constraints_pool_(this) {}

template <typename T>
void IcfModel<T>::ResetParameters(std::unique_ptr<IcfParameters<T>> params) {
  DRAKE_DEMAND(params != nullptr);
  params_ = std::move(params);

  // Set some sizes.
  num_velocities_ = v0().size();
  num_bodies_ = ssize(this->params().body_to_clique);
  num_cliques_ = ssize(this->params().clique_sizes);
  max_clique_size_ =
      num_cliques_ > 0 ? std::ranges::max(this->params().clique_sizes) : 0;
  clique_starts_.resize(num_cliques_ + 1);
  clique_starts_[0] = 0;
  std::partial_sum(this->params().clique_sizes.begin(),
                   this->params().clique_sizes.end(),
                   clique_starts_.begin() + 1);

  // Compute the initial spatial velocity V_WB0 = J_WB⋅v0 for each body.
  V_WB0_.Resize(num_bodies_, 6, 1);
  CalcBodySpatialVelocities(v0(), &V_WB0_);

  // Set the scaling factor diag(M)^{-1/2} for convergence checks.
  scale_factor_ = M0().diagonal().cwiseInverse().cwiseSqrt();

  // Define the sparse dynamics matrix A = M + δt D and the diagonal Delassus
  // operator estimate W = diag(M)⁻¹.
  const std::vector<int>& clique_sizes = this->params().clique_sizes;
  const int num_cliques = ssize(clique_sizes);
  A_.Resize(num_cliques, clique_sizes, clique_sizes);
  clique_diagonal_mass_inverse_.Resize(num_cliques, clique_sizes);
  for (int c = 0; c < num_cliques_; ++c) {
    const int v_start = clique_starts()[c];
    const int nv = clique_sizes[c];
    A_[c] = M0().block(v_start, v_start, nv, nv);
    A_[c].diagonal() += time_step() * D0().segment(v_start, nv);
    clique_diagonal_mass_inverse_[c] =
        M0().block(v_start, v_start, nv, nv).diagonal().cwiseInverse();
  }

  // Set the linear cost term r = A v₀ - δt k₀.
  Av0_.resize(num_velocities_);
  MultiplyByDynamicsMatrix(v0(), &Av0_);
  r_ = Av0_ - time_step() * k0();

  VerifyInvariants();

  // The old sparsity pattern is now invalid. It must be recomputed with
  // SetSparsityPattern().
  sparsity_pattern_ = nullptr;
}

template <typename T>
Eigen::VectorBlock<const VectorX<T>> IcfModel<T>::clique_segment(
    int clique, const VectorX<T>& full_vector) const {
  DRAKE_ASSERT(full_vector.size() == num_velocities_);
  return full_vector.segment(clique_starts()[clique],
                             params().clique_sizes[clique]);
}

template <typename T>
Eigen::VectorBlock<VectorX<T>> IcfModel<T>::mutable_clique_segment(
    int clique, VectorX<T>* full_vector) const {
  DRAKE_ASSERT(full_vector != nullptr);
  DRAKE_ASSERT(full_vector->size() == num_velocities_);
  return full_vector->segment(clique_starts()[clique],
                              params().clique_sizes[clique]);
}

template <typename T>
void IcfModel<T>::ResizeData(IcfData<T>* data) const {
  data->Resize(
      {.num_bodies = num_bodies_,
       .num_velocities = num_velocities_,
       .max_clique_size = max_clique_size_,
       .num_ball_constraints = ball_constraints_pool_.num_constraints(),
       .num_couplers = coupler_constraints_pool_.num_constraints(),
       .num_distance_constraints = distance_constraints_pool_.num_constraints(),
       .num_welds = weld_constraints_pool_.num_constraints(),
       .gain_sizes = gain_constraints_pool_.constraint_sizes(),
       .limit_sizes = limit_constraints_pool_.constraint_sizes(),
       .patch_sizes = patch_constraints_pool_.patch_sizes(),
       .num_islands = partition_.num_islands()});
}

template <typename T>
void IcfModel<T>::CalcData(const VectorX<T>& v, IcfData<T>* data) const {
  // Set generalized velocities v.
  data->set_v(v);

  // Set momentum cost (1/2 v'Av - r'v) and gradient (Av - r).
  CalcMomentumTerms(v, data);

  // Compute spatial velocities for all bodies.
  EigenPool<Vector6<T>>& V_WB = data->mutable_V_WB();
  CalcBodySpatialVelocities(v, &V_WB);

  // Compute constraint data.
  ball_constraints_pool_.CalcData(V_WB, &data->mutable_ball_constraints_data());
  coupler_constraints_pool_.CalcData(v,
                                     &data->mutable_coupler_constraints_data());
  distance_constraints_pool_.CalcData(
      V_WB, &data->mutable_distance_constraints_data());
  gain_constraints_pool_.CalcData(v, &data->mutable_gain_constraints_data());
  limit_constraints_pool_.CalcData(v, &data->mutable_limit_constraints_data());
  patch_constraints_pool_.CalcData(V_WB,
                                   &data->mutable_patch_constraints_data());
  weld_constraints_pool_.CalcData(V_WB, &data->mutable_weld_constraints_data());

  // Accumulate gradient contributions from constraints.
  VectorX<T>& gradient = data->mutable_gradient();
  ball_constraints_pool_.AccumulateGradient(*data, &gradient);
  coupler_constraints_pool_.AccumulateGradient(*data, &gradient);
  distance_constraints_pool_.AccumulateGradient(*data, &gradient);
  gain_constraints_pool_.AccumulateGradient(*data, &gradient);
  limit_constraints_pool_.AccumulateGradient(*data, &gradient);
  patch_constraints_pool_.AccumulateGradient(*data, &gradient);
  weld_constraints_pool_.AccumulateGradient(*data, &gradient);

  // Accumulate cost contributions from constraints.
  data->set_cost(data->momentum_cost() + data->ball_constraints_data().cost() +
                 data->coupler_constraints_data().cost() +
                 data->distance_constraints_data().cost() +
                 data->gain_constraints_data().cost() +
                 data->limit_constraints_data().cost() +
                 data->patch_constraints_data().cost() +
                 data->weld_constraints_data().cost());
}

template <typename T>
T IcfModel<T>::CalcData(const VectorX<T>& v, int island,
                        IcfData<T>* data) const {
  unused(v, island, data);
  DRAKE_UNREACHABLE();
  return {};
}

template <typename T>
std::unique_ptr<BlockSparseSymmetricMatrix<MatrixX<T>>>
IcfModel<T>::MakeHessian(const IcfData<T>& data) const {
  auto hessian = std::make_unique<BlockSparseSymmetricMatrix<MatrixX<T>>>(
      sparsity_pattern());
  UpdateHessian(data, hessian.get());
  return hessian;
}

template <typename T>
void IcfModel<T>::UpdateHessian(
    const IcfData<T>& data,
    BlockSparseSymmetricMatrix<MatrixX<T>>* hessian) const {
  hessian->SetZero();

  // Initialize hessian = A (block diagonal).
  for (int c = 0; c < num_cliques_; ++c) {
    ConstMatrixXView A_clique = A(c);
    hessian->AddToBlock(c, c, A_clique);
  }

  // Add constraints' contributions.
  ball_constraints_pool_.AccumulateHessian(data, hessian);
  coupler_constraints_pool_.AccumulateHessian(data, hessian);
  distance_constraints_pool_.AccumulateHessian(data, hessian);
  gain_constraints_pool_.AccumulateHessian(data, hessian);
  limit_constraints_pool_.AccumulateHessian(data, hessian);
  patch_constraints_pool_.AccumulateHessian(data, hessian);
  weld_constraints_pool_.AccumulateHessian(data, hessian);
}

template <typename T>
std::unique_ptr<BlockSparseSymmetricMatrix<MatrixX<T>>>
IcfModel<T>::MakeHessian(int island, const IcfData<T>& data) const {
  auto hessian = std::make_unique<BlockSparseSymmetricMatrix<MatrixX<T>>>(
      island_sparsity_pattern(island));
  UpdateHessian(island, data, hessian.get());
  return hessian;
}

template <typename T>
void IcfModel<T>::UpdateHessian(
    int island, const IcfData<T>& data,
    BlockSparseSymmetricMatrix<MatrixX<T>>* hessian) const {
  unused(island, data, hessian);
  DRAKE_UNREACHABLE();
}

template <typename T>
void IcfModel<T>::CalcSearchDirectionData(
    const IcfData<T>& data, const VectorX<T>& w,
    IcfSearchDirectionData<T>* search_direction_data) const {
  search_direction_data->w.resize(num_velocities_);
  search_direction_data->U.Resize(num_bodies_, 6, 1);

  // We'll use search_direction_data->w as scratch, to avoid memory allocation.
  VectorX<T>& temp = search_direction_data->w;
  MultiplyByDynamicsMatrix(w, &temp);  // temp = A⋅w

  const VectorX<T>& v = data.v();
  search_direction_data->a = temp.dot(w);       // a = ‖w‖²
  const T vAw = v.dot(temp);                    // vAw = vᵀ⋅A⋅w
  search_direction_data->b = vAw - w.dot(r());  // b = vᵀ⋅A⋅w - w⋅r
  search_direction_data->c = data.momentum_cost();

  // It is now safe to overwrite with the desired value.
  search_direction_data->w = w;

  // U = J⋅w.
  CalcBodySpatialVelocities(w, &search_direction_data->U);
}

template <typename T>
void IcfModel<T>::CalcSearchDirectionData(
    const IcfData<T>& data, const VectorX<T>& w, int island,
    IcfSearchDirectionData<T>* search_direction_data) const {
  search_direction_data->w.resize(num_velocities_);
  search_direction_data->U.Resize(num_bodies_, 6, 1);

  // Use search_direction_data->w as scratch for A⋅w over the island's cliques.
  VectorX<T>& temp = search_direction_data->w;
  MultiplyByDynamicsMatrix(w, island, &temp);  // temp = A⋅w (island segments).

  const VectorX<T>& v = data.v();
  const VectorX<T>& Av = data.Av();

  // a = wᵀ⋅A⋅w, b = vᵀ⋅A⋅w - wᵀ⋅r, c = ½vᵀ⋅A⋅v - rᵀ⋅v, all restricted to the
  // island's clique segments (the only segments where w is nonzero).
  T a = 0, vAw = 0, wr = 0, c = 0;
  for (int cl : partition_.island_cliques(island)) {
    const auto w_c = clique_segment(cl, w);
    const auto Aw_c = clique_segment(cl, temp);
    const auto v_c = clique_segment(cl, v);
    const auto Av_c = clique_segment(cl, Av);
    const auto r_c = clique_segment(cl, r_);
    a += w_c.dot(Aw_c);
    vAw += v_c.dot(Aw_c);
    wr += w_c.dot(r_c);
    c += v_c.dot(0.5 * Av_c - r_c);
  }
  search_direction_data->a = a;
  search_direction_data->b = vAw - wr;
  search_direction_data->c = c;

  // It is now safe to overwrite the scratch with the desired value.
  search_direction_data->w = w;

  // U = J⋅w over the island's bodies.
  CalcBodySpatialVelocities(w, island, &search_direction_data->U);
}

template <typename T>
T IcfModel<T>::CalcCostAlongLine(
    const T& alpha, const IcfData<T>& data,
    const IcfSearchDirectionData<T>& search_direction, T* dcost_dalpha,
    T* d2cost_dalpha2) const {
  const T& a = search_direction.a;
  const T& b = search_direction.b;
  const T& c = search_direction.c;

  VectorXView v_alpha = data.scratch().v_alpha[0];
  DRAKE_ASSERT(v_alpha.size() == num_velocities_);
  v_alpha.noalias() = data.v() + alpha * search_direction.w;

  // Compute momentum contributions:
  T cost = (0.5 * a * alpha + b) * alpha + c;  // = aα²/2 + bα + c
  *dcost_dalpha = a * alpha + b;
  *d2cost_dalpha2 = a;

  // Add coupler constraints contributions:
  {
    T constraint_dcost, constraint_d2cost;

    coupler_constraints_pool_.CalcData(
        v_alpha, &data.scratch().coupler_constraints_data);
    coupler_constraints_pool_.CalcCostAlongLine(
        data.scratch().coupler_constraints_data, search_direction.w,
        &constraint_dcost, &constraint_d2cost);

    cost += data.scratch().coupler_constraints_data.cost();
    *dcost_dalpha += constraint_dcost;
    *d2cost_dalpha2 += constraint_d2cost;
  }

  // Add gain constraints contributions:
  {
    T constraint_dcost, constraint_d2cost;

    gain_constraints_pool_.CalcData(v_alpha,
                                    &data.scratch().gain_constraints_data);
    gain_constraints_pool_.CalcCostAlongLine(
        data.scratch().gain_constraints_data, search_direction.w,
        &data.scratch().Gw_gain, &constraint_dcost, &constraint_d2cost);

    cost += data.scratch().gain_constraints_data.cost();
    *dcost_dalpha += constraint_dcost;
    *d2cost_dalpha2 += constraint_d2cost;
  }

  // Add limit constraints contributions:
  {
    T constraint_dcost, constraint_d2cost;

    limit_constraints_pool_.CalcData(v_alpha,
                                     &data.scratch().limit_constraints_data);
    limit_constraints_pool_.CalcCostAlongLine(
        data.scratch().limit_constraints_data, search_direction.w,
        &data.scratch().Gw_limit, &constraint_dcost, &constraint_d2cost);

    cost += data.scratch().limit_constraints_data.cost();
    *dcost_dalpha += constraint_dcost;
    *d2cost_dalpha2 += constraint_d2cost;
  }

  // Add patch constraints contributions:
  {
    T constraint_dcost, constraint_d2cost;

    EigenPool<Vector6<T>>& V_WB_alpha = data.scratch().V_WB_alpha;
    DRAKE_ASSERT(V_WB_alpha.size() == num_bodies_);

    CalcBodySpatialVelocities(v_alpha, &V_WB_alpha);
    patch_constraints_pool_.CalcData(V_WB_alpha,
                                     &data.scratch().patch_constraints_data);
    patch_constraints_pool_.CalcCostAlongLine(
        data.scratch().patch_constraints_data, search_direction.U,
        &data.scratch().U_AbB_W, &constraint_dcost, &constraint_d2cost);

    cost += data.scratch().patch_constraints_data.cost();
    *dcost_dalpha += constraint_dcost;
    *d2cost_dalpha2 += constraint_d2cost;
  }

  // Add ball constraints contributions:
  {
    T constraint_dcost, constraint_d2cost;

    EigenPool<Vector6<T>>& V_WB_alpha = data.scratch().V_WB_alpha;
    // N.B. V_WB_alpha was already computed above for patch constraints.

    ball_constraints_pool_.CalcData(V_WB_alpha,
                                    &data.scratch().ball_constraints_data);
    ball_constraints_pool_.CalcCostAlongLine(
        data.scratch().ball_constraints_data, search_direction.U,
        &constraint_dcost, &constraint_d2cost);

    cost += data.scratch().ball_constraints_data.cost();
    *dcost_dalpha += constraint_dcost;
    *d2cost_dalpha2 += constraint_d2cost;
  }

  // Add distance constraints contributions:
  {
    T constraint_dcost, constraint_d2cost;

    EigenPool<Vector6<T>>& V_WB_alpha = data.scratch().V_WB_alpha;
    // N.B. V_WB_alpha was already computed above for patch constraints.

    distance_constraints_pool_.CalcData(
        V_WB_alpha, &data.scratch().distance_constraints_data);
    distance_constraints_pool_.CalcCostAlongLine(
        data.scratch().distance_constraints_data, search_direction.U,
        &constraint_dcost, &constraint_d2cost);

    cost += data.scratch().distance_constraints_data.cost();
    *dcost_dalpha += constraint_dcost;
    *d2cost_dalpha2 += constraint_d2cost;
  }

  // Add weld constraints contributions:
  {
    T constraint_dcost, constraint_d2cost;

    EigenPool<Vector6<T>>& V_WB_alpha = data.scratch().V_WB_alpha;
    // N.B. V_WB_alpha was already computed above for patch constraints.

    weld_constraints_pool_.CalcData(V_WB_alpha,
                                    &data.scratch().weld_constraints_data);
    weld_constraints_pool_.CalcCostAlongLine(
        data.scratch().weld_constraints_data, search_direction.U,
        &constraint_dcost, &constraint_d2cost);

    cost += data.scratch().weld_constraints_data.cost();
    *dcost_dalpha += constraint_dcost;
    *d2cost_dalpha2 += constraint_d2cost;
  }

  return cost;
}

// TODO(#23912): Try removing allocations since this will now be used in the hot
// path by joint locking.
template <typename T>
T IcfModel<T>::CalcCostAlongLine(
    const T& alpha, const IcfData<T>& data,
    const IcfSearchDirectionData<T>& search_direction, int island,
    T* dcost_dalpha, T* d2cost_dalpha2) const {
  unused(alpha, data, search_direction, island, dcost_dalpha, d2cost_dalpha2);
  DRAKE_UNREACHABLE();
  return {};
}

template <typename T>
void IcfModel<T>::SetSparsityPattern() {
  DRAKE_DEMAND(params_ != nullptr);

  // N.B. we make a copy here because block_sizes will be moved into the
  // BlockSparsityPattern at the end of this function.
  // TODO(#23912): This line allocates.
  std::vector<int> block_sizes = params().clique_sizes;

  // Build diagonal entries in the sparsity pattern.
  // TODO(#23912): This line allocates.
  std::vector<std::vector<int>> sparsity(num_cliques_);
  for (int c = 0; c < num_cliques_; ++c) {
    // TODO(#23912): This line allocates.
    sparsity[c].emplace_back(c);
  }

  // Build off-diagonal entries in the sparsity pattern.
  ball_constraints_pool_.CalcSparsityPattern(&sparsity);
  distance_constraints_pool_.CalcSparsityPattern(&sparsity);
  patch_constraints_pool_.CalcSparsityPattern(&sparsity);
  weld_constraints_pool_.CalcSparsityPattern(&sparsity);

  // Precompute the iteration-invariant ball, distance, and weld Hessian blocks
  // now that all constraint data and model parameters are finalized.
  ball_constraints_pool_.PrecomputeHessianBlocks();
  distance_constraints_pool_.PrecomputeHessianBlocks();
  weld_constraints_pool_.PrecomputeHessianBlocks();

  // TODO(#23912): This line allocates.
  sparsity_pattern_ = std::make_unique<BlockSparsityPattern>(
      std::move(block_sizes), std::move(sparsity));

  // Partition cliques into islands (connected components) for per-island
  // solves, then group constraints and bodies by island.
  partition_.Compute(*sparsity_pattern_);
  BuildIslandMaps();
  BuildIslandSparsityPatterns();
}

template <typename T>
void IcfModel<T>::BuildIslandSparsityPatterns() {
  const int num_islands = partition_.num_islands();
  const std::vector<std::vector<int>>& global_neighbors =
      sparsity_pattern_->neighbors();
  const std::vector<int>& clique_sizes = params().clique_sizes;

  // Reuse one set of pattern objects across calls, but rebuild the contents
  // each time the sparsity changes (the SetSparsityPattern() cadence).
  island_sparsity_patterns_.clear();
  island_sparsity_patterns_.reserve(num_islands);

  for (int i = 0; i < num_islands; ++i) {
    const std::span<const int> cliques = partition_.island_cliques(i);
    const int n = ssize(cliques);

    // Local block sizes follow the island's (ascending) clique order.
    island_block_sizes_.assign(n, 0);
    for (int l = 0; l < n; ++l)
      island_block_sizes_[l] = clique_sizes[cliques[l]];

    // Local neighbors: map each global neighbor of clique cliques[l] to its
    // island-local index. All neighbors lie in the same island (islands are
    // connected components), and local indexing preserves global ordering, so
    // the lower-triangular (j >= i) invariant carries over.
    island_neighbors_.resize(n);
    for (int l = 0; l < n; ++l) {
      std::vector<int>& local = island_neighbors_[l];
      local.clear();
      local.reserve(ssize(global_neighbors[cliques[l]]));
      for (int g : global_neighbors[cliques[l]]) {
        local.push_back(partition_.clique_local_index(g));
      }
    }

    std::vector<int> block_sizes(island_block_sizes_.begin(),
                                 island_block_sizes_.begin() + n);
    std::vector<std::vector<int>> neighbors(island_neighbors_.begin(),
                                            island_neighbors_.begin() + n);
    island_sparsity_patterns_.emplace_back(std::move(block_sizes),
                                           std::move(neighbors));
  }
}

template <typename T>
void IcfModel<T>::BuildIslandMaps() {
  const int num_islands = partition_.num_islands();

  // Builds `map` so that map.items(i) lists the indices k in [0, n) whose
  // key_fn(k) is island i. A negative key omits item k (e.g., anchored bodies).
  std::vector<int>& keys = island_keys_;
  auto build = [&](IslandItemMap* map, int n, auto&& key_fn) {
    keys.resize(n);
    for (int k = 0; k < n; ++k) keys[k] = key_fn(k);
    map->Build(num_islands, std::span<const int>(keys.data(), n));
  };

  // The island of a (single-clique) constraint, or of a constraint between two
  // bodies (at least one of which is dynamic; both share an island).
  auto island_of_clique = [&](int clique) {
    return partition_.clique_to_island(clique);
  };
  auto island_of_bodies = [&](int b0, int b1) {
    const int c0 = body_to_clique(b0);
    const int clique = (c0 >= 0) ? c0 : body_to_clique(b1);
    DRAKE_ASSERT(clique >= 0);
    return island_of_clique(clique);
  };

  const std::vector<std::pair<int, int>>& ball_bodies =
      ball_constraints_pool_.body_pairs();
  build(&island_balls_, ssize(ball_bodies), [&](int k) {
    return island_of_bodies(ball_bodies[k].first, ball_bodies[k].second);
  });

  const std::vector<int>& coupler_clique =
      coupler_constraints_pool_.constraint_to_clique();
  build(&island_couplers_, ssize(coupler_clique), [&](int k) {
    return island_of_clique(coupler_clique[k]);
  });

  const std::vector<std::pair<int, int>>& distance_bodies =
      distance_constraints_pool_.body_pairs();
  build(&island_distances_, ssize(distance_bodies), [&](int k) {
    return island_of_bodies(distance_bodies[k].first,
                            distance_bodies[k].second);
  });

  const std::vector<int>& gain_clique = gain_constraints_pool_.clique();
  build(&island_gains_, ssize(gain_clique), [&](int k) {
    return island_of_clique(gain_clique[k]);
  });

  const std::vector<int>& limit_clique = limit_constraints_pool_.clique();
  build(&island_limits_, ssize(limit_clique), [&](int k) {
    return island_of_clique(limit_clique[k]);
  });

  const std::vector<std::pair<int, int>>& patch_bodies =
      patch_constraints_pool_.bodies();
  build(&island_patches_, ssize(patch_bodies), [&](int k) {
    return island_of_bodies(patch_bodies[k].first, patch_bodies[k].second);
  });

  const std::vector<std::pair<int, int>>& weld_bodies =
      weld_constraints_pool_.body_pairs();
  build(&island_welds_, ssize(weld_bodies), [&](int k) {
    return island_of_bodies(weld_bodies[k].first, weld_bodies[k].second);
  });

  // Dynamic bodies only; anchored bodies (negative clique) belong to no island.
  build(&island_bodies_, num_bodies_, [&](int b) {
    return is_anchored(b) ? -1 : island_of_clique(body_to_clique(b));
  });
}

template <typename T>
void IcfModel<T>::UpdateTimeStep(const T& time_step) {
  DRAKE_DEMAND(time_step > 0);
  DRAKE_DEMAND(params_ != nullptr);

  // Linearized dynamics matrix A = M + δt⋅D
  for (int c = 0; c < num_cliques_; ++c) {
    const int v_start = clique_starts()[c];
    const int nv = params().clique_sizes[c];
    A_[c] = M0().block(v_start, v_start, nv, nv);
    A_[c].diagonal() += time_step * D0().segment(v_start, nv);
    clique_diagonal_mass_inverse_[c] =
        M0().block(v_start, v_start, nv, nv).diagonal().cwiseInverse();
  }

  // Linear cost term r = A v₀ - δt k₀
  MultiplyByDynamicsMatrix(v0(), &Av0_);
  r_ = Av0_ - time_step * k0();

  params_->time_step = time_step;

  // Ball, distance, and weld constraint regularization R depends on the time
  // step. Recompute Hessian blocks whenever dt changes so AccumulateHessian()
  // uses up-to-date values.
  ball_constraints_pool_.PrecomputeHessianBlocks();
  distance_constraints_pool_.PrecomputeHessianBlocks();
  weld_constraints_pool_.PrecomputeHessianBlocks();
}

template <typename T>
void IcfModel<T>::ReduceInto(IcfModel<T>* reduced_model,
                             ReducedMapping* mapping) const {
  DRAKE_DEMAND(reduced_model != nullptr);
  DRAKE_DEMAND(mapping != nullptr);
  const auto& full_params = params();
  const auto& unlocked_dofs = full_params.reduction.unlocked_dofs;

  mapping->velocity_subsequence.ResetToSize(num_velocities());
  for (const int unlocked : unlocked_dofs) {
    mapping->velocity_subsequence.push(unlocked);
  }

  auto reduced_params = reduced_model->ReleaseParameters();

  // Reduce a bunch of easy params.
  reduced_params->time_step = full_params.time_step;
  reduced_params->v0 = SelectRows(full_params.v0, unlocked_dofs);
  reduced_params->M0 = SelectRowsCols(full_params.M0, unlocked_dofs);
  reduced_params->D0 = SelectRows(full_params.D0, unlocked_dofs);
  reduced_params->k0 = SelectRows(full_params.k0, unlocked_dofs);
  reduced_params->body_mass = full_params.body_mass;
  reduced_params->body_is_floating = full_params.body_is_floating;

  // Map all cliques possibly having unlocked DoFs.
  mapping->clique_subsequence.ResetToSize(num_cliques());
  mapping->clique_dof_subsequences.resize(num_cliques());
  const auto& per_clique_unlocked_dofs =
      full_params.reduction.per_clique_unlocked_dofs;
  reduced_params->clique_sizes.clear();
  DRAKE_DEMAND(num_cliques() == ssize(per_clique_unlocked_dofs));
  for (int c = 0; c < num_cliques(); ++c) {
    // Clique participates if at least one of its dofs is not locked.
    int clique_unlocked_count = ssize(per_clique_unlocked_dofs[c]);
    if (clique_unlocked_count > 0) {
      reduced_params->clique_sizes.push_back(clique_unlocked_count);
      mapping->clique_subsequence.push(c);
    }
    mapping->clique_dof_subsequences[c].ResetToSize(
        full_params.clique_sizes[c]);
    for (const int& per_clique_unlocked : per_clique_unlocked_dofs[c]) {
      mapping->clique_dof_subsequences[c].push(per_clique_unlocked);
    }
  }

  // Reduce per-body params.
  reduced_params->body_to_clique.resize(num_bodies());
  reduced_params->J_WB.Clear();
  for (int b = 0; b < num_bodies(); ++b) {
    constexpr int kRows =
        decltype(full_params.J_WB)::MatrixView::RowsAtCompileTime;
    const int full_clique = full_params.body_to_clique[b];

    if (is_anchored(b) ||
        !mapping->clique_subsequence.participates(full_clique)) {
      // This body will not participate in the reduced problem.
      reduced_params->body_to_clique[b] = -1;
      reduced_params->J_WB.Add(kRows, 0);
    } else {
      const int reduced_clique =
          mapping->clique_subsequence.permuted_index(full_clique);
      reduced_params->body_to_clique[b] = reduced_clique;
      const int reduced_cols = ssize(per_clique_unlocked_dofs[full_clique]);
      if (is_floating(b)) {
        // Floating/free body should be either all locked or all unlocked.
        DRAKE_DEMAND(reduced_cols == full_params.clique_sizes[full_clique] ||
                     reduced_cols == 0);
      }
      reduced_params->J_WB.Add(kRows, reduced_cols);
      for (int q = 0; q < reduced_cols; ++q) {
        reduced_params->J_WB[b].col(q) =
            full_params.J_WB[b].col(per_clique_unlocked_dofs[full_clique][q]);
      }
    }
  }

  // Initialize the reduction params of the reduced model for no further
  // reduction.
  auto set_full_indices = [](int size, std::vector<int>* dofs) {
    dofs->resize(size);
    std::iota(dofs->begin(), dofs->end(), 0);
  };
  set_full_indices(reduced_params->v0.size(),
                   &reduced_params->reduction.unlocked_dofs);
  int nc = ssize(reduced_params->clique_sizes);
  reduced_params->reduction.per_clique_unlocked_dofs.resize(nc);
  for (int c = 0; c < nc; ++c) {
    set_full_indices(reduced_params->clique_sizes[c],
                     &reduced_params->reduction.per_clique_unlocked_dofs[c]);
  }

  // Bring the reduced model to basic sanity.
  reduced_model->ResetParameters(std::move(reduced_params));

  // Reduce the constraints.
  ball_constraints_pool().ReduceInto(*mapping,
                                     &reduced_model->ball_constraints_pool());
  coupler_constraints_pool().ReduceInto(
      *mapping, &reduced_model->coupler_constraints_pool());
  distance_constraints_pool().ReduceInto(
      *mapping, &reduced_model->distance_constraints_pool());
  gain_constraints_pool().ReduceInto(*mapping,
                                     &reduced_model->gain_constraints_pool());
  limit_constraints_pool().ReduceInto(*mapping,
                                      &reduced_model->limit_constraints_pool());
  patch_constraints_pool().ReduceInto(*mapping,
                                      &reduced_model->patch_constraints_pool());
  weld_constraints_pool().ReduceInto(*mapping,
                                     &reduced_model->weld_constraints_pool());

  // Refuse multiple levels of reduction.
  DRAKE_DEMAND(!reduced_model->is_reducible());
}

template <typename T>
void IcfModel<T>::VerifyInvariants() const {
  DRAKE_DEMAND(params_ != nullptr);
  DRAKE_DEMAND(time_step() > 0);

  DRAKE_DEMAND(num_bodies_ > 0);
  DRAKE_DEMAND(num_velocities_ >= 0);
  DRAKE_DEMAND(num_cliques_ >= 0);
  DRAKE_DEMAND(max_clique_size_ >= 0);

  DRAKE_DEMAND(v0().size() == num_velocities_);
  DRAKE_DEMAND(M0().rows() == num_velocities_);
  DRAKE_DEMAND(M0().cols() == num_velocities_);
  DRAKE_DEMAND(D0().size() == num_velocities_);
  DRAKE_DEMAND(k0().size() == num_velocities_);
  DRAKE_DEMAND(scale_factor().size() == num_velocities_);

  DRAKE_DEMAND(params().J_WB.size() == num_bodies_);
  DRAKE_DEMAND(V_WB0_.size() == num_bodies_);
  DRAKE_DEMAND(ssize(params().body_mass) == num_bodies_);

  DRAKE_DEMAND(A_.size() == num_cliques_);
  DRAKE_DEMAND(r_.size() == num_velocities_);
  DRAKE_DEMAND(Av0_.size() == num_velocities_);
  DRAKE_DEMAND(clique_diagonal_mass_inverse_.size() == num_cliques_);

  DRAKE_DEMAND(ssize(clique_starts()) == num_cliques_ + 1);
  DRAKE_DEMAND(ssize(params().clique_sizes) == num_cliques_);
  DRAKE_DEMAND(ssize(params().body_to_clique) == num_bodies_);
  DRAKE_DEMAND(ssize(params().body_is_floating) == num_bodies_);

  for (int b = 0; b < num_bodies_; ++b) {
    const int c = body_to_clique(b);
    DRAKE_DEMAND(c < num_cliques_);
    if (c >= 0) {
      DRAKE_DEMAND(J_WB(b).cols() == A(c).rows());
    }
  }

  int nv = 0;
  for (int c = 0; c < num_cliques_; ++c) {
    const int clique_nv = clique_size(c);
    DRAKE_DEMAND(A(c).rows() == clique_nv);
    DRAKE_DEMAND(A(c).cols() == clique_nv);
    nv += clique_nv;
  }
  DRAKE_DEMAND(nv == num_velocities_);

  const auto& reduction = params().reduction;
  DemandIndicesValid(reduction.unlocked_dofs, num_velocities_);
  DRAKE_DEMAND(ssize(reduction.per_clique_unlocked_dofs) == num_cliques_);
  nv = 0;
  for (int c = 0; c < num_cliques_; ++c) {
    const auto& unlocked = reduction.per_clique_unlocked_dofs[c];
    DemandIndicesValid(unlocked, clique_size(c));
    nv += ssize(unlocked);
  }
  DRAKE_DEMAND(nv == ssize(reduction.unlocked_dofs));
}

template <typename T>
void IcfModel<T>::MultiplyByDynamicsMatrix(const VectorX<T>& v,
                                           VectorX<T>* result) const {
  DRAKE_ASSERT(v.size() == num_velocities_);
  DRAKE_ASSERT(result != nullptr);
  DRAKE_ASSERT(result->size() == num_velocities_);

  for (int c = 0; c < num_cliques_; ++c) {
    ConstMatrixXView A_clique = A(c);
    VectorBlock<const VectorX<T>> v_clique = clique_segment(c, v);
    VectorBlock<VectorX<T>> Av_clique = mutable_clique_segment(c, result);
    Av_clique.noalias() = A_clique * v_clique;  // Required to avoid allocation!
  }
}

template <typename T>
void IcfModel<T>::MultiplyByDynamicsMatrix(const VectorX<T>& v, int island,
                                           VectorX<T>* result) const {
  DRAKE_ASSERT(v.size() == num_velocities_);
  DRAKE_ASSERT(result != nullptr);
  DRAKE_ASSERT(result->size() == num_velocities_);

  for (int c : partition_.island_cliques(island)) {
    ConstMatrixXView A_clique = A(c);
    VectorBlock<const VectorX<T>> v_clique = clique_segment(c, v);
    VectorBlock<VectorX<T>> Av_clique = mutable_clique_segment(c, result);
    Av_clique.noalias() = A_clique * v_clique;  // Required to avoid allocation!
  }
}

template <typename T>
void IcfModel<T>::CalcMomentumTerms(const VectorX<T>& v,
                                    IcfData<T>* data) const {
  DRAKE_ASSERT(v.size() == num_velocities_);
  VectorX<T>& Av = data->mutable_Av();
  VectorXView Av_minus_r = data->scratch().Av_minus_r[0];
  DRAKE_ASSERT(Av_minus_r.size() == num_velocities_);

  // Cost.
  MultiplyByDynamicsMatrix(v, &Av);
  Av_minus_r = 0.5 * Av - r_;
  data->set_momentum_cost(v.dot(Av_minus_r));

  // Gradient.
  data->mutable_gradient() = Av - r_;
}

template <typename T>
T IcfModel<T>::CalcMomentumTerms(const VectorX<T>& v, int island,
                                 IcfData<T>* data) const {
  DRAKE_ASSERT(v.size() == num_velocities_);
  VectorX<T>& Av = data->mutable_Av();
  VectorX<T>& gradient = data->mutable_gradient();

  T momentum_cost = 0;
  for (int c : partition_.island_cliques(island)) {
    ConstMatrixXView A_clique = A(c);
    const auto v_c = clique_segment(c, v);
    const auto r_c = clique_segment(c, r_);
    auto Av_c = mutable_clique_segment(c, &Av);
    Av_c.noalias() = A_clique * v_c;  // Required to avoid allocation!
    // Cost ½vᵀAv - rᵀv and gradient Av - r, restricted to this clique.
    momentum_cost += v_c.dot(0.5 * Av_c - r_c);
    mutable_clique_segment(c, &gradient) = Av_c - r_c;
  }
  return momentum_cost;
}

template <typename T>
void IcfModel<T>::CalcBodySpatialVelocities(
    const VectorX<T>& v, EigenPool<Vector6<T>>* V_pool) const {
  EigenPool<Vector6<T>>& spatial_velocities = *V_pool;
  DRAKE_ASSERT(v.size() == num_velocities_);
  DRAKE_ASSERT(spatial_velocities.size() == num_bodies_);
  for (int b = 0; b < num_bodies_; ++b) {
    const int c = body_to_clique(b);
    Vector6<T>& V_WB = spatial_velocities[b];
    if (c >= 0) {
      VectorBlock<const VectorX<T>> v_clique = clique_segment(c, v);
      if (is_floating(b)) {
        V_WB = v_clique;
      } else {
        V_WB = J_WB(b) * v_clique;
      }
    } else {
      V_WB.setZero();  // Anchored body.
    }
  }
}

template <typename T>
void IcfModel<T>::CalcBodySpatialVelocities(
    const VectorX<T>& v, int island, EigenPool<Vector6<T>>* V_pool) const {
  EigenPool<Vector6<T>>& spatial_velocities = *V_pool;
  DRAKE_ASSERT(v.size() == num_velocities_);
  DRAKE_ASSERT(spatial_velocities.size() == num_bodies_);
  // island_bodies() lists only dynamic bodies (anchored bodies belong to no
  // island), so every body here has a valid clique.
  for (int b : island_bodies_.items(island)) {
    const int c = body_to_clique(b);
    DRAKE_ASSERT(c >= 0);
    Vector6<T>& V_WB = spatial_velocities[b];
    VectorBlock<const VectorX<T>> v_clique = clique_segment(c, v);
    if (is_floating(b)) {
      V_WB = v_clique;
    } else {
      V_WB = J_WB(b) * v_clique;
    }
  }
}

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::IcfModel);
