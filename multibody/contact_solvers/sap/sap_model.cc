#include "drake/multibody/contact_solvers/sap/sap_model.h"

#include <memory>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/math/linear_solve.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/block_sparse_supernodal_solver.h"
#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"
#include "drake/multibody/contact_solvers/sap/dense_supernodal_solver.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

using systems::Context;

HessianFactorizationCache::HessianFactorizationCache(
    SapHessianFactorizationType type, const std::vector<MatrixX<double>>* A,
    const BlockSparseMatrix<double>* J) {
  DRAKE_DEMAND(A != nullptr);
  DRAKE_DEMAND(J != nullptr);
  switch (type) {
    case SapHessianFactorizationType::kBlockSparseCholesky:
      factorization_ = std::make_unique<BlockSparseSuperNodalSolver>(*A, *J);
      break;
    case SapHessianFactorizationType::kDense:
      factorization_ = std::make_unique<DenseSuperNodalSolver>(A, J);
      break;
  }
}

std::unique_ptr<HessianFactorizationCache> HessianFactorizationCache::Clone()
    const {
  throw std::runtime_error(
      "Attempting to clone an expensive Hessian factorization.");
}

void HessianFactorizationCache::UpdateWeightMatrixAndFactor(
    const std::vector<MatrixX<double>>& G) {
  DRAKE_DEMAND(!is_empty());
  mutable_factorization()->SetWeightMatrix(G);
  mutable_factorization()->Factor();
}

void HessianFactorizationCache::SolveInPlace(
    EigenPtr<MatrixX<double>> rhs) const {
  DRAKE_DEMAND(!is_empty());
  // TODO(amcastro-tri): SuperNodalSolver should provide this in-place signature
  // for multiple RHSs.
  const int num_rhs = rhs->cols();
  for (int i = 0; i < num_rhs; ++i) {
    auto rhs_i = rhs->col(i);
    // N.B. Unfortunately SolveInPlace() only accepts VectorXd*, while here the
    // return of col() is a block object. Therefore, we are forced to make a
    // copy.
    // TODO(amcastro-tri): Update SuperNodalSolver::SolveInPlace() to accept
    // EigenPtr instead.
    rhs_i = factorization()->Solve(rhs_i);
  }
}

template <typename T>
SapModel<T>::SapModel(const SapContactProblem<T>* problem_ptr,
                      SapHessianFactorizationType hessian_type)
    : problem_(problem_ptr), hessian_type_(hessian_type) {
  // Graph to the original contact problem, including all cliques
  // (participating and non-participating).
  const ContactProblemGraph& graph = problem().graph();

  // Permutations to map indexes from participating cliques/dofs to the original
  // set of cliques/dofs.
  const PartialPermutation& cliques_permutation = graph.participating_cliques();
  PartialPermutation velocities_permutation =
      MakeParticipatingVelocitiesPermutation(problem());
  PartialPermutation impulses_permutation = MakeImpulsesPermutation(graph);

  // Extract momentum matrix for participating DOFs only.
  const int num_participating_cliques =
      cliques_permutation.permuted_domain_size();
  std::vector<MatrixX<T>> dynamics_matrix(num_participating_cliques);
  cliques_permutation.Apply(problem().dynamics_matrix(), &dynamics_matrix);

  // Get v* for participating DOFs only.
  const int nv_participating = velocities_permutation.permuted_domain_size();
  VectorX<T> v_star(nv_participating);
  velocities_permutation.Apply(problem().v_star(), &v_star);

  // Compute diagonal scaling inv_sqrt_A.
  VectorX<T> inv_sqrt_A(nv_participating);
  int clique_offset = 0;
  for (const auto& Ac : dynamics_matrix) {
    const int clique_nv = Ac.rows();
    inv_sqrt_A.segment(clique_offset, clique_nv) =
        Ac.diagonal().cwiseInverse().cwiseSqrt();
    clique_offset += clique_nv;
  }

  // Computation of a diagonal approximation to the Delassus operator.
  VectorX<T> delassus_diagonal(num_constraint_equations());
  CalcDelassusDiagonalApproximation(dynamics_matrix, &delassus_diagonal);

  // Create constraints bundle.
  std::unique_ptr<SapConstraintBundle<T>> constraints_bundle =
      std::make_unique<SapConstraintBundle<T>>(&problem(), delassus_diagonal);

  // N.B. const_model_data_ is meant to be created once at construction and
  // remain const afterwards.
  const_model_data_.velocities_permutation = std::move(velocities_permutation);
  const_model_data_.impulses_permutation = std::move(impulses_permutation);
  const_model_data_.dynamics_matrix = std::move(dynamics_matrix);
  const_model_data_.constraints_bundle = std::move(constraints_bundle);
  // N.B. We must extract p* after the dynamics matrix has been moved into
  // const_model_data_.
  VectorX<T> p_star(nv_participating);
  MultiplyByDynamicsMatrix(v_star, &p_star);
  const_model_data_.v_star = std::move(v_star);
  const_model_data_.p_star = std::move(p_star);
  const_model_data_.inv_sqrt_A = std::move(inv_sqrt_A);
  const_model_data_.delassus_diagonal = std::move(delassus_diagonal);

  system_ = std::make_unique<SapModelSystem>(num_velocities());
  DeclareCacheEntries();
}

template <typename T>
void SapModel<T>::DeclareCacheEntries() {
  // Sanity check a system to manage cache resources has been created.
  DRAKE_DEMAND(system_ != nullptr);

  const auto& constraint_velocities_cache_entry = system_->DeclareCacheEntry(
      "Constraint velocities, vc = J⋅v.",
      systems::ValueProducer(this, &SapModel<T>::CalcConstraintVelocities),
      {systems::System<T>::xd_ticket()});
  system_->mutable_cache_indexes().constraint_velocities =
      constraint_velocities_cache_entry.cache_index();

  const T& dt = this->time_step();
  const auto& w = this->const_model_data_.delassus_diagonal;
  SapConstraintBundleDataCache bundle_cache_model;
  bundle_cache_model.bundle_data = this->constraints_bundle().MakeData(dt, w);
  const auto& bundle_data_cache_entry = system_->DeclareCacheEntry(
      "Constraint bundle data.",
      systems::ValueProducer(this, bundle_cache_model,
                             &SapModel<T>::CalcConstraintBundleDataCache),
      {system_->cache_entry_ticket(
          system_->cache_indexes().constraint_velocities)});
  system_->mutable_cache_indexes().bundle_data =
      bundle_data_cache_entry.cache_index();

  const auto& impulses_cache_entry = system_->DeclareCacheEntry(
      "Impulses, γ = P(−R⁻¹(vc − v̂)).",
      systems::ValueProducer(this, &SapModel<T>::CalcImpulsesCache),
      {system_->cache_entry_ticket(
          system_->cache_indexes().constraint_velocities)});
  system_->mutable_cache_indexes().impulses =
      impulses_cache_entry.cache_index();

  const auto& momentum_gain_cache_entry = system_->DeclareCacheEntry(
      "Momentum gain. Δv = v - v*, Δp = A⋅Δv.",
      systems::ValueProducer(this, &SapModel<T>::CalcMomentumGainCache),
      {systems::System<T>::xd_ticket()});
  system_->mutable_cache_indexes().momentum_gain =
      momentum_gain_cache_entry.cache_index();

  const auto& cost_cache_entry = system_->DeclareCacheEntry(
      "Cost cache.", systems::ValueProducer(this, &SapModel<T>::CalcCostCache),
      {system_->cache_entry_ticket(system_->cache_indexes().momentum_gain),
       system_->cache_entry_ticket(system_->cache_indexes().impulses)});
  system_->mutable_cache_indexes().cost = cost_cache_entry.cache_index();

  const auto& gradients_cache_entry = system_->DeclareCacheEntry(
      "Gradients cache.",
      systems::ValueProducer(this, &SapModel<T>::CalcGradientsCache),
      {system_->cache_entry_ticket(system_->cache_indexes().momentum_gain),
       system_->cache_entry_ticket(system_->cache_indexes().impulses)});
  system_->mutable_cache_indexes().gradients =
      gradients_cache_entry.cache_index();

  const auto& hessian_cache_entry = system_->DeclareCacheEntry(
      "Hessian cache.",
      systems::ValueProducer(this, &SapModel<T>::CalcHessianCache),
      {system_->cache_entry_ticket(
          system_->cache_indexes().constraint_velocities)});
  system_->mutable_cache_indexes().hessian = hessian_cache_entry.cache_index();

  // N.B. The default constructible SapHessianFactorizationCache is empty. Since
  // the computation of the factorization is costly, we delay its construction
  // to the very first time it is computed.
  const auto& hessian_factorization_cache_entry = system_->DeclareCacheEntry(
      "Hessian factorization cache.",
      systems::ValueProducer(this, &SapModel<T>::CalcHessianFactorizationCache),
      {system_->cache_entry_ticket(
          system_->cache_indexes().constraint_velocities)});
  system_->mutable_cache_indexes().hessian_factorization =
      hessian_factorization_cache_entry.cache_index();
}

template <typename T>
std::unique_ptr<systems::Context<T>> SapModel<T>::MakeContext() const {
  return system_->CreateDefaultContext();
}

template <typename T>
const VectorX<T>& SapModel<T>::GetVelocities(const Context<T>& context) const {
  system_->ValidateContext(context);
  return context.get_discrete_state(system_->velocities_index()).value();
}

template <typename T>
Eigen::VectorBlock<VectorX<T>> SapModel<T>::GetMutableVelocities(
    Context<T>* context) const {
  DRAKE_DEMAND(context != nullptr);
  system_->ValidateContext(*context);
  return context->get_mutable_discrete_state(system_->velocities_index())
      .get_mutable_value();
}

template <typename T>
void SapModel<T>::SetVelocities(const VectorX<T>& v,
                                Context<T>* context) const {
  DRAKE_DEMAND(v.size() == num_velocities());
  system_->ValidateContext(*context);
  context->SetDiscreteState(system_->velocities_index(), v);
}

template <typename T>
void SapModel<T>::CalcConstraintVelocities(const Context<T>& context,
                                           VectorX<T>* vc) const {
  system_->ValidateContext(context);
  vc->resize(num_constraint_equations());
  const VectorX<T>& v = GetVelocities(context);
  constraints_bundle().J().Multiply(v, vc);
}

template <typename T>
void SapModel<T>::CalcConstraintBundleDataCache(
    const systems::Context<T>& context,
    SapConstraintBundleDataCache* cache) const {
  system_->ValidateContext(context);
  const VectorX<T>& vc = EvalConstraintVelocities(context);
  constraints_bundle().CalcData(vc, &cache->bundle_data);
}

template <typename T>
void SapModel<T>::CalcImpulsesCache(const Context<T>& context,
                                    ImpulsesCache<T>* cache) const {
  // Impulses are computed as a side effect of updating the Hessian cache.
  // Therefore if the Hessian cache is up to date we do not need to recompute
  // the impulses but simply make a copy into the impulses cache.
  const systems::CacheEntry& hessian_cache_entry =
      system_->get_cache_entry(system_->cache_indexes().hessian);
  const systems::CacheEntryValue& hessian_value =
      hessian_cache_entry.get_cache_entry_value(context);
  if (!hessian_value.is_out_of_date()) {
    const auto& hessian_cache =
        hessian_value.template get_value<HessianCache<T>>();
    *cache = hessian_cache.impulses;
    return;
  }

  system_->ValidateContext(context);
  cache->Resize(num_constraint_equations());
  const SapConstraintBundleData& bundle_data =
      EvalSapConstraintBundleData(context);
  constraints_bundle().CalcImpulses(bundle_data, &cache->gamma);
}

template <typename T>
void SapModel<T>::CalcMomentumGainCache(const Context<T>& context,
                                        MomentumGainCache<T>* cache) const {
  system_->ValidateContext(context);
  cache->Resize(num_velocities());
  const VectorX<T>& v = GetVelocities(context);
  cache->velocity_gain = v - v_star();
  MultiplyByDynamicsMatrix(v, &cache->p);
  cache->momentum_gain = cache->p - p_star();
}

template <typename T>
void SapModel<T>::CalcCostCache(const Context<T>& context,
                                CostCache<T>* cache) const {
  system_->ValidateContext(context);
  const MomentumGainCache<T>& gain_cache = EvalMomentumGainCache(context);
  const VectorX<T>& velocity_gain = gain_cache.velocity_gain;
  const VectorX<T>& momentum_gain = gain_cache.momentum_gain;
  cache->momentum_cost = 0.5 * velocity_gain.dot(momentum_gain);
  const SapConstraintBundleData& bundle_data =
      EvalSapConstraintBundleData(context);
  cache->regularizer_cost = constraints_bundle().CalcCost(bundle_data);
  cache->cost = cache->momentum_cost + cache->regularizer_cost;
}

template <typename T>
void SapModel<T>::CalcGradientsCache(const systems::Context<T>& context,
                                     GradientsCache<T>* cache) const {
  cache->Resize(num_velocities());
  const VectorX<T>& momentum_gain = EvalMomentumGain(context);  // = A⋅(v−v*)
  const VectorX<T>& gamma = EvalImpulses(context);
  constraints_bundle().J().MultiplyByTranspose(gamma, &cache->j);  // = Jᵀ⋅γ
  // Update ∇ᵥℓ = A⋅(v−v*) - Jᵀ⋅γ
  cache->cost_gradient = momentum_gain - cache->j;
}

template <typename T>
void SapModel<T>::CalcHessianCache(const systems::Context<T>& context,
                                   HessianCache<T>* cache) const {
  system_->ValidateContext(context);
  cache->Resize(num_constraints(), num_constraint_equations());
  const SapConstraintBundleData& bundle_data =
      EvalSapConstraintBundleData(context);
  constraints_bundle().CalcImpulsesAndConstraintsHessian(
      bundle_data, &cache->impulses.gamma, &cache->G);
}

template <typename T>
void SapModel<T>::CalcHessianFactorizationCache(
    const systems::Context<T>&, HessianFactorizationCache*) const {
  throw std::runtime_error(
      "Hessian computation is only supported for T = double");
}

template <>
void SapModel<double>::CalcHessianFactorizationCache(
    const systems::Context<double>& context,
    HessianFactorizationCache* hessian) const {
  // Make only for the very first time. This can be an expensive computation for
  // sparse Hessians even when the factorization is not yet computed.
  if (hessian->is_empty()) {
    *hessian = HessianFactorizationCache(hessian_type_, &dynamics_matrix(),
                                         &constraints_bundle().J());
  }
  const std::vector<MatrixX<double>>& G = EvalConstraintsHessian(context);
  hessian->UpdateWeightMatrixAndFactor(G);
}

template <typename T>
int SapModel<T>::num_cliques() const {
  return problem().graph().participating_cliques().permuted_domain_size();
}

template <typename T>
int SapModel<T>::num_velocities() const {
  return velocities_permutation().permuted_domain_size();
}

template <typename T>
int SapModel<T>::num_constraints() const {
  return problem().num_constraints();
}

template <typename T>
int SapModel<T>::num_constraint_equations() const {
  return problem().num_constraint_equations();
}

template <typename T>
const std::vector<MatrixX<T>>& SapModel<T>::dynamics_matrix() const {
  return const_model_data_.dynamics_matrix;
}

template <typename T>
const VectorX<T>& SapModel<T>::v_star() const {
  return const_model_data_.v_star;
}

template <typename T>
const VectorX<T>& SapModel<T>::p_star() const {
  return const_model_data_.p_star;
}

template <typename T>
const VectorX<T>& SapModel<T>::inv_sqrt_dynamics_matrix() const {
  return const_model_data_.inv_sqrt_A;
}

template <typename T>
const SapConstraintBundle<T>& SapModel<T>::constraints_bundle() const {
  DRAKE_DEMAND(const_model_data_.constraints_bundle != nullptr);
  return *const_model_data_.constraints_bundle;
}

template <typename T>
PartialPermutation SapModel<T>::MakeParticipatingVelocitiesPermutation(
    const SapContactProblem<T>& problem) {
  const PartialPermutation& cliques_permutation =
      problem.graph().participating_cliques();

  const int num_participating_cliques =
      cliques_permutation.permuted_domain_size();
  // Compute v_participating_start such that
  // v_participating_start[participating_clique] stores the index to the first
  // participating velocity for participating_clique in `this` SapModel.
  std::vector<int> v_participating_start(num_participating_cliques);
  v_participating_start[0] = 0;
  for (int participating_clique = 1;
       participating_clique < num_participating_cliques;
       ++participating_clique) {
    const int previous_participating_clique =
        cliques_permutation.domain_index(participating_clique - 1);
    const int previous_clique_nv =
        problem.num_velocities(previous_participating_clique);
    v_participating_start[participating_clique] =
        v_participating_start[participating_clique - 1] + previous_clique_nv;
  }

  // Compute participating_velocities such that v_participating =
  // participating_velocities[v] maps velocity index v in the original `problem`
  // to participating velocity index v_participating in `this` SapModel.
  std::vector<int> participating_velocities(problem.num_velocities(), -1);
  int v_start = 0;  // First velocity for a given clique in the original model.
  for (int clique = 0; clique < problem.num_cliques(); ++clique) {
    const int nv = problem.num_velocities(clique);
    if (cliques_permutation.participates(clique)) {
      const int clique_participating =
          cliques_permutation.permuted_index(clique);
      // Add participating dofs to the list.
      for (int i = 0; i < nv; ++i) {
        const int v = v_start + i;
        const int v_participating =
            v_participating_start[clique_participating] + i;
        participating_velocities[v] = v_participating;
      }
    }
    v_start += nv;
  }

  // Sanity check.
  DRAKE_DEMAND(v_start == problem.num_velocities());

  return PartialPermutation(std::move(participating_velocities));
}

template <typename T>
PartialPermutation SapModel<T>::MakeImpulsesPermutation(
    const ContactProblemGraph& graph) const {
  if (problem().num_constraints() == 0)
    return PartialPermutation();  // empty permutation.

  std::vector<int> constraint_start(problem().num_constraints());
  constraint_start[0] = 0;
  for (int i = 1; i < problem().num_constraints(); ++i) {
    const int previous_constraint_size =
        problem().get_constraint(i - 1).num_constraint_equations();
    constraint_start[i] = constraint_start[i - 1] + previous_constraint_size;
  }

  std::vector<int> impulses_permutation(problem().num_constraint_equations());
  int group_offset = 0;  // impulse index.
  for (const ContactProblemGraph::ConstraintCluster& g : graph.clusters()) {
    for (int i : g.constraint_index()) {
      const SapConstraint<T>& c = problem().get_constraint(i);
      const int ni = c.num_constraint_equations();
      const int offset = constraint_start[i];
      for (int m = 0; m < ni; ++m) {
        impulses_permutation[offset + m] = group_offset + m;
      }
      group_offset += ni;
    }
  }

  return PartialPermutation(std::move(impulses_permutation));
}

template <typename T>
void SapModel<T>::MultiplyByDynamicsMatrix(const VectorX<T>& v,
                                           VectorX<T>* p) const {
  DRAKE_DEMAND(v.size() == num_velocities());
  DRAKE_DEMAND(p->size() == num_velocities());
  int clique_start = 0;
  for (const auto& Ab : dynamics_matrix()) {
    const int clique_size = Ab.rows();
    p->segment(clique_start, clique_size) =
        Ab * v.segment(clique_start, clique_size);
    clique_start += clique_size;
  }
}

template <typename T>
void SapModel<T>::CalcDelassusDiagonalApproximation(
    const std::vector<MatrixX<T>>& A, VectorX<T>* delassus_diagonal) const {
  DRAKE_DEMAND(delassus_diagonal != nullptr);
  DRAKE_DEMAND(static_cast<int>(A.size()) == num_cliques());

  // We compute a factorization of A once so we can re-use it multiple times
  // below.
  const int num_cliques = A.size();  // N.B. Participating cliques.
  std::vector<VectorX<T>> A_diag_inv(num_cliques);
  for (int c = 0; c < num_cliques; ++c) {
    A_diag_inv[c] = A[c].diagonal().cwiseInverse();
  }

  // Scan constraints in the order specified by the graph.
  const int num_constraints = problem().num_constraints();
  std::vector<MatrixX<T>> W(num_constraints);

  const ContactProblemGraph& graph = problem().graph();
  const PartialPermutation& cliques_permutation = graph.participating_cliques();

  int i_cluster = 0;
  for (const ContactProblemGraph::ConstraintCluster& e :
       problem().graph().clusters()) {
    for (int i : e.constraint_index()) {
      const SapConstraint<T>& constraint = problem().get_constraint(i);
      const int ni = constraint.num_constraint_equations();
      if (W[i_cluster].size() == 0) {
        // Resize and initialize to zero on the first time it gets accessed.
        W[i_cluster].resize(ni, ni);
        W[i_cluster].setZero();
      }

      // Clique 0 is always present. Add its contribution.
      {
        const int c =
            cliques_permutation.permuted_index(constraint.first_clique());
        const MatrixBlock<T>& Jic = constraint.first_clique_jacobian();
        Jic.MultiplyWithScaledTransposeAndAddTo(A_diag_inv[c], &W[i_cluster]);
      }

      // Adds clique 1 contribution, if present.
      if (constraint.num_cliques() == 2) {
        const int c =
            cliques_permutation.permuted_index(constraint.second_clique());
        const MatrixBlock<T>& Jic = constraint.second_clique_jacobian();
        Jic.MultiplyWithScaledTransposeAndAddTo(A_diag_inv[c], &W[i_cluster]);
      }
      ++i_cluster;
    }
  }

  // Compute Delassus_diagonal as the rms norm of the diagonal block for the
  // i-th constraint.
  delassus_diagonal->resize(num_constraint_equations());
  int constraint_start = 0;
  for (int i = 0; i < num_constraints; ++i) {
    const int ni = W[i].rows();
    // TODO(amcastro-tri): consider using the scalar diagonal value W[i](e, e)
    // or something like max(W[i](e, e), W_rms).
    delassus_diagonal->segment(constraint_start, ni)
        .setConstant(W[i].norm() / ni);
    constraint_start += ni;
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapModel);
