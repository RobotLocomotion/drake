#include "drake/multibody/contact_solvers/sap/sap_model.h"

#include <memory>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

using systems::Context;

template <typename T>
SapModel<T>::SapModel(const SapContactProblem<T>* problem_ptr)
    : problem_(problem_ptr) {
  // Graph to the original contact problem, including all cliques
  // (participating and non-participating).
  const ContactProblemGraph& graph = problem().graph();

  // Permutations to map indexes from participating cliques/dofs to the original
  // set of cliques/dofs.
  const PartialPermutation& cliques_permutation = graph.participating_cliques();
  velocities_permutation_ = MakeParticipatingVelocitiesPermutation(problem());

  // Extract momentum matrix for participating DOFs only.
  const int num_participating_cliques =
      cliques_permutation.permuted_domain_size();
  dynamics_matrix_.resize(num_participating_cliques);
  cliques_permutation.Apply(problem().dynamics_matrix(), &dynamics_matrix_);

  // Get v* and p* for participating DOFs only.
  const int nv_participating = velocities_permutation_.permuted_domain_size();
  p_star_.resize(nv_participating);
  v_star_.resize(nv_participating);
  velocities_permutation_.Apply(problem().v_star(), &v_star_);
  MultiplyByDynamicsMatrix(v_star_, &p_star_);

  // TODO(amcastro-tri): Compute diagonal approximation of the Delassus
  // operator. For now we set it to NaN so that if used by accident it'll
  // trigger an easy trail to follow.
  const VectorX<T> delassus_diagonal =
      VectorX<T>::Constant(num_constraints(), NAN);

  // Create constraints bundle.
  constraints_bundle_ =
      std::make_unique<SapConstraintBundle<T>>(&problem(), delassus_diagonal);

  system_ = std::make_unique<SapModelSystem>(num_velocities());
  DeclareCacheEntries();
}

template <typename T>
void SapModel<T>::DeclareCacheEntries() {
  const auto& constraint_velocities_cache_entry = system_->DeclareCacheEntry(
      "Constraint velocities, vc = J⋅v.",
      systems::ValueProducer(this, &SapModel<T>::CalcConstraintVelocities),
      {systems::System<T>::xd_ticket()});
  system_->mutable_cache_indexes().constraint_velocities =
      constraint_velocities_cache_entry.cache_index();

  const auto& momentum_gain_cache_entry = system_->DeclareCacheEntry(
      "Momentum gain, momentum_gain = A⋅(v-v*).",
      systems::ValueProducer(this, &SapModel<T>::CalcMomentumGain),
      {systems::System<T>::xd_ticket()});
  system_->mutable_cache_indexes().momentum_gain =
      momentum_gain_cache_entry.cache_index();

  // This cache entry is not costly to compute. Having this computation in the
  // cache however elimianates heap allocation for temporaries.
  const auto& velocity_gain_cache_entry = system_->DeclareCacheEntry(
      "Velocity gain, velocity_gain = v-v*.",
      systems::ValueProducer(this, &SapModel<T>::CalcVelocityGain),
      {systems::System<T>::xd_ticket()});
  system_->mutable_cache_indexes().velocity_gain =
      velocity_gain_cache_entry.cache_index();

  const auto& momentum_cost_cache_entry = system_->DeclareCacheEntry(
      "Momentum cost, momentum_cost = 1/2⋅(v-v*)ᵀ⋅A⋅(v-v*).",
      systems::ValueProducer(this, &SapModel<T>::CalcMomentumCost),
      {system_->cache_entry_ticket(
           system_->mutable_cache_indexes().velocity_gain),
       system_->cache_entry_ticket(
           system_->mutable_cache_indexes().momentum_gain)});
  system_->mutable_cache_indexes().momentum_cost =
      momentum_cost_cache_entry.cache_index();
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
  constraints_bundle_->J().Multiply(v, vc);
}

template <typename T>
void SapModel<T>::CalcVelocityGain(const Context<T>& context,
                                   VectorX<T>* velocity_gain) const {
  system_->ValidateContext(context);
  velocity_gain->resize(num_velocities());
  *velocity_gain = GetVelocities(context) - v_star();
}

template <typename T>
void SapModel<T>::CalcMomentumGain(const Context<T>& context,
                                   VectorX<T>* momentum_gain) const {
  system_->ValidateContext(context);
  momentum_gain->resize(num_velocities());
  const VectorX<T>& velocity_gain = EvalVelocityGain(context);
  MultiplyByDynamicsMatrix(velocity_gain, momentum_gain);
}

template <typename T>
void SapModel<T>::CalcMomentumCost(const Context<T>& context,
                                   T* momentum_cost) const {
  system_->ValidateContext(context);
  const VectorX<T>& velocity_gain = EvalVelocityGain(context);
  const VectorX<T>& momentum_gain = EvalMomentumGain(context);
  (*momentum_cost) = 0.5 * velocity_gain.dot(momentum_gain);
}

template <typename T>
int SapModel<T>::num_cliques() const {
  return problem().graph().participating_cliques().permuted_domain_size();
}

template <typename T>
int SapModel<T>::num_velocities() const {
  return velocities_permutation_.permuted_domain_size();
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
  return dynamics_matrix_;
}

template <typename T>
const VectorX<T>& SapModel<T>::v_star() const {
  return v_star_;
}

template <typename T>
const VectorX<T>& SapModel<T>::p_star() const {
  return p_star_;
}

template <typename T>
PartialPermutation SapModel<T>::MakeParticipatingVelocitiesPermutation(
    const SapContactProblem<T>& problem) {
  const PartialPermutation& cliques_permutation =
      problem.graph().participating_cliques();
  int v_first = 0;  // first velocity for a given clique.
  int num_participating_velocities = 0;
  std::vector<int> participating_velocities(problem.num_velocities(), -1);
  for (int c = 0; c < problem.num_cliques(); ++c) {
    const int nv = problem.num_velocities(c);
    if (cliques_permutation.participates(c)) {
      // Add participating dofs to the list.
      for (int i = 0; i < nv; ++i) {
        const int v = v_first + i;
        const int v_participating = num_participating_velocities++;
        participating_velocities[v] = v_participating;
      }
    }
    v_first += nv;
  }
  return PartialPermutation(std::move(participating_velocities));
}

template <typename T>
void SapModel<T>::MultiplyByDynamicsMatrix(const VectorX<T>& v,
                                           VectorX<T>* p) const {
  DRAKE_DEMAND(v.size() == num_velocities());
  DRAKE_DEMAND(p->size() == num_velocities());
  int clique_start = 0;
  for (const auto& Ab : dynamics_matrix_) {
    const int clique_size = Ab.rows();
    p->segment(clique_start, clique_size) =
        Ab * v.segment(clique_start, clique_size);
    clique_start += clique_size;
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapModel)
