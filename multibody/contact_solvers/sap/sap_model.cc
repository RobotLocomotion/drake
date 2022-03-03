#include "drake/multibody/contact_solvers/sap/sap_model.h"

#include "drake/common/default_scalars.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

using systems::Context;

template <typename T>
SapModel<T>::SapModel(const SapContactProblem<T>* problem_ptr)
    : problem_(problem_ptr) {
  // Graph for the original contact problem, including all cliques
  // (participating and non-participating).
  const ContactProblemGraph& graph = problem().graph();

  // Permutations to map indexes from participating cliques/dofs to the original
  // set of cliques/dofs.
  // TODO: remove cliques_permutation_.
  const PartialPermutation& cliques_permutation_ =
      graph.participating_cliques();
  velocities_permutation_ = MakeParticipatingVelocitiesPermutation(
      problem(), cliques_permutation_);
  impulses_permutation_ = MakeImpulsesPermutation(graph);

  // Extract momentum matrix's per-tree diagonal blocks. Compute diagonal
  // scaling inv_sqrt_A.
  const int num_participating_cliques =
      cliques_permutation_.permuted_domain_size();
  A_.resize(num_participating_cliques);  
  cliques_permutation_.Apply(problem().dynamics_matrix(), &A_);  

  // Compute inv_sqrt_A_.
  const int nv_participating = velocities_permutation_.permuted_domain_size();
  inv_sqrt_A_.resize(nv_participating);
  int offset = 0;
  for (int clique = 0; clique < num_participating_cliques; ++clique) {
    const MatrixX<T>& Aclique = A_[clique];
    // Each block must be square.
    DRAKE_DEMAND(Aclique.rows() == Aclique.cols());
    const int nv = Aclique.rows();  // Number of DOFs in the clique.
    inv_sqrt_A_.segment(offset, nv) =
        Aclique.diagonal().cwiseInverse().cwiseSqrt();
    offset += nv;
  }

  p_star_.resize(nv_participating);
  v_star_.resize(nv_participating);
  velocities_permutation_.Apply(problem().v_star(), &v_star_);
  MultiplyByDynamicsMatrix(v_star_, &p_star_);
  CalcDelassusDiagonalApproximation(A_, cliques_permutation_,
                                    &delassus_diagonal_);

  // TODO: remove this DEMAND.
  DRAKE_DEMAND(delassus_diagonal_.size() == problem().num_constraints());  

  // Create constraints bundle.
  constraints_bundle_ = std::make_unique<SapConstraintBundle<T>>(
      &problem(), delassus_diagonal_);

  DeclareStateAndCacheEntries();      
}

template <typename T>
void SapModel<T>::DeclareStateAndCacheEntries() {
  system_.DeclareDiscreteState(num_velocities());

  // Cache constraint velocities vc.
  const auto& constraint_velocities_cache_entry = system_.DeclareCacheEntry(
      "Constraint velocities vc.",
      systems::ValueProducer(
          this, &SapModel<T>::CalcConstraintVelocities),
      {systems::System<T>::xd_ticket()});
  system_.mutable_cache_indexes().constraint_velocities =
      constraint_velocities_cache_entry.cache_index();
}

template <typename T>
std::unique_ptr<systems::Context<T>> SapModel<T>::MakeContext() const {
  return system_.CreateDefaultContext();
}

template <typename T>
const VectorX<T>& SapModel<T>::GetVelocities(const Context<T>& context) const {
  system_.ValidateContext(context);
  return context.get_discrete_state(system_.velocities_index()).value();
}

template <typename T>
void SapModel<T>::SetVelocities(const VectorX<T>& v, Context<T>* context) const {
  system_.ValidateContext(*context);
  context->SetDiscreteState(system_.velocities_index(), v);
}

template <typename T>
void SapModel<T>::CalcConstraintVelocities(const Context<T>& context,
                                           VectorX<T>* vc) const {
  system_.ValidateContext(context);                                             
  const VectorX<T>& v = GetVelocities(context);
  J().Multiply(v, vc);
}

template <typename T>
int SapModel<T>::num_cliques() const {
  return cliques_permutation().permuted_domain_size();
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
  return A_;
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
    const SapContactProblem<T>& problem,
    const PartialPermutation& cliques_permutation) const {
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
PartialPermutation SapModel<T>::MakeImpulsesPermutation(
    const ContactProblemGraph& graph) const {
  std::vector<int> constraint_start(problem().num_constraints());
  if (problem().num_constraints() == 0)  
    return PartialPermutation();  // empty permutation.
    
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
void SapModel<T>::CalcDelassusDiagonalApproximation(
    const std::vector<MatrixX<T>>& A,
    const PartialPermutation& cliques_permutation,
    VectorX<T>* delassus_diagonal) const {
  DRAKE_DEMAND(delassus_diagonal != nullptr);    

  // We compute a factorization of A once so we can re-use it multiple times
  // below.
  const int num_cliques = A.size();  // N.B. Participating cliques.
  std::vector<Eigen::LDLT<MatrixX<T>>> A_ldlt;
  A_ldlt.resize(num_cliques);
  for (int c = 0; c < num_cliques; ++c) {
    A_ldlt[c] = A[c].ldlt();
  }

  // Scan constraints in the order specified by the graph.
  const int num_constraints = problem().num_constraints();
  std::vector<MatrixX<T>> W(num_constraints);

  for (const ContactProblemGraph::ConstraintCluster& e :
       problem().graph().clusters()) {
    for (int i : e.constraint_index()) {
      const SapConstraint<T>& constraint = problem().get_constraint(i);
      const int ni = constraint.num_constraint_equations();
      if (W[i].size() == 0) {
        // Resize and initialize to zero on the first time it gets accessed.
        W[i].resize(ni, ni);
        W[i].setZero();
      }

      // Clique 0 is always present. Add its contribution.
      {
        const int c = cliques_permutation.permuted_index(constraint.first_clique());
        const MatrixX<T>& Jic = constraint.first_clique_jacobian();
        W[i] += Jic * A_ldlt[c].solve(Jic.transpose());
      }

      // Adds clique 1 contribution, if present.
      if (constraint.num_cliques() == 2) {
        const int c = cliques_permutation.permuted_index(constraint.second_clique());
        const MatrixX<T>& Jic = constraint.second_clique_jacobian();
        W[i] += Jic * A_ldlt[c].solve(Jic.transpose());
      }
    }
  }

  // Compute delassus_diagonal as the rms norm of the diagonal block for the
  // i-th constraint.
  delassus_diagonal->resize(num_constraints);
  for (int i = 0; i < num_constraints; ++i) {
    (*delassus_diagonal)[i] = W[i].norm() / W[i].rows();
  }
}

template <typename T>
void SapModel<T>::MultiplyByDynamicsMatrix(const VectorX<T>& v,
                                           VectorX<T>* p) const {
  DRAKE_DEMAND(v.size() == num_velocities());
  DRAKE_DEMAND(p->size() == num_velocities());
  int clique_start = 0;
  for (const auto& Ab : A_) {
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
