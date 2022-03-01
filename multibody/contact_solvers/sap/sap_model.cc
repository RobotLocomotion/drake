#include "drake/multibody/contact_solvers/sap/sap_model.h"

#include "drake/common/default_scalars.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapModel<T>::SapModel(const T& time_step,
                      const SystemDynamicsData<T>& dynamics_data,
                      const PointContactData<T>& contact_data) {
  (void)time_step;
  (void)dynamics_data;
  (void)contact_data;
}

template <typename T>
SapModel<T>::SapModel(const SapContactProblem<T>* problem) : problem_(problem) {
  // Graph for the original contact problem, including all cliques
  // (participating and non-participating).
  const ContactProblemGraph& graph = sap_problem().graph();

  // Permutations to map indexes from participating cliques/dofs to the original
  // set of cliques/dofs.
  // TODO: remove cliques_permutation_.
  cliques_permutation_ = graph.participating_cliques();
  velocities_permutation_ = MakeParticipatingVelocitiesPermutation(
      sap_problem(), cliques_permutation_);
  impulses_permutation_ = MakeImpulsesPermutation(graph);

  // Extract momentum matrix's per-tree diagonal blocks. Compute diagonal
  // scaling inv_sqrt_A.
  const int num_participating_cliques =
      cliques_permutation_.permuted_domain_size();
  A_.resize(num_participating_cliques);  
  cliques_permutation_.Apply(sap_problem().dynamics_matrix(), &A_);  

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
  velocities_permutation_.Apply(sap_problem().v_star(), &v_star_);
  MultiplyByDynamicsMatrix(v_star_, &p_star_);
  CalcDelassusDiagonalApproximation(A_, sap_problem(), graph,
                                    cliques_permutation_, &delassus_diagonal_);

  // TODO: remove this DEMAND.
  DRAKE_DEMAND(delassus_diagonal_.size() == sap_problem().num_constraints());  

  // Create constraints bundle.
  constraints_bundle_ = std::make_unique<SapConstraintBundle<T>>(
      &sap_problem(), delassus_diagonal_);
}

template <typename T>
int SapModel<T>::num_cliques() const {
  return cliques_permutation_.domain_size();
}

template <typename T>
int SapModel<T>::num_participating_cliques() const {
  return cliques_permutation_.permuted_domain_size();
}

template <typename T>
int SapModel<T>::num_velocities() const {
  return sap_problem().num_velocities();
}

template <typename T>
int SapModel<T>::num_participating_velocities() const {
  return velocities_permutation_.permuted_domain_size();
}

template <typename T>
int SapModel<T>::num_constraints() const {
  return sap_problem().num_constraints();
}

template <typename T>
int SapModel<T>::num_impulses() const {
  return sap_problem().num_constraint_equations();
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
PartialPermutation SapModel<T>::MakeParticipatingCliquesPermutation(
    const ContactProblemGraph& graph) const {
  std::vector<int> participating_cliques(graph.num_cliques(), -1);
  int num_participating_cliques = 0;
  for (const auto& e : graph.clusters()) {
    const int c0 = e.cliques().first();
    const int c1 = e.cliques().second();
    if (participating_cliques[c0] < 0) {
      participating_cliques[c0] = num_participating_cliques++;
    }
    if (c1 != c0 && participating_cliques[c1] < 0)
      participating_cliques[c1] = num_participating_cliques++;
  }  
  return PartialPermutation(std::move(participating_cliques));
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
  std::vector<int> constraint_start(sap_problem().num_constraints());
  if (sap_problem().num_constraints() == 0)  
    return PartialPermutation();  // empty permutation.
    
  constraint_start[0] = 0;
  for (int i = 1; i < sap_problem().num_constraints(); ++i) {
    const int previous_constraint_size =
        sap_problem().get_constraint(i - 1).num_constraint_equations();
    constraint_start[i] = constraint_start[i - 1] + previous_constraint_size;
  }

  std::vector<int> impulses_permutation(sap_problem().num_constraint_equations());
  int group_offset = 0;  // impulse index.
  for (const ContactProblemGraph::ConstraintCluster& g : graph.clusters()) {
    for (int i : g.constraint_index()) {
      const SapConstraint<T>& c = sap_problem().get_constraint(i);
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
    const std::vector<MatrixX<T>>& A, const SapContactProblem<T>& problem,
    const ContactProblemGraph& graph, 
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
  const int num_constraints = problem.num_constraints();
  std::vector<MatrixX<T>> W(num_constraints);

  for (const ContactProblemGraph::ConstraintCluster& e : graph.clusters()) {
    for (int i : e.constraint_index()) {      
      const SapConstraint<T>& constraint = sap_problem().get_constraint(i);
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
void SapModel<T>::CalcDelassusDiagonalApproximation(
    int nc, const std::vector<MatrixX<T>>& At, const BlockSparseMatrix<T>& J,
    VectorX<T>* delassus_diagonal) const {
  DRAKE_DEMAND(delassus_diagonal != nullptr);
  DRAKE_DEMAND(delassus_diagonal->size() == nc);
  const int nt = At.size();  // Number of trees.

  // We compute a factorization of A once so we can re-use it multiple times
  // below.
  std::vector<Eigen::LDLT<MatrixX<T>>> A_ldlt;
  A_ldlt.resize(nt);
  for (int t = 0; t < nt; ++t) {
    const auto& At_local = At[t];
    A_ldlt[t] = At_local.ldlt();
  }

  // We compute a diagonal approximation to the Delassus operator W. We
  // initialize it to zero and progressively add contributions in an O(n) pass.
  std::vector<Matrix3<T>> W(nc, Matrix3<T>::Zero());
  for (auto [p, t, Jpt] : J.get_blocks()) {
    // Verify assumption that this indeed is a contact Jacobian.
    // TODO: Update this to be written in terms of general constraints.
    DRAKE_DEMAND(J.row_start(p) % 3 == 0);
    DRAKE_DEMAND(Jpt.rows() % 3 == 0);
    // ic_start is the first contact point of patch p.
    const int ic_start = J.row_start(p) / 3;
    // k-th contact within patch p.
    for (int k = 0; k < Jpt.rows() / 3; ++k) {
      const int ic = ic_start + k;
      const auto& Jkt = Jpt.template middleRows<3>(3 * k);
      // This effectively computes Jₖₜ⋅A⁻¹⋅Jₖₜᵀ.
      W[ic] += Jkt * A_ldlt[t].solve(Jkt.transpose());
    }
  }

  // Compute delassus_diagonal as the rms norm of k-th diagonal block.
  for (int k = 0; k < nc; ++k) {
    (*delassus_diagonal)[k] = W[k].norm() / 3;
  }
}

template <typename T>
void SapModel<T>::MultiplyByDynamicsMatrix(const VectorX<T>& v,
                                           VectorX<T>* p) const {
  DRAKE_DEMAND(v.size() == num_participating_velocities());
  DRAKE_DEMAND(p->size() == num_participating_velocities());
  int clique_start = 0;
  for (const auto& Ab : A_) {
    const int clique_size = Ab.rows();
    p->segment(clique_start, clique_size) =
        Ab * v.segment(clique_start, clique_size);
    clique_start += clique_size;
  }
}

template <typename T>
void SapModel<T>::CalcConstraintVelocities(const VectorX<T>& v,
                                           VectorX<T>* vc) const {
  DRAKE_DEMAND(v.size() == num_participating_velocities());
  DRAKE_DEMAND(vc != nullptr);
  J().Multiply(v, vc);
}

template <typename T>
void SapModel<T>::CalcUnprojectedImpulses(const VectorX<T>& vc,
                                          VectorX<T>* y) const {
  DRAKE_DEMAND(vc.size() == num_impulses());
  DRAKE_DEMAND(y != nullptr);
  DRAKE_DEMAND(y->size() == num_impulses());
  constraints_bundle_->CalcUnprojectedImpulses(vc, y);
}

template <typename T>
void SapModel<T>::ProjectImpulses(const VectorX<T>& y,
                                  VectorX<T>* gamma) const {
  constraints_bundle_->ProjectImpulses(y, gamma);
}

template <typename T>
void SapModel<T>::ProjectImpulsesAndCalcConstraintsHessian(
    const VectorX<T>& y, VectorX<T>* gamma, std::vector<MatrixX<T>>* G) const {
  constraints_bundle_->ProjectImpulsesAndCalcConstraintsHessian(y, gamma, G);
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapModel)    
