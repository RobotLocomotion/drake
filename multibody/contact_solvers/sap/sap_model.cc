#include "drake/multibody/contact_solvers/sap/sap_model.h"

#include "drake/common/default_scalars.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// a good way to document it and describe its invariants.
template <typename T>
SapConstraintBundle<T>::SapConstraintBundle(
    BlockSparseMatrix<T>&& J, VectorX<T>&& vhat, VectorX<T>&& R,
    std::vector<const SapConstraint<T>*>&& constraints)
    : J_(std::move(J)),
      vhat_(std::move(vhat)),
      R_(std::move(R)),
      Rinv_(R_.cwiseInverse()),
      constraints_(std::move(constraints)) {}

template <typename T>
int SapConstraintBundle<T>::num_constraints() const {
  return constraints_.size();
}

template <typename T>
void SapConstraintBundle<T>::MultiplyByJacobian(const VectorX<T>& v,
                                                 VectorX<T>* vc) const {
  J_.Multiply(v, vc);
}

template <typename T>
void SapConstraintBundle<T>::MultiplyByJacobianTranspose(
    const VectorX<T>& gamma, VectorX<T>* jc) const {
  J_.MultiplyByTranspose(gamma, jc);
}

template <typename T>
void SapConstraintBundle<T>::CalcUnprojectedImpulses(const VectorX<T>& vc,
                                                      VectorX<T>* y) const {
  *y = Rinv_.asDiagonal() * (vhat_ - vc);
}

template <typename T>
void SapConstraintBundle<T>::ProjectImpulses(
    const VectorX<T>& y, const VectorX<T>& R, VectorX<T>* gamma,
    std::vector<MatrixX<T>>* dPdy) const {
  int constraint_start = 0;
  for (int i = 0; i < num_constraints(); ++i) {
    const SapConstraint<T>& c = *constraints_[i];
    const int ni = c.num_constrained_dofs();
    const auto y_i = y.segment(constraint_start, ni);
    const auto R_i = R.segment(constraint_start, ni);
    auto gamma_i = gamma->segment(constraint_start, ni);
    if (dPdy != nullptr) {
      MatrixX<T>& dPdy_i = (*dPdy)[i];
      c.Project(y_i, R_i, &gamma_i, &dPdy_i);
    } else {
      c.Project(y_i, R_i, &gamma_i);
    }
    constraint_start += ni;
  }
}

template <typename T>
void SapConstraintBundle<T>::ProjectImpulsesAndCalcConstraintsHessian(
    const VectorX<T>& y, const VectorX<T>& R, VectorX<T>* gamma,
    std::vector<MatrixX<T>>* G) const {
  ProjectImpulses(y, R, gamma, G);  // G = dPdy.

  // The regularizer Hessian is G = ∇²ℓ = d²ℓ/dvc² = dP/dy⋅R⁻¹.
  int constraint_start = 0;
  for (int i = 0; i < num_constraints(); ++i) {
    const SapConstraint<T>& c = *constraints_[i];
    const int ni = c.num_constrained_dofs();
    const auto R_i = R.segment(constraint_start, ni);
    const MatrixUpTo6<T> dPdy_i = (*G)[i];
    (*G)[i] = dPdy_i * R_i.cwiseInverse().asDiagonal();
    constraint_start += ni;
  }
}

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
  const ContactProblemGraph graph = sap_problem().MakeGraph();

  // Permutations to map indexes from participating cliques/dofs to the original
  // set of cliques/dofs.
  cliques_permutation_ = MakeParticipatingCliquesPermutation(graph);
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

  BlockSparseMatrix<T> J = MakeConstraintsBundleJacobian(sap_problem(), graph,
                                                         cliques_permutation_);

  CalcDelassusDiagonalApproximation(A_, sap_problem(), graph,
                                    cliques_permutation_, &delassus_diagonal_);

  // TODO: remove this DEMAND.
  DRAKE_DEMAND(delassus_diagonal_.size() == sap_problem().num_constraints());  

  // Vector of bias velocities and diagonal matrix R.
  VectorX<T> vhat(sap_problem().num_constrained_dofs());
  VectorX<T> R(sap_problem().num_constrained_dofs());

  // Create vector of constraints but not in the oder they were enumerated in
  // the SapProblem, but in the computationally convenient order enumerated in
  // the ContactProblemGraph.
  std::vector<const SapConstraint<T>*> constraints;
  constraints.reserve(sap_problem().num_constraints());

  offset = 0;  // impulse index.
  for (const ContactProblemGraph::ConstraintGroup& e : graph.constraint_groups()) {
    for (int i : e.constraints_index) {
      const SapConstraint<T>& c = sap_problem().get_constraint(i);
      constraints.push_back(&c);

      const int ni = c.num_constrained_dofs();
      const T& wi = delassus_diagonal_[i];

      vhat.segment(offset, ni) = c.CalcBiasTerm(sap_problem().time_step(), wi);
      R.segment(offset, ni) =
          c.CalcDiagonalRegularization(sap_problem().time_step(), wi);

      offset += ni;
    }
  }

  // Create constraints bundle.
  constraints_bundle_ = std::make_unique<SapConstraintBundle<T>>(
      std::move(J), std::move(vhat), std::move(R), std::move(constraints));
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
  return sap_problem().num_constrained_dofs();
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
  for (const auto& e : graph.constraint_groups()) {
    const int c0 = e.cliques.first();
    const int c1 = e.cliques.second();
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
        sap_problem().get_constraint(i - 1).num_constrained_dofs();
    constraint_start[i] = constraint_start[i - 1] + previous_constraint_size;
  }

  std::vector<int> impulses_permutation(sap_problem().num_constrained_dofs());
  int group_offset = 0;  // impulse index.
  for (const ContactProblemGraph::ConstraintGroup& g : graph.constraint_groups()) {
    for (int i : g.constraints_index) {
      const SapConstraint<T>& c = sap_problem().get_constraint(i);
      const int ni = c.num_constrained_dofs();
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

  for (const ContactProblemGraph::ConstraintGroup& e : graph.constraint_groups()) {
    for (int i : e.constraints_index) {      
      const SapConstraint<T>& constraint = sap_problem().get_constraint(i);
      const int ni = constraint.num_constrained_dofs();
      if (W[i].size() == 0) {
        // Resize and initialize to zero on the first time it gets accessed.
        W[i].resize(ni, ni);
        W[i].setZero();
      }

      // Clique 0 is always present. Add its contribution.
      {
        const int c = cliques_permutation.permuted_index(constraint.clique0());
        const MatrixX<T>& Jic = constraint.clique0_jacobian();
        W[i] += Jic * A_ldlt[c].solve(Jic.transpose());
      }

      // Adds clique 1 contribution, if present.
      if (constraint.num_cliques() == 2) {
        const int c = cliques_permutation.permuted_index(constraint.clique1());
        const MatrixX<T>& Jic = constraint.clique1_jacobian();
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
BlockSparseMatrix<T> SapModel<T>::MakeConstraintsBundleJacobian(
    const SapContactProblem<T>& problem, const ContactProblemGraph& graph,
    const PartialPermutation& cliques_permutation) const {
  // We have at most two blocks per row, and one row per edge in the graph.
  const int non_zero_blocks_capacity = 2 * graph.num_constraint_groups();
  BlockSparseMatrixBuilder<T> builder(
      graph.num_constraint_groups(), cliques_permutation.permuted_domain_size(),
      non_zero_blocks_capacity);

  // DOFs per edge.
  // TODO: consider the graph storing "weights"; number of dofs per node
  // (clique) and number of constrained dofs per edge.
  std::vector<int> group_dofs(graph.num_constraint_groups(), 0);
  for (int e = 0; e < graph.num_constraint_groups(); ++e) {
    const auto& edge = graph.get_constraint_group(e);
    for (int k : edge.constraints_index) {
      const SapConstraint<T>& c = problem.get_constraint(k);
      group_dofs[e] += c.num_constrained_dofs();
    }
  }

  // Add a block row (with one or two blocks) per group of constraints in the
  // graph.
  for (int block_row = 0; block_row < graph.num_constraint_groups(); ++block_row) {
    const auto& constraint_group = graph.get_constraint_group(block_row);
    // N.B. These are clique indexes in the original contact problem (including
    // both participating and non-participating cliques).
    const int c0 = constraint_group.cliques.first();
    const int c1 = constraint_group.cliques.second();

    // Allocate Jacobian blocks for this group of constraints.
    MatrixX<T> J0, J1;
    const int num_rows = group_dofs[block_row];
    const int nv0 = problem.num_velocities(c0);
    J0.resize(num_rows, nv0);
    if (c1 != c0) {  // If not a "loop" in the graph.
      const int nv1 = problem.num_velocities(c1);
      J1.resize(num_rows, nv1);
    }

    // Constraints are added in the order set by the graph.
    int row_start = 0;
    for (int i : constraint_group.constraints_index) {
      const SapConstraint<T>& c = problem.get_constraint(i);
      const int ni = c.num_constrained_dofs();

      // N.B. Each edge stores its cliques as a sorted pair. However, the pair
      // of cliques in the original constraints can be in arbitrary order.
      // Therefore below we must check to what clique in the original constraint
      // the group's cliques correspond to.
      
      J0.middleRows(row_start, ni) =
            c0 == c.clique0() ? c.clique0_jacobian() : c.clique1_jacobian();
      if (c1 != c0) {
        J1.middleRows(row_start, ni) =
            c1 == c.clique0() ? c.clique0_jacobian() : c.clique1_jacobian();
      }

      row_start += ni;
    }

    const int participating_c0 = cliques_permutation.permuted_index(c0);
    builder.PushBlock(block_row, participating_c0, J0);
    if (c1 != c0) {
      const int participating_c1 = cliques_permutation.permuted_index(c1);
      builder.PushBlock(block_row, participating_c1, J1);
    }
  }

  return builder.Build();
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
  constraints_bundle_->ProjectImpulses(y, R(), gamma);
}

template <typename T>
void SapModel<T>::ProjectImpulsesAndCalcConstraintsHessian(
    const VectorX<T>& y, VectorX<T>* gamma, std::vector<MatrixX<T>>* G) const {
  constraints_bundle_->ProjectImpulsesAndCalcConstraintsHessian(y, R(), gamma,
                                                                G);
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapConstraintBundle)
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapModel)    
