#include "drake/multibody/contact_solvers/sap/sap_model.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// a good way to document it and describe its invariants.
template <typename T>
SapConstraintsBundle<T>::SapConstraintsBundle(
    BlockSparseMatrix<T>&& J, VectorX<T>&& vhat, VectorX<T>&& R,
    std::vector<const SapConstraint<T>*>&& constraints)
    : J_(std::move(J)),
      vhat_(std::move(vhat)),
      R_(std::move(R)),
      Rinv_(R_.cwiseInverse()),
      constraints_(std::move(constraints)) {}

template <typename T>
int SapConstraintsBundle<T>::num_constraints() const {
  return constraints_.size();
}

template <typename T>
void SapConstraintsBundle<T>::MultiplyByJacobian(const VectorX<T>& v,
                                                 VectorX<T>* vc) const {
  J_.Multiply(v, vc);
}

template <typename T>
void SapConstraintsBundle<T>::MultiplyByJacobianTranspose(
    const VectorX<T>& gamma, VectorX<T>* jc) const {
  J_.MultiplyByTranspose(gamma, jc);
}

template <typename T>
void SapConstraintsBundle<T>::CalcUnprojectedImpulses(const VectorX<T>& vc,
                                                      VectorX<T>* y) const {
  *y = Rinv_.asDiagonal() * (vhat_ - vc);
}

template <typename T>
void SapConstraintsBundle<T>::ProjectImpulses(
    const VectorX<T>& y, const VectorX<T>& R, VectorX<T>* gamma,
    std::vector<MatrixX<T>>* dPdy) const {
  int constraint_start = 0;
  for (int k = 0; k < num_constraints(); ++k) {
    const SapConstraint<T>& c = *constraints_[k];
    const int nk = c.num_constrained_dofs();
    const auto y_k = y.segment(constraint_start, nk);
    const auto R_k = R.segment(constraint_start, nk);
    auto gamma_k = gamma->segment(constraint_start, nk);
    if (dPdy != nullptr) {
      MatrixX<T>& dPdy_k = (*dPdy)[k];
      c.Project(y_k, R_k, &gamma_k, &dPdy_k);
    } else {
      c.Project(y_k, R_k, &gamma_k);
    }
    constraint_start += nk;
  }
}

template <typename T>
void SapConstraintsBundle<T>::CalcProjectImpulsesAndCalcConstraintsHessian(
    const VectorX<T>& y, const VectorX<T>& R, VectorX<T>* gamma,
    std::vector<MatrixX<T>>* G) const {
  ProjectImpulses(y, R, gamma, G);  // G = dPdy.

  // The regularizer Hessian is G = ∇²ℓ = d²ℓ/dvc² = dP/dy⋅R⁻¹.
  int constraint_start = 0;
  for (int k = 0; k < num_constraints(); ++k) {
    const SapConstraint<T>& c = *constraints_[k];
    const int nk = c.num_constrained_dofs();
    const auto R_k = R.segment(constraint_start, nk);
    const MatrixUpTo6<T> dPdy_k = (*G)[k];
    (*G)[k] = dPdy_k * R_k.cwiseInverse().asDiagonal();
    constraint_start += nk;
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
  const ContactProblemGraph graph = sap_problem().MakeGraph();
  cliques_permutation_ = MakeParticipatingCliquesPermutation(graph);
  velocities_permutation_ = MakeParticipatingVelocitiesPermutation(
      sap_problem(), cliques_permutation_);

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
  for (const ContactProblemGraph::Edge& e : graph.edges()) {
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
  constraints_bundle_ = std::make_unique<SapConstraintsBundle<T>>(
      std::move(J), std::move(vhat), std::move(R), std::move(constraints));
}

template <typename T>
int SapModel<T>::num_velocities() const {
  return sap_problem().num_velocities();
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
const VectorX<T>& SapModel<T>::v_star() const {
  return sap_problem().v_star();
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
  for (const auto& e : graph.edges()) {
    const int c0 = e.cliques.first();
    const int c1 = e.cliques.second();
    if (c0 >= 0 && participating_cliques[c0] < 0) {
      participating_cliques[c0] = num_participating_cliques++;
    }
    if (participating_cliques[c1] < 0)
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
void SapModel<T>::CalcDelassusDiagonalApproximation(
    const std::vector<MatrixX<T>>& A, const SapContactProblem<T>& problem,
    const ContactProblemGraph& graph, 
    const PartialPermutation& cliques_permutation,
    VectorX<T>* delassus_diagonal) const {
  DRAKE_DEMAND(delassus_diagonal != nullptr);
  DRAKE_DEMAND(delassus_diagonal->size() == problem.num_constraints());
  const int num_cliques = A.size();

  // We compute a factorization of A once so we can re-use it multiple times
  // below.
  std::vector<Eigen::LDLT<MatrixX<T>>> A_ldlt;
  A_ldlt.resize(num_cliques);
  for (int c = 0; c < num_cliques; ++c) {
    A_ldlt[c] = A[c].ldlt();
  }

  // Scan constraints in the order specified by the graph.
  const int num_constraints = problem.num_constraints();
  std::vector<MatrixX<T>> W(num_constraints);

  for (const ContactProblemGraph::Edge& e : graph.edges()) {
    for (int k : e.constraints_index) {      
      const SapConstraint<T>& constraint = sap_problem().get_constraint(k);
      const int nk = constraint.num_constrained_dofs();
      if (W[k].size() == 0) {
        // Resize and initialize to zero on the first time it gets accessed.
        W[k].resize(nk, nk);
        W[k].setZero();
      }

      // Clique 0 is always present. Add its contribution.
      {
        const int c = cliques_permutation.permuted_index(constraint.clique0());
        const MatrixX<T>& Jkc = constraint.clique0_jacobian();
        W[k] += Jkc * A_ldlt[c].solve(Jkc.transpose());
      }

      // Adds clique 1 contribution, if present.
      if (constraint.num_cliques() == 2) {
        const int c = cliques_permutation.permuted_index(constraint.clique1());
        const MatrixX<T>& Jkc = constraint.clique1_jacobian();
        W[k] += Jkc * A_ldlt[c].solve(Jkc.transpose());
      }
    }
  }

  // Compute delassus_diagonal as the rms norm of the diagonal block for the
  // k-th constraint.
  for (int k = 0; k < num_constraints; ++k) {
    (*delassus_diagonal)[k] = W[k].norm() / W[k].rows();
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
  const int non_zero_blocks_capacity = 2 * graph.num_edges();
  BlockSparseMatrixBuilder<T> builder(
      graph.num_edges(), cliques_permutation.permuted_domain_size(),
      non_zero_blocks_capacity);

  // DOFs per edge.
  // TODO: consider the graph storing "weights"; number of dofs per node
  // (clique) and number of constrained dofs per edge.
  std::vector<int> edge_dofs(graph.num_edges(), 0);
  for (int e = 0; e < graph.num_edges(); ++e) {
    const auto& edge = graph.get_edge(e);
    for (int k : edge.constraints_index) {
      const SapConstraint<T>& c = problem.get_constraint(k);
      edge_dofs[e] += c.num_constrained_dofs();
    }
  }

  // Add a block row (with one or two blocks) per edge in the graph.
  for (int block_row = 0; block_row < graph.num_edges(); ++block_row) {
    const auto& e = graph.get_edge(block_row);
    const int c0 = e.cliques.first();
    const int c1 = e.cliques.second();

    // Allocate Jacobian blocks for this edge.
    MatrixX<T> J0, J1;
    const int num_rows = edge_dofs[block_row];
    if (c0 >= 0) {
      //  && cliques_permutation.participates(c0)
      PRINT_VAR(c0);
      const int nv0 = problem.num_velocities(c0);
      J0.resize(num_rows, nv0);
    }
    DRAKE_DEMAND(c1 >= 0);
    PRINT_VAR(c1);
    const int nv1 = problem.num_velocities(c1);
    J1.resize(num_rows, nv1);

    int row_start = 0;
    for (int k : e.constraints_index) {
      const SapConstraint<T>& c = problem.get_constraint(k);
      const int nk = c.num_constrained_dofs();

      // N.B. Each edge stores its cliques as a sorted pair. However, the pair
      // of cliques in the original constraints can be in arbitrary order.
      // Therefore below we must check to what clique in the original constraint
      // the edge's cliques correspond to.

      if (c0 >= 0) {
        J0.middleRows(row_start, nk) =
            c0 == c.clique0() ? c.clique0_jacobian() : c.clique1_jacobian();
      }
      J1.middleRows(row_start, nk) =
          c1 == c.clique0() ? c.clique0_jacobian() : c.clique1_jacobian();

      row_start += nk;
    }

    if (c0 >= 0) {
      const int participating_c0 = cliques_permutation.permuted_index(c0);
      builder.PushBlock(block_row, participating_c0, J0);
    }
    const int participating_c1 = cliques_permutation.permuted_index(c1);
    builder.PushBlock(block_row, participating_c1, J1);
  }

  return builder.Build();
}

template <typename T>
void SapModel<T>::MultiplyByDynamicsMatrix(const VectorX<T>& v,
                                           VectorX<T>* p) const {
  int block_start = 0;
  for (const auto& Ab : A_) {
    const int block_size = Ab.rows();
    p->segment(block_start, block_size) =
        Ab * v.segment(block_start, block_size);
    block_start += block_size;
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
