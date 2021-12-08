#include "drake/multibody/contact_solvers/sap_contact_problem.h"

#include <iostream>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/contact_problem_graph.h"
#include "drake/multibody/contact_solvers/partial_permutation.h"
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapConstraint<T>::SapConstraint(int clique, const MatrixX<T>& J)
    : clique0_(clique), J0_(J) {
  DRAKE_DEMAND(clique >= 0);
  num_constrained_dofs_ = J.rows();
}

template <typename T>
SapConstraint<T>::SapConstraint(int clique0, int clique1, const MatrixX<T>& J0,
                                const MatrixX<T>& J1)
    : clique0_(clique0), clique1_(clique1), J0_(J0), J1_(J1) {
  DRAKE_DEMAND(clique0 >= 0);
  DRAKE_DEMAND(clique1 >= 0);
  DRAKE_DEMAND(clique0 != clique1);
  DRAKE_DEMAND(J0.rows() == J1.rows());
  num_constrained_dofs_ = J0.rows();
}

template <typename T>
int SapConstraint<T>::num_cliques() const {
  return clique1_ < 0 ? 1 : 2;
}

template <typename T>
int SapConstraint<T>::num_constrained_dofs() const {
  return num_constrained_dofs_;
}

template <typename T>
int SapConstraint<T>::clique0() const {
  return clique0_;
}

template <typename T>
int SapConstraint<T>::clique1() const {
  return clique1_;
}

template <typename T>
const MatrixX<T>& SapConstraint<T>::clique0_jacobian() const {
  return J0_;
}

template <typename T>
const MatrixX<T>& SapConstraint<T>::clique1_jacobian() const {
  return J1_;
}

template <typename T>
SapFrictionConeConstraint<T>::SapFrictionConeConstraint(int clique,
                                                        const MatrixX<T>& J,
                                                        const T& mu)
    : SapConstraint<T>(clique, J), projection_(mu) {
  DRAKE_DEMAND(mu >= 0.0);
  DRAKE_DEMAND(this->clique0_jacobian().rows() == 3);
}

template <typename T>
SapFrictionConeConstraint<T>::SapFrictionConeConstraint(int clique0,
                                                        int clique1,
                                                        const MatrixX<T>& J0,
                                                        const MatrixX<T>& J1,
                                                        const T& mu)
    : SapConstraint<T>(clique0, clique1, J0, J1), projection_(mu) {
  DRAKE_DEMAND(mu >= 0.0);
  DRAKE_DEMAND(this->clique0_jacobian().rows() == 3);
  DRAKE_DEMAND(this->clique1_jacobian().rows() == 3);
}

template <typename T>
VectorX<T> SapFrictionConeConstraint<T>::Project(
    const Eigen::Ref<const VectorX<T>>&, std::optional<MatrixX<T>> dPdy) const {
  if (dPdy) {
    DRAKE_ASSERT(dPdy->rows() == 3);
    DRAKE_ASSERT(dPdy->cols() == 3);
    dPdy->setZero();
  }
  return Vector3<T>();
}

template <typename T>
SapContactProblem<T>::SapContactProblem(std::vector<MatrixX<T>>&& A,
                                        VectorX<T>&& v_star)
    : A_(std::move(A)), v_star_(std::move(v_star)) {
  nv_ = 0;
  for (const auto& Ac : A_) {
    DRAKE_DEMAND(Ac.rows() == Ac.cols());
    PRINT_VAR(Ac.rows());
    nv_ += Ac.rows();
  }
  PRINT_VAR(nv_);
  PRINT_VAR(v_star_.size());
  DRAKE_DEMAND(v_star_.size() == nv_);
}

template <typename T>
SapContactProblem<T>::SapContactProblem(
    std::vector<MatrixX<T>>&& A, VectorX<T>&& v_star,
    std::vector<std::unique_ptr<SapConstraint<T>>>&& constraints)
    : A_(std::move(A)),
      v_star_(std::move(v_star)),
      constraints_(std::move(constraints)) {
  nv_ = 0;
  for (const auto& Ac : A_) {
    DRAKE_DEMAND(Ac.rows() == Ac.cols());
    nv_ += Ac.rows();
  }
  PRINT_VAR(nv_);
  PRINT_VAR(v_star_.size());
  DRAKE_DEMAND(v_star_.size() == nv_);
}

template <typename T>
void SapContactProblem<T>::AddConstraint(std::unique_ptr<SapConstraint<T>> c) {
  DRAKE_DEMAND(c->clique0() < num_cliques());
  DRAKE_DEMAND(c->clique1() < num_cliques());
  DRAKE_DEMAND(c->clique0_jacobian().cols() == num_velocities(c->clique0()));
  if (c->clique1() >= 0) {
    DRAKE_DEMAND(c->clique1_jacobian().cols() == num_velocities(c->clique1()));
  }
  constraints_.push_back(std::move(c));
}

template <typename T>
int SapContactProblem<T>::num_cliques() const {
  return A_.size();
}

template <typename T>
int SapContactProblem<T>::num_constraints() const {
  return constraints_.size();
}

template <typename T>
int SapContactProblem<T>::num_velocities() const {
  return nv_;
}

template <typename T>
int SapContactProblem<T>::num_velocities(int clique) const {
  DRAKE_DEMAND(0 <= clique && clique < num_cliques());
  return A_[clique].rows();
}

template <typename T>
const SapConstraint<T>& SapContactProblem<T>::get_constraint(int k) const {
  DRAKE_DEMAND(0 <= k && k < num_constraints());
  return *constraints_[k];
}

template <typename T>
const std::vector<MatrixX<T>>& SapContactProblem<T>::dynamics_matrix() const {
  return A_;
}

template <typename T>
ContactProblemGraph SapContactProblem<T>::MakeGraph() const {
  std::unordered_map<SortedPair<int>, std::vector<int>> edge_constraints;
  for (size_t k = 0; k < constraints_.size(); ++k) {
    const auto& c = constraints_[k];
    const auto cliques_pair = drake::MakeSortedPair(c->clique0(), c->clique1());
    edge_constraints[cliques_pair].push_back(k);
  }

  const int num_edges = edge_constraints.size();
  ContactProblemGraph graph(num_cliques(), num_edges);
  for (auto& e : edge_constraints) {
    graph.AddEdge(ContactProblemGraph::Edge(e.first, std::move(e.second)));
  }

  return graph;
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

template class ::drake::multibody::contact_solvers::internal::SapConstraint<
    double>;
template class ::drake::multibody::contact_solvers::internal::
    SapFrictionConeConstraint<double>;

template class ::drake::multibody::contact_solvers::internal::SapContactProblem<
    double>;
