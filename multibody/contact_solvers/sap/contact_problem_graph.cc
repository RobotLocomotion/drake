#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

ContactProblemGraph::ContactProblemGraph(int num_cliques, int num_edges)
    : num_cliques_(num_cliques) {
  edges_.reserve(num_edges);
}

int ContactProblemGraph::AddConstraintGroup(ConstraintGroup&& e) {
  const int edge_index = edges_.size();
  num_constraints_ += e.constraints_index.size();
  edges_.emplace_back(e);
  return edge_index;
}

int ContactProblemGraph::num_cliques() const { return num_cliques_; }
int ContactProblemGraph::num_constraint_groups() const {
  return static_cast<int>(edges_.size());
}
int ContactProblemGraph::num_constraints() const { return num_constraints_; }

const std::vector<ContactProblemGraph::ConstraintGroup>& ContactProblemGraph::constraint_groups()
    const {
  return edges_;
}

const ContactProblemGraph::ConstraintGroup& ContactProblemGraph::get_constraint_group(int e) const {
  DRAKE_DEMAND(0 <= e && e < num_constraint_groups());
  return edges_[e];
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
