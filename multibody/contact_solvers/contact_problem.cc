#include "drake/multibody/contact_solvers/contact_problem.h"

#include <algorithm>

#include "drake/common/sorted_pair.h"

using drake::SortedPair;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

#if 0
ContactProblemGraph ContactProblemGraph::MakeGraphOfParticipatingCliques(
    std::vector<int>* participating_cliques) const {
  std::vector<bool> clique_participates(num_cliques(), false);
  for (const auto& e : edges_) {
    if (e.cliques.first() >= 0) clique_participates[e.cliques.first()] = true;
    if (e.cliques.second() >= 0) clique_participates[e.cliques.second()] = true;
  }
  const int num_participating_cliques =
      std::count(clique_participates.begin(), clique_participates.end(), true);
  participating_cliques->resize(num_participating_cliques);

  // We build a map from participating clique index to original index in `this`
  // graph. We mark non-participating cliques with -1.
  std::vector<int> participating_clique_index(num_cliques(), -1);
  int c_participating = 0;
  for (int c = 0; c < num_cliques(); ++c) {
    if (clique_participates[c]) {
      participating_clique_index[c] = c_participating;
      (*participating_cliques)[c_participating++] = c;
    }
  }
  if (num_participating_cliques == num_cliques()) return *this;

  ContactProblemGraph participating_cliques_graph(num_participating_cliques,
                                                  num_edges());
  for (const auto& e : edges_) {
    const int c0 = e.cliques.first() >= 0
                       ? participating_clique_index[e.cliques.first()]
                       : -1;
    const int c1 = e.cliques.second() >= 0
                       ? participating_clique_index[e.cliques.second()]
                       : -1;
    Edge participating_edge({c0, c1}, e.constraints_index);
    participating_cliques_graph.AddEdge(std::move(participating_edge));
  }
  return participating_cliques_graph;
}
#endif

PartialPermutation MakeParticipatingCliquesPermutation(
    const ContactProblemGraph& graph) {
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

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

template class ::drake::multibody::contact_solvers::internal::SapContactProblem<
    double>;
