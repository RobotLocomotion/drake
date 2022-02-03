#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// Cliques are the nodes.
// ConstraintGroups are a bundle of constraints that connect two cliques.
class ContactProblemGraph {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactProblemGraph);

  struct Clique {
    int nv;  // Number of generalized velocities per clique.
  };

  struct ConstraintGroup {
    ConstraintGroup(const drake::SortedPair<int>& cliques_in,
         std::vector<int>&& constraints_in)
        : cliques(cliques_in), constraints_index(std::move(constraints_in)) {}
    ConstraintGroup(const drake::SortedPair<int>& cliques_in,
         const std::vector<int>& constraints_in)
        : cliques(cliques_in), constraints_index(constraints_in) {}
    drake::SortedPair<int> cliques;
    // TODO: Add size and constraint_size below.
    // int size;  // Total number of constraints for this edge.
    // For the i-th constraint in this edge, constraint_size[i] contains the
    // number of DOFs constrained by the k-th constraint of the problem, where k
    // = constraints_index[i].
    // std::vector<int> constraint_size
    std::vector<int> constraints_index;
  };

  // Graph for an empty model.
  ContactProblemGraph() = default;

  ContactProblemGraph(int num_cliques, int num_edges);

  int AddConstraintGroup(ConstraintGroup&& e);

  int num_cliques() const;
  int num_constraint_groups() const;
  int num_constraints() const;
  const std::vector<ConstraintGroup>& constraint_groups() const;
  const ConstraintGroup& get_constraint_group(int e) const;

 private:
  int num_cliques_{0};
  int num_constraints_{0};
  std::vector<Clique> cliques_;
  std::vector<ConstraintGroup> edges_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
