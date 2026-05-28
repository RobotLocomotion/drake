#pragma once

#include <vector>

#include "drake/math/partial_permutation.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* Describes the mapping of an ICF problem (IcfModel and constraint pools) to a
reduced problem. For each of the permutations, the domain is the domain of the
original problem, and the permuted domain is that of the reduced problem.

All permutations preserve ordering; some elements may not participate, but
those that do will be sorted in ascending order. */
struct ReducedMapping {
  /* Maps original velocity indices to reduced velocity indices. */
  math::internal::PartialPermutation velocity_permutation;
  /* Maps original clique indices to reduced clique indices. */
  math::internal::PartialPermutation clique_permutation;
  /* For each clique, maps original dof indices to reduced dof indices. The
  outer vector is indexed by clique indices of the original problem. */
  std::vector<math::internal::PartialPermutation> clique_dof_permutations;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
