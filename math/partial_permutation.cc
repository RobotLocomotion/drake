#include "drake/multibody/contact_solvers/sap/partial_permutation.h"

#include <algorithm>
#include <utility>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

PartialPermutation::PartialPermutation(std::vector<int> permutation)
    : permutation_(std::move(permutation)) {
  const int domain_size = permutation_.size();
  if (domain_size == 0) return;
  // Determine size of the permuted domain.
  const int permuted_domain_size =
      *std::max_element(permutation_.begin(), permutation_.end()) + 1;

  if (permuted_domain_size > domain_size) {
    throw std::logic_error(fmt::format(
        "The size of the permuted domain must be smaller or equal than that of "
        "the original domain. Index {}, larger or equal than the domain size, "
        "appears in the input permutation.",
        permuted_domain_size - 1));
  }

  // Allocate inverse permutation and indicate values that are not present with
  // an invalid (-1) index.
  inverse_permutation_.resize(permuted_domain_size, -1);
  // Fill in inverse permutation and check for valid permuted entries.
  for (int i = 0; i < domain_size; ++i) {
    const int i_permuted = permutation_[i];
    // Ignore negative entries since those correspond to indexes that do not
    // participate in the partial permutation.
    if (i_permuted >= 0) {
      if (inverse_permutation_[i_permuted] < 0) {
        inverse_permutation_[i_permuted] = i;
      } else {
        throw std::logic_error(
            fmt::format("Index {} appears at least twice in the input "
                        "permutation. At {} and at {}.",
                        i_permuted, inverse_permutation_[i_permuted], i));
      }
    }
  }

  // Verify that all indexes in [0; permuted_domain_size) were specified. This
  // corresponds to all indexes in inverse_permutation_ being positive.
  for (int i_permuted = 0; i_permuted < permuted_domain_size; ++i_permuted) {
    if (inverse_permutation_[i_permuted] < 0) {
      throw std::logic_error(
          fmt::format("Index {} not present in the permutation. However the "
                      "maximum specified permuted index is {}.",
                      i_permuted, permuted_domain_size - 1));
    }
  }
}

PartialPermutation::PartialPermutation(int domain_size) {
  permutation_.resize(domain_size, -1);
}

int PartialPermutation::push(int i) {
  DRAKE_THROW_UNLESS(0 <= i && i < domain_size());
  if (!participates(i)) {
    permutation_[i] = permuted_domain_size();
    inverse_permutation_.push_back(i);
  }
  return permuted_index(i);
}

int PartialPermutation::permuted_index(int i) const {
  DRAKE_THROW_UNLESS(0 <= i && i < domain_size());
  if (permutation_[i] < 0) {
    throw std::runtime_error(
        fmt::format("Index {} does not participate in this permutation.", i));
  }
  return permutation_[i];
}

int PartialPermutation::domain_index(int i_permuted) const {
  DRAKE_THROW_UNLESS(0 <= i_permuted && i_permuted < permuted_domain_size());
  return inverse_permutation_[i_permuted];
}

bool PartialPermutation::participates(int i) const {
  DRAKE_THROW_UNLESS(0 <= i && i < domain_size());
  return permutation_[i] >= 0;
}

void PartialPermutation::ExtendToFullPermutation() {
  for (int i = 0; i < domain_size(); ++i) {
    push(i);
  }
}

VertexPartialPermutation::VertexPartialPermutation() = default;

VertexPartialPermutation::VertexPartialPermutation(
    std::vector<int> vertex_permutation) {
  std::vector<int> dof_permutation(3 * vertex_permutation.size(), -1);
  for (int i = 0; i < ssize(vertex_permutation); ++i) {
    if (vertex_permutation[i] < 0) {
      continue;
    }
    dof_permutation[3 * i] = 3 * vertex_permutation[i];
    dof_permutation[3 * i + 1] = 3 * vertex_permutation[i] + 1;
    dof_permutation[3 * i + 2] = 3 * vertex_permutation[i] + 2;
  }
  vertex_permutation_ = PartialPermutation(std::move(vertex_permutation));
  dof_permutation_ = PartialPermutation(std::move(dof_permutation));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
