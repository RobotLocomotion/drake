#pragma once

#include <iostream>
#include <memory>
#include <variant>
#include <vector>

#include "drake/common/eigen_types.h"
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// Represents a permutation p such that u[p(i)] = x[i].
class PartialPermutation {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PartialPermutation);

  PartialPermutation() = default;

  // The permutation is provided as an array of ints with N =
  // permutation.size(), this permutation maps integers in [0, N) to integers in
  // [0, M), with M <= N. The permutation for the i-th integer in the domain [0,
  // N) is defined by integer ip = permutation[i] in [0, M). The permutation is
  // a full permutation when M = N and all integers in [0, M) are present in
  // `permutation`. It is a "partial permutation" when M < N. An index i that is
  // not permuted is indicated with a negative entry, i.e. permutation[i] < 0.
  // For both partial and full permutations, all integers in [0, M) must be
  // present or an exception is thrown.
  //
  // Example: permutation = {-1, 0, 2, 1, -1, 3}, which indicates that P(3) = 1
  // and that P(0) is not defined or is not permuted (partial permutation).
  // For this case domain_size() = 6 and permuted_domain_size() = 4. Notice that
  // all integers in [0, 4) are present in the provided array `permutation`.
  explicit PartialPermutation(std::vector<int>&& permutation);

  // TODO: write unit test for this constructor.
  PartialPermutation(int domain_size, std::vector<int>&& inverse_permutation);

  int domain_size() const { return permutation_.size(); }
  int permuted_domain_size() const { return inverse_permutation_.size(); }

  // Returns the index to which i is permuted by this permutation. That is,
  // i_permuted = P(i).
  // @throws a std::runtime_error if i does not particate in the permutation,
  // see participates().
  int permuted_index(int i) const;

  // Returns `true` if the index i in the domain of the permutation participates
  // in the permutation.
  bool participates(int i) const { return permutation_[i] >= 0; }

  // TODO: templatize on class type, rather than scalar type. The only
  // reuirement being to have [] operator (or do it in temrs of iterators?)
  // N.B. requirement could be to suppor range iteration, when we start using
  // Eigen 3.4
  // PointerToPermutedVectorType must resolve to a pointer to a class of type
  // PermutedVectorType. Both VectorType and PermutedVectorType must implement:
  //  - VectorType::size().
  //  - VectorType::operator[](int).
  template <class VectorType, class PointerToPermutedVectorType>
  void Apply(const VectorType& x,
             PointerToPermutedVectorType x_permuted) const {
    DRAKE_DEMAND(static_cast<int>(x.size()) == domain_size());
    DRAKE_DEMAND(x_permuted != nullptr);
    DRAKE_DEMAND(static_cast<int>(x_permuted->size()) ==
                 permuted_domain_size());
    for (int i_permuted = 0; i_permuted < permuted_domain_size();
         ++i_permuted) {
      const int i = inverse_permutation_[i_permuted];
      (*x_permuted)[i_permuted] = x[i];
    }
  }

  template <class PermutedVectorType, class PointerToVectorType>
  void ApplyInverse(const PermutedVectorType& x_permuted,
                    PointerToVectorType x) const {
    DRAKE_DEMAND(static_cast<int>(x_permuted.size()) == permuted_domain_size());
    DRAKE_DEMAND(x != nullptr);
    DRAKE_DEMAND(static_cast<int>(x->size()) == domain_size());
    for (int i_permuted = 0; i_permuted < permuted_domain_size();
         ++i_permuted) {
      const int i = inverse_permutation_[i_permuted];
      (*x)[i] = x_permuted[i_permuted];
    }
  }

  // Returns permutation as a std::vector, see constructor for details.
  const std::vector<int>& permutation() const {
    return permutation_;
  }

 private:
  std::vector<int> permutation_;
  std::vector<int> inverse_permutation_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
