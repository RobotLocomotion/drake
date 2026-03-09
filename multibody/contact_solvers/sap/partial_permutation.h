#pragma once

#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

// TODO(xuchenhan-tri): Move this class to math namespace.

// Given a set S with n elements, this class represents the permutation of S
// into a new set S' containing m (m ≤ n) non-repeated elements of S.
// That is, given S = {s₁, s₂, ⋯ ,sᵢ, ⋯, sₙ}, this class represents the
// mapping into a new set S' = {s'₁, s'₂, ⋯ ,s'ᵢₚ, ⋯ ,s'ₘ} where each s'ᵢₚ
// corresponds to a distinct element of S. The mapping is defined by the
// permutation of indexes in S into indexes in S' as ip = P(i), with i ∈ [0,n)
// and ip ∈ [0,m).
// In general we say that we have a "partial permutation" when m < n and
// otherwise we say we have a "full permutation" when m = n.
// We refer to S as the "domain", and to the number of elements contained in S
// as "domain size". Similarly, we refer to S' as the "permuted domain" and to
// the number of elements in S' as to "permuted domain size".
// We define the "inverse permutation" as the mapping from elements s'ᵢₚ in S'
// to elements sᵢ in S whenever there exist i ∈ [0,n) such that ip = P(i).
// Elements sᵢ for which no ip is defined are left unchanged.
class PartialPermutation {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PartialPermutation);

  // An empty permutation. That is, domain_size() and permuted_domain_size()
  // both return zero and Apply() and ApplyInverse() are no-ops.
  PartialPermutation() = default;

  // The permutation is provided as the vector `permutation` with size n =
  // permutation.size(). This permutation maps indexes in [0, n) to indexes in
  // [0, m), with m <= n. The permutation for the i-th integer in the domain [0,
  // n) is given by ip = permutation[i] in [0, m). An index i that is not
  // permuted is indicated with a negative entry, i.e. permutation[i] < 0. The
  // permutation is said to be a "full permutation" whenever m = n and all
  // indexes in [0, m) are present in `permutation`. It is said to be a "partial
  // permutation" when m < n. Every index in [0, m) must be present and appear
  // only once, or an exception is thrown.
  //
  // Example: permutation = {-1, 0, 2, 1, -1, 3} indicates P(1) = 0, P(2) = 2,
  // P(3) = 1 and P(5) = 3. Since their entries are negative, elements indexed
  // with 0 and 4 in the domain do not participate in the permutation. For this
  // case domain_size() = 6 and permuted_domain_size() = 4. Notice that all
  // indexes in [0, 4) are present in the provided array `permutation`.
  //
  // @throws if any entry in `permutation` is larger or equal than
  // permutation.size() (this would correspond to m > n).
  // @throws if `permutation` contains repeated entries.
  // @throws if ip = P(i) for i ∈ [0,n) does not include all indexes in [0,m),
  // i.e. there are holes in the permuted domain.
  explicit PartialPermutation(std::vector<int> permutation);

  // Constructs a partial permutation of `domain_size` and
  // permuted_domain_size() equal to zero. In other words, participates(i) =
  // false for in [0, domain_size). The permutation can be updated with further
  // calls to push().
  // @throws exception if domain_size is negative.
  explicit PartialPermutation(int domain_size);

  // If participates(i) = false, defines P(i) = permuted_domain_size() and
  // further increases the permuted domain size. If participates(i) = true, the
  // permutation does not change and this method simply returns P(i).
  // @throws exception if i is not in [0, domain_size()).
  // @returns P(i), i.e. permuted_index(i).
  int push(int i);

  // The size n of the domain.
  int domain_size() const { return permutation_.size(); }

  // The size m of the permuted domain.
  int permuted_domain_size() const { return inverse_permutation_.size(); }

  // Returns the index to which i is permuted by this permutation. That is,
  // i_permuted = P(i).
  // @throws a std::runtime_error if i does not particate in the permutation,
  // see participates().
  // @throws exception if i is not in [0, domain_size()).
  int permuted_index(int i) const;

  // Returns `true` if the index i in the domain of the permutation participates
  // in the permutation.
  // @throws exception if i is not in [0, domain_size()).
  bool participates(int i) const;

  // Returns index i such that i_permuted = P(i), i.e. the inverse mapping from
  // i_permuted to i.
  // @throws if i_permuted is not in [0, permuted_domain_size()).
  int domain_index(int i_permuted) const;

  // This method applies this permutation to the elements of x and writes them
  // into x_permuted. That is, x_permuted[permuted_index(i)] = x[i] for all
  // indexes i for which participates(i) is true.
  //
  // @pre VectorType must have value semantics while PointerToPermutedVectorType
  // must have pointer semantics. Both types must implement:
  //  - VectorType::size().
  //  - VectorType::operator[](int).
  //
  // N.B. we allow for VectorType and PointerToPermutedVectorType to be
  // completely different types to allow things like VectorType = VectorX or
  // Eigen::Ref together with PointerToPermutedVectorType = VectorX* or
  // drake::EigenPtr.
  // TODO(amcastro-tri): consider an overload that takes iterators instead.
  // Note that Eigen only supports iterators from version 3.4.
  template <class VectorType, class PointerToPermutedVectorType>
  void Apply(const VectorType& x,
             PointerToPermutedVectorType x_permuted) const {
    DRAKE_THROW_UNLESS(static_cast<int>(x.size()) == domain_size());
    DRAKE_THROW_UNLESS(x_permuted != nullptr);
    DRAKE_THROW_UNLESS(static_cast<int>(x_permuted->size()) ==
                       permuted_domain_size());
    for (int i_permuted = 0; i_permuted < permuted_domain_size();
         ++i_permuted) {
      const int i = inverse_permutation_[i_permuted];
      (*x_permuted)[i_permuted] = x[i];
    }
  }

  // This method applies this inverse permutation to the elements of x_permuted
  // and writes them into x. That is, x[i] = x_permuted[permuted_index(i)] for
  // all indexes i for which participates(i) is true.
  //
  // @pre PermutedVectorType must have value semantics while PointerToVectorType
  // must have pointer semantics. Both types must implement:
  //  - VectorType::size().
  //  - VectorType::operator[](int).
  //
  // N.B. we allow for PermutedVectorType and PointerToVectorType to be
  // completely different types to allow things like PermutedVectorType =
  // VectorX or Eigen::Ref together with PointerToVectorType = VectorX* or
  // drake::EigenPtr.
  // TODO(amcastro-tri): consider an overload that takes iterators instead.
  // Note that Eigen only supports iterators from version 3.4.
  template <class PermutedVectorType, class PointerToVectorType>
  void ApplyInverse(const PermutedVectorType& x_permuted,
                    PointerToVectorType x) const {
    DRAKE_THROW_UNLESS(static_cast<int>(x_permuted.size()) ==
                       permuted_domain_size());
    DRAKE_THROW_UNLESS(x != nullptr);
    DRAKE_THROW_UNLESS(static_cast<int>(x->size()) == domain_size());
    for (int i_permuted = 0; i_permuted < permuted_domain_size();
         ++i_permuted) {
      const int i = inverse_permutation_[i_permuted];
      (*x)[i] = x_permuted[i_permuted];
    }
  }

  // Returns permutation as a std::vector, see constructor for details.
  const std::vector<int>& permutation() const { return permutation_; }

  // Extends this partial permutation to a full permutation. No-op if this
  // partial permutation is already a full permutation. This is equivalent to
  // pushing all indexes in the domain that do not participate in the
  // permutation in order.
  void ExtendToFullPermutation();

 private:
  // Vector of size n, domain size, such that for i ∈ [0,n) ip = permutation_[i]
  // either maps to an element in [0,m), with no repetition, or it maps to a
  // negative index.
  std::vector<int> permutation_;
  // Vector of size m, permuted domain size, such that for ip in [0,m)
  // inverse_permutation_[ip] maps to elements in [0,n).
  std::vector<int> inverse_permutation_;
};

// Partial permutations for vertices in 3D.
//
// When dealing with meshes and particles in contact, we often need to permute
// vertices in 3D. The partial permutation on the vertices induces partial
// permutation of the degrees of freedom (dofs) of the vertices, and these
// permutations are often used in tandem.
//
// VertexPartialPermutation is a convenience class that encapsulates both the
// vertex and dof permutations. It is used in the context of contact problems
// where we need to permute the vertices and their associated degrees of freedom
// (dofs) in a consistent manner.
//
// In particular, this class assumes that the dofs are indexed such that
// dof_index = 3 * vertex_index + coordinate_index for each vertex_index and
// coordinate_index range in {0, 1, 2}.
class VertexPartialPermutation {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VertexPartialPermutation);

  // Constructs an empty permutation.
  VertexPartialPermutation();

  // Constructs a VertexPartialPermutation based on the given vector of
  // permutation of vertex indices.
  // @pre `vertex_permutation` is a valid input for the PartialPermutation
  // constructor.
  explicit VertexPartialPermutation(std::vector<int> vertex_permutation);

  const PartialPermutation& vertex() const { return vertex_permutation_; }

  const PartialPermutation& dof() const { return dof_permutation_; }

 private:
  PartialPermutation vertex_permutation_;
  PartialPermutation dof_permutation_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
