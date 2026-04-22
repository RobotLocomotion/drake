#include "drake/multibody/contact_solvers/sap/partial_permutation.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/expect_throws_message.h"

using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

GTEST_TEST(PartialPermutation, EmptyPermutation) {
  PartialPermutation p;
  EXPECT_EQ(p.domain_size(), 0);
  EXPECT_EQ(p.permuted_domain_size(), 0);
}

GTEST_TEST(PartialPermutation, EmptyPermutationFromStdVector) {
  std::vector<int> permutation;
  PartialPermutation p(std::move(permutation));
  EXPECT_EQ(p.domain_size(), 0);
  EXPECT_EQ(p.permuted_domain_size(), 0);
}

// Creates a permutation of size 6 where none of the indexes participates and
// adds permuted indexes one at a time with push().
GTEST_TEST(PartialPermutation, PushElements) {
  PartialPermutation p(6);
  EXPECT_EQ(p.domain_size(), 6);
  EXPECT_EQ(p.permuted_domain_size(), 0);
  for (int i = 0; i < 6; ++i) {
    EXPECT_FALSE(p.participates(i));
  }

  EXPECT_EQ(p.push(1), 0);
  EXPECT_EQ(p.permuted_domain_size(), 1);  // permuted domain increases.
  EXPECT_EQ(p.push(3), 1);
  EXPECT_EQ(p.permuted_domain_size(), 2);  // permuted domain increases.
  EXPECT_EQ(p.push(2), 2);
  EXPECT_EQ(p.permuted_domain_size(), 3);  // permuted domain increases.
  EXPECT_EQ(p.push(3), 1);                 // already added.
  EXPECT_EQ(p.permuted_domain_size(), 3);  // No change.
  EXPECT_EQ(p.push(5), 3);
  EXPECT_EQ(p.permuted_domain_size(), 4);  // permuted domain increases.

  const std::vector<int> expected_permutation = {-1, 0, 2, 1, -1, 3};
  EXPECT_EQ(p.permutation(), expected_permutation);
}

GTEST_TEST(PartialPermutation, Construction) {
  const std::vector<int> permutation = {-1, 0, 2, 1, -1, 3};
  PartialPermutation p(permutation);
  EXPECT_EQ(p.permutation(), permutation);
  EXPECT_EQ(p.domain_size(), 6);
  EXPECT_EQ(p.permuted_domain_size(), 4);
  EXPECT_FALSE(p.participates(0));
  DRAKE_EXPECT_THROWS_MESSAGE(
      p.permuted_index(0),
      "Index .* does not participate in this permutation.");
  EXPECT_TRUE(p.participates(1));
  EXPECT_EQ(p.permuted_index(1), 0);
  EXPECT_TRUE(p.participates(2));
  EXPECT_EQ(p.permuted_index(2), 2);
  EXPECT_TRUE(p.participates(1));
  EXPECT_EQ(p.permuted_index(3), 1);
  EXPECT_FALSE(p.participates(4));
  DRAKE_EXPECT_THROWS_MESSAGE(
      p.permuted_index(4),
      "Index .* does not participate in this permutation.");
  EXPECT_EQ(p.permuted_index(5), 3);

  // Unit test inverse mapping from the domain of permuted indexes to the domain
  // of original indexes.
  EXPECT_EQ(p.domain_index(0), 1);
  EXPECT_EQ(p.domain_index(1), 3);
  EXPECT_EQ(p.domain_index(2), 2);
  EXPECT_EQ(p.domain_index(3), 5);

  // Argument index out of bounds.
  EXPECT_THROW(p.participates(6), std::exception);
  EXPECT_THROW(p.participates(-1), std::exception);
  EXPECT_THROW(p.permuted_index(6), std::exception);
  EXPECT_THROW(p.permuted_index(-1), std::exception);
  EXPECT_THROW(p.domain_index(-1), std::exception);
  EXPECT_THROW(p.domain_index(4), std::exception);
}

// Perform the permutation of a VectorXd.
GTEST_TEST(PartialPermutation, PermuteEigenVectorAndBack) {
  PartialPermutation p({0, -1, 2, 1});
  EXPECT_EQ(p.domain_size(), 4);
  EXPECT_EQ(p.permuted_domain_size(), 3);
  const VectorXd x = VectorXd::LinSpaced(p.domain_size(), 0.0, 3.0);
  VectorXd xp(p.permuted_domain_size());
  p.Apply(x, &xp);
  const VectorXd xp_expected = (VectorXd(3) << 0., 3., 2.).finished();
  EXPECT_EQ(xp, xp_expected);
  VectorXd x_back = -VectorXd::Ones(p.domain_size());
  p.ApplyInverse(xp, &x_back);
  const VectorXd x_back_expected = (VectorXd(4) << 0., -1., 2., 3.).finished();
  EXPECT_EQ(x_back, x_back_expected);
}

// Perform the permutation of a std::vector<int>.
GTEST_TEST(PartialPermutation, PermuteStdVectorAndBack) {
  PartialPermutation p({0, -1, 2, 1});
  EXPECT_EQ(p.domain_size(), 4);
  EXPECT_EQ(p.permuted_domain_size(), 3);
  const std::vector<int> x = {0, 1, 2, 3};
  std::vector<int> xp(p.permuted_domain_size());
  p.Apply(x, &xp);
  const std::vector<int> xp_expected = {0, 3, 2};
  EXPECT_EQ(xp, xp_expected);
  std::vector<int> x_back(p.domain_size(), -1);
  p.ApplyInverse(xp, &x_back);
  const std::vector<int> x_back_expected = {0, -1, 2, 3};
  EXPECT_EQ(x_back, x_back_expected);
}

GTEST_TEST(PartialPermutation, PermutedDomainIsLargerThanOriginalDomain) {
  // The input permutation is invalid since index = 4 is out-of-bounds for a
  // domain of size 4.
  DRAKE_EXPECT_THROWS_MESSAGE(
      PartialPermutation({0, -1, 4, 1}),
      "The size of the permuted domain must be smaller or equal than that of "
      "the original domain. Index 4, larger or equal than the domain size, "
      "appears in the input permutation.");
}

// Verifies the constructor throws if there are repeated indexes.
GTEST_TEST(PartialPermutation, RepeatedIndex) {
  DRAKE_EXPECT_THROWS_MESSAGE(PartialPermutation({0, -1, 2, 2}),
                              "Index 2 appears at least twice in the input "
                              "permutation. At 2 and at 3.");
}

GTEST_TEST(PartialPermutation, MissingIndex) {
  DRAKE_EXPECT_THROWS_MESSAGE(PartialPermutation({0, -1, -1, 4, 1}),
                              "Index 2 not present in the permutation. However "
                              "the maximum specified permuted index is 4.");
}

GTEST_TEST(PartialPermutation, ExtendToFullPermutation) {
  const std::vector<int> permutation = {0, -1, 2, 1};
  PartialPermutation p(permutation);
  p.ExtendToFullPermutation();
  EXPECT_EQ(p.domain_size(), 4);
  EXPECT_EQ(p.permuted_domain_size(), 4);
  EXPECT_EQ(p.permutation(), (PartialPermutation({0, 3, 2, 1})).permutation());
}

GTEST_TEST(VertexPartialPermutation, Constructor) {
  const std::vector<int> permutation = {0, -1, 2, 1};
  const VertexPartialPermutation p(permutation);

  const PartialPermutation& vertex = p.vertex();
  const PartialPermutation& dof = p.dof();
  const std::vector<int> expected_dof_permutation = {0, 1, 2, -1, -1, -1,
                                                     6, 7, 8, 3,  4,  5};

  EXPECT_EQ(vertex.domain_size(), 4);
  EXPECT_EQ(vertex.permuted_domain_size(), 3);
  EXPECT_EQ(vertex.permutation(), permutation);
  EXPECT_EQ(dof.domain_size(), 12);
  EXPECT_EQ(dof.permuted_domain_size(), 9);
  EXPECT_EQ(dof.permutation(), expected_dof_permutation);
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
