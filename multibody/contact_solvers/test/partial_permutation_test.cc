#include "drake/multibody/contact_solvers/partial_permutation.h"

#include <numeric>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

#include <iostream>
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VEC(a)                             \
  std::cout << #a ": ";                          \
  for (const auto& e : a) std::cout << e << " "; \
  std::cout << std::endl;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

GTEST_TEST(PartialPermutation, PermutedIndex) {
  PartialPermutation p({-1, 0, 2, 1, -1, 3});
  EXPECT_EQ(p.domain_size(), 6);
  EXPECT_EQ(p.permuted_domain_size(), 4);
  EXPECT_THROW(p.permuted_index(0), std::runtime_error);
  EXPECT_EQ(p.permuted_index(1), 0);
  EXPECT_EQ(p.permuted_index(2), 2);
  EXPECT_EQ(p.permuted_index(3), 1);
  EXPECT_THROW(p.permuted_index(4), std::runtime_error);
  EXPECT_EQ(p.permuted_index(5), 3);
}

GTEST_TEST(PartialPermutation, PermuteEigenVector) {
  PartialPermutation p({0, -1, 2, 1});
  EXPECT_EQ(p.domain_size(), 4);
  EXPECT_EQ(p.permuted_domain_size(), 3);
  const VectorXd x = VectorXd::LinSpaced(p.domain_size(), 0.0, 3.0);
  VectorXd xp(p.permuted_domain_size());
  p.Apply(x, &xp);
  PRINT_VAR(x.transpose());
  PRINT_VAR(xp.transpose());
  VectorXd x_back(p.domain_size());
  p.ApplyInverse(xp, &x_back);
  PRINT_VAR(x_back.transpose());
}

GTEST_TEST(PartialPermutation, PermuteStdVector) {
  PartialPermutation p({0, -1, 2, 1});
  EXPECT_EQ(p.domain_size(), 4);
  EXPECT_EQ(p.permuted_domain_size(), 3);
  std::vector<int> x(p.domain_size());
  std::iota(x.begin(), x.end(), 1);
  std::vector<int> xp(p.permuted_domain_size());
  p.Apply(x, &xp);
  std::vector<int> x_back(p.domain_size(), -1);  
  p.ApplyInverse(xp, &x_back);
  PRINT_VEC(x);
  PRINT_VEC(xp);
  PRINT_VEC(x_back);
  EXPECT_EQ(x_back[0], x[0]);
  EXPECT_EQ(x_back[1], -1);  // This entry is left untouched by the inverse.
  EXPECT_EQ(x_back[2], x[2]);
  EXPECT_EQ(x_back[3], x[3]);
}

GTEST_TEST(PartialPermutation, PermutedDomainIsLargerThanOriginalDomain) {
  // The input permutation is invalid since index = 4 is out-of-bounds for a
  // domain of size 4.
  DRAKE_EXPECT_THROWS_MESSAGE(
      PartialPermutation p({0, -1, 4, 1}),
      "The size of the permuted domain must be smaller or equal than that of "
      "the original domian. Index 4, larger or equal than the domain size, "
      "appears in the input permutation.");
}

GTEST_TEST(PartialPermutation, RepeatedIndex) {
  DRAKE_EXPECT_THROWS_MESSAGE(PartialPermutation dut({0, -1, 2, 2}),
                              "Index 2 appears at least twice in the input "
                              "permutation. At 2 and at 3.");
}

GTEST_TEST(PartialPermutation, RepeatedIndex2) {
  DRAKE_EXPECT_THROWS_MESSAGE(PartialPermutation dut({0, 1, 2, 1}),
                              "Index 1 appears at least twice in the input "
                              "permutation. At 1 and at 3.");
}

// Verifies that the inverse of a permuted vector is the original vector.
GTEST_TEST(PartialPermutation, PermuteAndBack) {
  PartialPermutation p({0, -1, 2, 1});
  EXPECT_EQ(p.domain_size(), 4);
  EXPECT_EQ(p.permuted_domain_size(), 3);
  const VectorXd x = VectorXd::LinSpaced(p.domain_size(), 1.0, 4.0);
  VectorXd xp(p.permuted_domain_size());
  p.Apply(x, &xp);  
  VectorXd x_back = VectorXd::Constant(p.domain_size(), -1);
  p.ApplyInverse(xp, &x_back);
  EXPECT_EQ(x_back[0], x[0]);
  EXPECT_EQ(x_back[1], -1);  // This entry is left untouched by the inverse.
  EXPECT_EQ(x_back[2], x[2]);
  EXPECT_EQ(x_back[3], x[3]);
  PRINT_VAR(x.transpose());
  PRINT_VAR(xp.transpose());
  PRINT_VAR(x_back.transpose());
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
