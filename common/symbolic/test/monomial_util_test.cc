#include "drake/common/symbolic/monomial_util.h"

#include <unordered_set>

#include "common/symbolic/_virtual_includes/monomial_util/drake/common/symbolic/monomial_util.h"
#include <gtest/gtest.h>

namespace drake {
namespace symbolic {
void CheckCalcMonomialBasisOrderUpToOne(const drake::symbolic::Variables& t) {
  // First compute the unsorted vector of monomial basis.
  const auto basis_unsort = CalcMonomialBasisOrderUpToOne(t, false);
  const int basis_size_expected = 1 << static_cast<int>(t.size());
  EXPECT_EQ(basis_unsort.rows(), basis_size_expected);
  std::unordered_set<drake::symbolic::Monomial> basis_set;
  basis_set.reserve(basis_size_expected);
  for (int i = 0; i < basis_unsort.rows(); ++i) {
    for (const drake::symbolic::Variable& ti : t) {
      EXPECT_LE(basis_unsort(i).degree(ti), 1);
    }
    basis_set.insert(basis_unsort(i));
  }
  EXPECT_EQ(basis_set.size(), basis_size_expected);

  // Now compute the sorted vector of monomial basis.
  const auto basis_sorted = CalcMonomialBasisOrderUpToOne(t, true);
  EXPECT_EQ(basis_sorted.rows(), basis_size_expected);
  for (int i = 0; i < basis_sorted.rows(); ++i) {
    EXPECT_GT(basis_set.count(basis_sorted(i)), 0);
  }
  // Make sure that basis_sorted is actually in the graded lexicographic order.
  for (int i = 0; i < basis_sorted.rows() - 1; ++i) {
    EXPECT_TRUE(GradedReverseLexOrder<std::less<Variable>>()(
        basis_sorted(i + 1), basis_sorted(i)));
  }
}

class CalcMonomialBasisTest : public ::testing::Test {
 protected:
  drake::symbolic::Variable t1_{"t1"};
  drake::symbolic::Variable t2_{"t2"};
  drake::symbolic::Variable t3_{"t3"};
  drake::symbolic::Variable t4_{"t4"};
};

TEST_F(CalcMonomialBasisTest, CalcMonomialBasisOrderUpToOne) {
  CheckCalcMonomialBasisOrderUpToOne(drake::symbolic::Variables({t1_}));
  CheckCalcMonomialBasisOrderUpToOne(drake::symbolic::Variables({t1_, t2_}));
  CheckCalcMonomialBasisOrderUpToOne(
      drake::symbolic::Variables({t1_, t2_, t3_}));
  CheckCalcMonomialBasisOrderUpToOne(
      drake::symbolic::Variables({t1_, t2_, t3_, t4_}));
}
}  // namespace symbolic
}  // namespace drake
