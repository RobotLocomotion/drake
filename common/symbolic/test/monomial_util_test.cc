#include "drake/common/symbolic/monomial_util.h"

#include <unordered_set>

#include <gtest/gtest.h>

namespace drake {
namespace symbolic {
void CheckCalcMonomialBasisOrderUpToOne(const drake::symbolic::Variables& t) {
  // First compute the unsorted vector of monomial basis.
  const auto basis_unsorted = CalcMonomialBasisOrderUpToOne(t, false);
  const int basis_size_expected =
      static_cast<int>(std::pow(2, static_cast<int>(t.size())));
  EXPECT_EQ(basis_unsorted.rows(), basis_size_expected);
  std::unordered_set<Monomial> basis_set;
  basis_set.reserve(basis_size_expected);
  for (int i = 0; i < basis_unsorted.rows(); ++i) {
    for (const Variable& ti : t) {
      EXPECT_LE(basis_unsorted(i).degree(ti), 1);
    }
    basis_set.insert(basis_unsorted(i));
  }
  // Make sure basis_set has the same size as basis_unsorted. Hence each element
  // inside basis_unsorted is unique.
  EXPECT_EQ(basis_set.size(), basis_size_expected);

  // Now compute the sorted vector of monomial basis.
  const auto basis_sorted = CalcMonomialBasisOrderUpToOne(t, true);
  EXPECT_EQ(basis_sorted.rows(), basis_size_expected);
  for (int i = 0; i < basis_sorted.rows(); ++i) {
    EXPECT_EQ(basis_set.count(basis_sorted(i)), 1);
  }
  // Make sure that basis_sorted is actually in the graded lexicographic order.
  for (int i = 0; i < basis_sorted.rows() - 1; ++i) {
    EXPECT_TRUE(GradedReverseLexOrder<std::less<Variable>>()(
        basis_sorted(i), basis_sorted(i + 1)));
  }
}

class CalcMonomialBasisTest : public ::testing::Test {
 protected:
  Variable t1_{"t1"};
  Variable t2_{"t2"};
  Variable t3_{"t3"};
  Variable t4_{"t4"};
};

TEST_F(CalcMonomialBasisTest, CalcMonomialBasisOrderUpToOne) {
  CheckCalcMonomialBasisOrderUpToOne(Variables({t1_}));
  CheckCalcMonomialBasisOrderUpToOne(Variables({t1_, t2_}));
  CheckCalcMonomialBasisOrderUpToOne(Variables({t1_, t2_, t3_}));
  CheckCalcMonomialBasisOrderUpToOne(Variables({t1_, t2_, t3_, t4_}));

  const Vector4<symbolic::Monomial> monomial12 =
      CalcMonomialBasisOrderUpToOne(Variables({t1_, t2_}), true);
  ASSERT_TRUE(std::less<Variable>()(t1_, t2_));
  EXPECT_EQ(monomial12[0], Monomial(t1_ * t2_));
  EXPECT_EQ(monomial12[1], Monomial(t2_));
  EXPECT_EQ(monomial12[2], Monomial(t1_));
  EXPECT_EQ(monomial12[3], Monomial(1));
}
}  // namespace symbolic
}  // namespace drake
