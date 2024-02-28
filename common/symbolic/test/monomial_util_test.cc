#include "drake/common/symbolic/monomial_util.h"

#include <unordered_set>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

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
    EXPECT_TRUE(basis_set.contains(basis_sorted(i)));
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

GTEST_TEST(MonomialBasis, TestMultipleVariables) {
  // Test overloaded MonomialBasis function with argument
  // unordered_map<Variables, int>
  const Variable x0("x0");
  const Variable x1("x1");
  const Variable x2("x2");

  const Variable y0("y0");
  const Variable y1("y1");

  const Variable z0("z0");

  const Variable w0("w0");
  const Variable w1("w1");

  const Variables x_set({x0, x1, x2});
  const Variables y_set({y0, y1});
  const Variables z_set({z0});
  const Variables w_set({w0, w1});

  const VectorX<Monomial> monomials =
      MonomialBasis({{x_set, 2}, {y_set, 1}, {z_set, 3}, {w_set, 0}});
  // There are 10 monomials in MonomialBasis(x_set, 2)
  // 3 monomials in MonomialBasis(y_set, 1)
  // 4 monomials in MonomialBasis(z_set, 3).
  // 1 monomial  in MonomialBasis(w_set, 0).
  // So in total we should have 10 * 3 * 4 * 1 = 120 monomials.
  EXPECT_EQ(monomials.rows(), 120);
  // Compute the total degree of a monomial `m` in the variables `var`.
  auto degree_in_vars = [](const Monomial& m, const Variables& vars) {
    return std::accumulate(vars.begin(), vars.end(), 0,
                           [&m](int degree, const Variable& v) {
                             return degree + m.degree(v);
                           });
  };
  for (int i = 0; i < monomials.rows(); ++i) {
    if (i != 0) {
      EXPECT_NE(monomials(i), monomials(i - 1));
      EXPECT_TRUE(GradedReverseLexOrder<std::less<Variable>>()(monomials(i - 1),
                                                               monomials(i)));
    }
    EXPECT_LE(degree_in_vars(monomials(i), x_set), 2);
    EXPECT_LE(degree_in_vars(monomials(i), y_set), 1);
    EXPECT_LE(degree_in_vars(monomials(i), z_set), 3);
    EXPECT_LE(degree_in_vars(monomials(i), w_set), 0);
  }

  // Test overlapping variables in the input.
  DRAKE_EXPECT_THROWS_MESSAGE(
      MonomialBasis({{Variables({x0, x1}), 2}, {Variables({x0, y0}), 1}}),
      ".* x0 shows up more than once .*");
}
}  // namespace symbolic
}  // namespace drake
