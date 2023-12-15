#include "drake/common/symbolic/lex_monomial.h"

#include <gtest/gtest.h>

#include "drake/common/ssize.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace symbolic {
namespace {

using test::VarLess;

class LexMonomialTest : public ::testing::Test {
 protected:
  const Variable var_z_{"z"};
  const Variable var_y_{"y"};
  const Variable var_x_{"x"};

  std::vector<LexMonomial> monomials_;

  void SetUp() override {
    // These monomials are sorted in increasing order according to the
    // lexicographic order.
    monomials_ = {
        LexMonomial{{{var_x_, 0}, {var_y_, 0}, {var_z_, 0}}},  // 1.0
        LexMonomial{{{var_x_, 0}, {var_y_, 0}, {var_z_, 1}}},  // z
        LexMonomial{{{var_x_, 0}, {var_y_, 0}, {var_z_, 2}}},  // z^2
        LexMonomial{{{var_x_, 0}, {var_y_, 1}, {var_z_, 0}}},  // y
        LexMonomial{{{var_x_, 0}, {var_y_, 1}, {var_z_, 1}}},  // yz
        LexMonomial{{{var_x_, 0}, {var_y_, 2}, {var_z_, 0}}},  // y^2
        LexMonomial{{{var_x_, 1}, {var_y_, 0}, {var_z_, 0}}},  // x
        LexMonomial{{{var_x_, 1}, {var_y_, 0}, {var_z_, 1}}},  // xz
        LexMonomial{{{var_x_, 1}, {var_y_, 1}, {var_z_, 0}}},  // xy
        LexMonomial{{{var_x_, 2}, {var_y_, 0}, {var_z_, 0}}},  // x^2
    };

    EXPECT_PRED2(VarLess, var_y_, var_x_);
    EXPECT_PRED2(VarLess, var_z_, var_y_);
    EXPECT_PRED2(VarLess, var_z_, var_x_);
  }
};

// Tests the comparison operators.
TEST_F(LexMonomialTest, TestComparison) {
  for (int i = 0; i < ssize(monomials_); ++i) {
    EXPECT_EQ(monomials_.at(i), monomials_.at(i));
    for (int j = i + 1; j < ssize(monomials_); ++j) {
      EXPECT_LT(monomials_.at(i), monomials_.at(j));
      EXPECT_FALSE(monomials_.at(j) < monomials_.at(j));

      EXPECT_LE(monomials_.at(i), monomials_.at(j));
      EXPECT_FALSE(monomials_.at(j) <= monomials_.at(i));

      EXPECT_GT(monomials_.at(j), monomials_.at(i));
      EXPECT_FALSE(monomials_.at(i) > monomials_.at(j));

      EXPECT_GE(monomials_.at(j), monomials_.at(i));
      EXPECT_FALSE(monomials_.at(i) >= monomials_.at(j));
    }
  }
}

TEST_F(LexMonomialTest, TestGetNextMonomial) {
  for (int i = 0; i < ssize(monomials_); ++i) {
    std::unique_ptr<OrderedMonomial> next_monomial_ordered =
        monomials_.at(i).GetNextMonomial();
    LexMonomial* next_monomial =
        dynamic_cast<LexMonomial*>(next_monomial_ordered.get());
    EXPECT_EQ(monomials_.at(i).GetVariables(), next_monomial->GetVariables());
    EXPECT_EQ(monomials_.at(i).degree(var_x_) + 1,
              next_monomial->degree(var_x_));
    EXPECT_EQ(monomials_.at(i).degree(var_y_), next_monomial->degree(var_y_));
    EXPECT_EQ(monomials_.at(i).degree(var_z_), next_monomial->degree(var_z_));
  }
}

}  // namespace
}  // namespace symbolic
}  // namespace drake
