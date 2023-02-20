#include "drake/math/binomial_coefficient.h"

#include <gtest/gtest.h>

namespace drake {
namespace math {
namespace {

// Implement the more traditional recursion.
int RecursiveBinomialCoefficient(int n, int k) {
  int sum;

  if (n == 0 || k == 0) {
    sum = 1;
  } else {
    sum = RecursiveBinomialCoefficient(n - 1, k - 1) +
          RecursiveBinomialCoefficient(n - 1, k);
  }

  if ((n == 1 && k == 0) || (n == 1 && k == 1)) {
    sum = 1;
  }
  if (k > n) {
    sum = 0;
  }

  return sum;
}

GTEST_TEST(BinomialCoefficientTest, Values) {
  EXPECT_EQ(BinomialCoefficient(1, 1), 1);
  EXPECT_EQ(BinomialCoefficient(4, 2), 6);
  EXPECT_EQ(BinomialCoefficient(10, 4), 210);
  EXPECT_EQ(BinomialCoefficient(10, 9), 10);
}

GTEST_TEST(BinomialCoefficientTest, MatchRecursiveSolution) {
  EXPECT_EQ(BinomialCoefficient(0, 0), RecursiveBinomialCoefficient(0, 0));
  EXPECT_EQ(BinomialCoefficient(1, 0), RecursiveBinomialCoefficient(1, 0));
  EXPECT_EQ(BinomialCoefficient(0, 1), RecursiveBinomialCoefficient(0, 1));
  EXPECT_EQ(BinomialCoefficient(1, 1), RecursiveBinomialCoefficient(1, 1));

  EXPECT_EQ(BinomialCoefficient(3, 2), RecursiveBinomialCoefficient(3, 2));
  EXPECT_EQ(BinomialCoefficient(24, 12), RecursiveBinomialCoefficient(24, 12));
  EXPECT_EQ(BinomialCoefficient(12, 24), RecursiveBinomialCoefficient(12, 24));
}

}  // namespace
}  // namespace math
}  // namespace drake
