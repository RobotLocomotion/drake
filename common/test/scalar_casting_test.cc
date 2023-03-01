/*
Shows implicit / explicit casting behaviors for scalar types.
*/

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/extract_double.h"
#include "drake/common/symbolic/expression.h"

namespace drake {
namespace {

using symbolic::Expression;

GTEST_TEST(ScalarCastingTest, Casting) {
  // From double.
  const AutoDiffXd ad = 1.0;
  EXPECT_EQ(ad, 1.0);
  const Expression sym = 1.0;
  EXPECT_EQ(sym, 1.0);

  // To double.
  // const double ad_to_double = ad;  // Compile error.
  const double ad_to_double = ExtractDoubleOrThrow(ad);
  EXPECT_EQ(ad_to_double, 1.0);
  // const double sym_to_double = sym;  // Compile error.
  const double sym_to_double = ExtractDoubleOrThrow(sym);
  EXPECT_EQ(sym_to_double, 1.0);
}

}  // namespace
}  // namespace drake
