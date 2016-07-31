#include "drake/core/Function.h"

#include "gtest/gtest.h"

namespace drake {
namespace {

// Tests the InputOutputRelation. Verify that linear is a polynomial.
GTEST_TEST(FunctionTest, InputOutputRelationLinearIsPolynomial) {
  EXPECT_TRUE((InputOutputRelation::isA(InputOutputRelation::Form::LINEAR,
                                        InputOutputRelation::Form::POLYNOMIAL)))
      << "linear is polynomial";
}

// Tests the InputOutputRelation. Verify that zero is arbitrary.
GTEST_TEST(FunctionTest, InputOutputRelationZeroIsArbitrary) {
  EXPECT_TRUE((InputOutputRelation::isA(InputOutputRelation::Form::ZERO,
                                        InputOutputRelation::Form::ARBITRARY)))
      << "zero is arbitrary";
}

// Verifies that the least common ancestor of the I/O relations
// AFFINE, LINEAR, AND POLYNOMIAL is polynomial.
GTEST_TEST(FunctionTest, InputOutputRelationLeastCommonAncestor) {
  EXPECT_TRUE((
      InputOutputRelation::leastCommonAncestor(
          {InputOutputRelation::Form::AFFINE, InputOutputRelation::Form::LINEAR,
           InputOutputRelation::Form::POLYNOMIAL}) ==
      InputOutputRelation::Form::POLYNOMIAL))
      << "least common ancestor should be polynomial";
}

// Verifies that compositions of I/O relations are as expected
GTEST_TEST(FunctionTest, InputOutputRelationCompositionTests) {
  InputOutputRelation g(InputOutputRelation::Form::LINEAR);
  InputOutputRelation f(InputOutputRelation::Form::POLYNOMIAL);

  EXPECT_EQ(InputOutputRelation::composeWith(g, f).form,
            InputOutputRelation::Form::POLYNOMIAL);

  EXPECT_EQ(InputOutputRelation::composeWith(f, g).form,
            InputOutputRelation::Form::POLYNOMIAL);
}

// Verify that combinations of I/O relations are as expected
GTEST_TEST(FunctionTest, InputOutputRelationCombinationTests) {
  InputOutputRelation g(InputOutputRelation::Form::LINEAR);
  InputOutputRelation f(InputOutputRelation::Form::POLYNOMIAL);

  EXPECT_EQ(InputOutputRelation::combine(g, f).form,
            InputOutputRelation::Form::POLYNOMIAL);

  EXPECT_EQ(InputOutputRelation::combine(f, g).form,
            InputOutputRelation::Form::POLYNOMIAL);
}

}  // namespace
}  // namespace drake
