#include "drake/systems/primitives/transfer_function.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace systems {
namespace {

using symbolic::Monomial;
using symbolic::Polynomial;
using symbolic::RationalFunction;
using symbolic::Variable;

// This matches the code example in the class documentation.
GTEST_TEST(TransferFunctionTest, BasicContinuousTimeTest) {
  auto s = TransferFunction::s();
  TransferFunction tf(1.0 / (s + 1.0));

  auto s_var = TransferFunction::s_var();
  EXPECT_EQ(tf.H().rows(), 1);
  EXPECT_EQ(tf.H().cols(), 1);
  EXPECT_EQ(tf.time_step(), 0.0);
  EXPECT_TRUE(tf.H()(0, 0).numerator().EqualTo(Polynomial(1.0)));
  EXPECT_TRUE(tf.H()(0, 0).denominator().EqualTo(Polynomial(s_var + 1.0)));
}

// This matches the code example in the class documentation.
GTEST_TEST(TransferFunctionTest, BasicDiscreteTimeTest) {
  auto z = TransferFunction::z();
  TransferFunction tf(1.0 / (z - 0.5), 0.1);

  auto z_var = TransferFunction::z_var();
  EXPECT_EQ(tf.H().rows(), 1);
  EXPECT_EQ(tf.H().cols(), 1);
  EXPECT_EQ(tf.time_step(), 0.1);
  EXPECT_TRUE(tf.H()(0, 0).numerator().EqualTo(Polynomial(1.0)));
  EXPECT_TRUE(tf.H()(0, 0).denominator().EqualTo(Polynomial(z_var - 0.5)));
}

GTEST_TEST(TransferFunctionTest, BadInputsToConstructor) {
  // Note: DRAKE_EXPECT_THROWS_MESSAGE uses `s` as a variable, so we can't use
  // it here.
  RationalFunction my_s(
      Monomial(Variable("s")));  // Not TransferFunction::s_var().
  DRAKE_EXPECT_THROWS_MESSAGE(TransferFunction(1.0 / (my_s + 1.0)),
                              ".*H must only be a function of.*");
  RationalFunction z(
      Monomial(Variable("z")));  // Not TransferFunction::z_var().
  DRAKE_EXPECT_THROWS_MESSAGE(TransferFunction(1.0 / (z - 1.0), 0.1),
                              ".*H must only be a function of.*");
  my_s = TransferFunction::s();
  z = TransferFunction::z();
  // Using s with a time_step > 0 will throw.
  DRAKE_EXPECT_THROWS_MESSAGE(TransferFunction(1.0 / (my_s + 1.0), 0.1),
                              ".*H must only be a function of.*");
  // Using z with a time_step == 0 will throw.
  DRAKE_EXPECT_THROWS_MESSAGE(TransferFunction(1.0 / (z - 1.0), 0.0),
                              ".*H must only be a function of.*");
  // time_step < 0 will throw.
  DRAKE_EXPECT_THROWS_MESSAGE(TransferFunction(1.0 / (z - 1.0), -0.1),
                              ".*time_step >= 0.*");

  // Using a variable other than s or z in a matrix will throw.
  RationalFunction other(Monomial(Variable("other")));
  MatrixX<RationalFunction> H(2, 2);
  H << z, 1 / z, other, z;
  DRAKE_EXPECT_THROWS_MESSAGE(TransferFunction(H, 0.1),
                              ".*H must only be a function of.*");
}

}  // namespace
}  // namespace systems
}  // namespace drake
