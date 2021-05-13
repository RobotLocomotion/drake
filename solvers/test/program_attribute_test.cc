#include "drake/solvers/program_attribute.h"

#include <gtest/gtest.h>

namespace drake {
namespace solvers {
namespace test {
namespace {

GTEST_TEST(ProgramAttributeTest, ToString) {
  const ProgramAttributes attrs{
      ProgramAttribute::kGenericCost,
      ProgramAttribute::kGenericConstraint};

  EXPECT_EQ(to_string(ProgramAttribute::kGenericCost), "GenericCost");
  EXPECT_EQ(to_string(attrs),
            "{ProgramAttributes: GenericCost, GenericConstraint}");

  std::ostringstream os;
  os << ProgramAttribute::kGenericCost;
  EXPECT_EQ(os.str(), "GenericCost");
  os = std::ostringstream{};
  os << attrs;
  EXPECT_EQ(os.str(), "{ProgramAttributes: GenericCost, GenericConstraint}");
}

GTEST_TEST(ProgramAttributeTest, Supported) {
  auto dut = &AreRequiredAttributesSupported;

  const ProgramAttributes linear1{
      ProgramAttribute::kLinearConstraint};
  const ProgramAttributes quadratic1{
      ProgramAttribute::kQuadraticCost};
  const ProgramAttributes quadratic2{
      ProgramAttribute::kQuadraticCost,
      ProgramAttribute::kQuadraticConstraint};
  const ProgramAttributes mixed2{
      ProgramAttribute::kLinearConstraint,
      ProgramAttribute::kQuadraticCost};
  const ProgramAttributes mixed3{
      ProgramAttribute::kQuadraticCost,
      ProgramAttribute::kQuadraticConstraint,
      ProgramAttribute::kGenericCost};

  // First check positive cases without error messages.
  // N.B. The argument order to AreRequired... is (required, supported).
  {
    EXPECT_TRUE(dut(quadratic1, quadratic1, nullptr));
    EXPECT_TRUE(dut(quadratic1, quadratic2, nullptr));
    EXPECT_TRUE(dut(quadratic1, mixed3,     nullptr));
    EXPECT_TRUE(dut(quadratic2, quadratic2, nullptr));
    EXPECT_TRUE(dut(quadratic2, mixed3,     nullptr));
  }

  // Now check the same but with error messages.
  {
    std::string error;
    error = "error";
    EXPECT_TRUE(dut(quadratic1, quadratic1, &error));
    EXPECT_EQ(error, "");
    error = "error";
    EXPECT_TRUE(dut(quadratic1, quadratic2, &error));
    EXPECT_EQ(error, "");
    error = "error";
    EXPECT_TRUE(dut(quadratic1, mixed3,     &error));
    EXPECT_EQ(error, "");
    error = "error";
    EXPECT_TRUE(dut(quadratic2, quadratic2, &error));
    EXPECT_EQ(error, "");
    error = "error";
    EXPECT_TRUE(dut(quadratic2, mixed3,     &error));
    EXPECT_EQ(error, "");
  }

  // Now check negative cases without error messages.
  {
    EXPECT_FALSE(dut(linear1, quadratic1, nullptr));
    EXPECT_FALSE(dut(mixed2,  quadratic2, nullptr));
    EXPECT_FALSE(dut(mixed3,  quadratic1, nullptr));
    EXPECT_FALSE(dut(mixed3,  linear1,    nullptr));
  }

  // Now check the same but with error messages.
  {
    std::string error;
    error = "error";
    EXPECT_FALSE(dut(linear1, quadratic1, &error));
    EXPECT_EQ(error, "a LinearConstraint was declared but is not supported");
    error = "error";
    EXPECT_FALSE(dut(mixed2,  quadratic2, &error));
    EXPECT_EQ(error, "a LinearConstraint was declared but is not supported");
    error = "error";
    EXPECT_FALSE(dut(mixed3,  quadratic1, &error));
    EXPECT_EQ(error, "a GenericCost and QuadraticConstraint were declared "
                     "but are not supported");
    error = "error";
    EXPECT_FALSE(dut(mixed3,  linear1,    &error));
    EXPECT_EQ(error, "a GenericCost, QuadraticCost, and QuadraticConstraint "
                     "were declared but are not supported");
  }
}

}  // namespace
}  // namespace test
}  // namespace solvers
}  // namespace drake
