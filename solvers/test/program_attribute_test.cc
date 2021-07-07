#include "drake/solvers/program_attribute.h"

#include <string>
#include <tuple>
#include <vector>

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
  // Define some helpful abbreviations.
  const ProgramAttributes lin_cons{
      ProgramAttribute::kLinearConstraint};
  const ProgramAttributes quad_cost{
      ProgramAttribute::kQuadraticCost};
  const ProgramAttributes quad_cost_cons{
      ProgramAttribute::kQuadraticCost,
      ProgramAttribute::kQuadraticConstraint};
  const ProgramAttributes lin_cons_quad_cost{
      ProgramAttribute::kLinearConstraint,
      ProgramAttribute::kQuadraticCost};
  const ProgramAttributes quad_cost_cons_generic_cost{
      ProgramAttribute::kQuadraticCost,
      ProgramAttribute::kQuadraticConstraint,
      ProgramAttribute::kGenericCost};

  // List out all of our test cases, formatted as tuples of:
  // - required attributes,
  // - supported attributes,
  // - expected unsupported message (if any).
  std::vector<std::tuple<ProgramAttributes, ProgramAttributes, std::string>>
  cases{
    {quad_cost, quad_cost, ""},
    {quad_cost, quad_cost_cons, ""},
    {quad_cost, quad_cost_cons_generic_cost, ""},
    {quad_cost_cons, quad_cost_cons, ""},
    {quad_cost_cons, quad_cost_cons_generic_cost, ""},
    {lin_cons, quad_cost,
     "a LinearConstraint was declared but is not supported"},
    {lin_cons_quad_cost, quad_cost_cons,
     "a LinearConstraint was declared but is not supported"},
    {quad_cost_cons_generic_cost, quad_cost,
     "a GenericCost and QuadraticConstraint were declared"
     " but are not supported"},
    {quad_cost_cons_generic_cost, lin_cons,
     "a GenericCost, QuadraticCost, and QuadraticConstraint were declared"
     " but are not supported"},
  };

  // Run all of the tests.
  for (const auto& one_case : cases) {
    const auto& required = std::get<0>(one_case);
    const auto& supported = std::get<1>(one_case);
    const auto& expected_message = std::get<2>(one_case);
    const bool expected_is_supported = expected_message.empty();
    SCOPED_TRACE(to_string(required) + " vs " + to_string(supported));

    const bool is_supported_1 = AreRequiredAttributesSupported(
        required, supported);
    EXPECT_EQ(is_supported_1, expected_is_supported);

    std::string message{"this should be clobbered"};
    const bool is_supported_2 = AreRequiredAttributesSupported(
        required, supported, &message);
    EXPECT_EQ(is_supported_2, expected_is_supported);
    EXPECT_EQ(message, expected_message);
  }
}

GTEST_TEST(ProgramTypeTest, tostring) {
  EXPECT_EQ(to_string(ProgramType::kLP), "linear programming");
  EXPECT_EQ(to_string(ProgramType::kQP), "quadratic programming");
  EXPECT_EQ(to_string(ProgramType::kSOCP), "second order cone programming");
  EXPECT_EQ(to_string(ProgramType::kSDP), "semidefinite programming");
  EXPECT_EQ(to_string(ProgramType::kGP), "geometric programming");
  EXPECT_EQ(to_string(ProgramType::kCGP), "conic geometric programming");
  EXPECT_EQ(to_string(ProgramType::kMILP), "mixed-integer linear programming");
  EXPECT_EQ(to_string(ProgramType::kMIQP),
            "mixed-integer quadratic programming");
  EXPECT_EQ(to_string(ProgramType::kMISOCP),
            "mixed-integer second order cone programming");
  EXPECT_EQ(to_string(ProgramType::kMISDP),
            "mixed-integer semidefinite programming");
  EXPECT_EQ(to_string(ProgramType::kQuadraticCostConicConstraint),
            "conic-constrained quadratic programming");
  EXPECT_EQ(to_string(ProgramType::kNLP), "nonlinear programming");
  EXPECT_EQ(to_string(ProgramType::kLCP), "linear complementarity programming");
  EXPECT_EQ(to_string(ProgramType::kUnknown),
            "uncategorized mathematical programming type");
}

}  // namespace
}  // namespace test
}  // namespace solvers
}  // namespace drake
