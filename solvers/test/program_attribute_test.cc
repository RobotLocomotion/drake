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

}  // namespace
}  // namespace test
}  // namespace solvers
}  // namespace drake
