#include "drake/solvers/solver_type_converter.h"

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace solvers {
namespace {

// We use this as a way to ensure we hit all SolverTypes.  The switch statement
// will complain if someone adds an enumeration value without an update here.
optional<SolverType> successor(optional<SolverType> solver_type) {
  if (solver_type == nullopt) {
    return SolverType::kDReal;
  }
  switch (*solver_type) {
    case SolverType::kDReal:
      return SolverType::kEqualityConstrainedQP;
    case SolverType::kEqualityConstrainedQP:
      return SolverType::kGurobi;
    case SolverType::kGurobi:
      return SolverType::kIpopt;
    case SolverType::kIpopt:
      return SolverType::kLinearSystem;
    case SolverType::kLinearSystem:
      return SolverType::kMobyLCP;
    case SolverType::kMobyLCP:
      return SolverType::kMosek;
    case SolverType::kMosek:
      return SolverType::kNlopt;
    case SolverType::kNlopt:
      return SolverType::kScs;
    case SolverType::kScs:
      return SolverType::kSnopt;
    case SolverType::kSnopt:
      return nullopt;
  }
  DRAKE_ABORT();
}

GTEST_TEST(SolverId, RoundTrip) {
  // Iterate over all known solver types.
  int iterations = 0;
  for (auto solver_type = successor(nullopt);
       solver_type != nullopt;
       solver_type = successor(solver_type)) {
    ++iterations;

    // Convert type -> id -> type and check for equality.
    const SolverId id = SolverTypeConverter::TypeToId(*solver_type);
    const optional<SolverType> round_trip = SolverTypeConverter::IdToType(id);
    ASSERT_TRUE(round_trip != nullopt);
    EXPECT_EQ(*round_trip, *solver_type);

    // Names of the well-known IDs shouldn't be empty.
    EXPECT_FALSE(id.name().empty());
  }

  // This should track the number of SolverType values, if we add any.
  EXPECT_EQ(iterations, 10);
}

}  // namespace
}  // namespace solvers
}  // namespace drake
