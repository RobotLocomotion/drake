#pragma once

#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/// For unit testing only, this function compares that all members in c1 are an
/// exact bitwise copy of members in c2, including the constraint Jacobian.
/// Tests are performed with EXPECT_EQ from <gtest/gtest.h>.
void ExpectBaseIsEqual(const SapConstraint<double>& c1,
                       const SapConstraint<double>& c2);

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
