#include "drake/multibody/contact_solvers/sap/expect_equal.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

void ExpectBaseIsEqual(const SapConstraint<double>& c1,
                       const SapConstraint<double>& c2) {
  EXPECT_EQ(c1.num_constraint_equations(), c2.num_constraint_equations());
  EXPECT_EQ(c1.num_cliques(), c2.num_cliques());
  EXPECT_EQ(c1.first_clique(), c2.first_clique());
  EXPECT_EQ(c1.first_clique_jacobian().MakeDenseMatrix(),
            c2.first_clique_jacobian().MakeDenseMatrix());
  if (c1.num_cliques() == 2) {
    EXPECT_EQ(c1.second_clique(), c2.second_clique());
    EXPECT_EQ(c1.second_clique_jacobian().MakeDenseMatrix(),
              c2.second_clique_jacobian().MakeDenseMatrix());
  }
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
