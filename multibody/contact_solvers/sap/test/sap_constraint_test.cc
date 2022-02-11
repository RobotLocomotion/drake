#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

#include <gtest/gtest.h>

using Eigen::MatrixXd;
using Eigen::Vector3d;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

// These Jacobian matrices have arbitrary values for testing. We specify the
// size of the matrix in the name, e.g. J32 is of size 3x2.
// clang-format off
const MatrixXd J32 =
    (MatrixXd(3, 2) << 2, 1,
                       1, 2,
                       1, 2).finished();

const MatrixXd J34 =
    (MatrixXd(3, 4) << 7, 1, 2, 3,
                       1, 8, 4, 5,
                       2, 4, 9, 6).finished();
// clang-format on

class TestConstraint final : public SapConstraint<double> {
 public:
  // These constructor set up an arbitrary constraint for one and two cliques.
  explicit TestConstraint(int clique)
      : SapConstraint<double>(clique, Vector3d(1., 2., 3), J32) {}

  TestConstraint(int first_clique, int second_clique)
      : SapConstraint<double>(first_clique, second_clique, Vector3d(1., 2., 3),
                              J32, J34) {}

  // N.B no-op overloads to allow us compile this testing constraint. These
  // methods are only tested for specific derived classes, not in this file.
  void Project(const Eigen::Ref<const VectorX<double>>&,
               const Eigen::Ref<const VectorX<double>>&,
               EigenPtr<VectorX<double>>, MatrixX<double>*) const final {}
  VectorX<double> CalcBiasTerm(const double&, const double&) const final {
    return Vector3d::Zero();
  }
  VectorX<double> CalcDiagonalRegularization(const double&,
                                             const double&) const final {
    return Vector3d::Zero();
  }
};

GTEST_TEST(SapConstraint, SingleCliqueConstraint) {
  TestConstraint c(12);
  EXPECT_EQ(c.num_constraint_equations(), 3);
  EXPECT_EQ(c.num_cliques(), 1);
  EXPECT_EQ(c.first_clique(), 12);
  EXPECT_THROW(c.second_clique(), std::exception);
  EXPECT_EQ(c.constraint_function(), Vector3d(1., 2., 3));
  EXPECT_EQ(c.first_clique_jacobian(), J32);
  EXPECT_THROW(c.second_clique_jacobian(), std::exception);
}

GTEST_TEST(SapConstraint, TwoCliquesConstraint) {
  TestConstraint c(11, 7);
  EXPECT_EQ(c.num_constraint_equations(), 3);
  EXPECT_EQ(c.num_cliques(), 2);
  EXPECT_EQ(c.first_clique(), 11);
  EXPECT_EQ(c.second_clique(), 7);
  EXPECT_EQ(c.constraint_function(), Vector3d(1., 2., 3));
  EXPECT_EQ(c.first_clique_jacobian(), J32);
  EXPECT_EQ(c.second_clique_jacobian(), J34);
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
