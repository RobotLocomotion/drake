#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

#include <memory>

#include <gtest/gtest.h>

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TestConstraint);

  // These constructor set up an arbitrary constraint for one and two cliques.
  TestConstraint(int clique, VectorXd g, MatrixXd J)
      : SapConstraint<double>(clique, std::move(g), std::move(J)) {}

  TestConstraint(int first_clique, int second_clique, VectorXd g,
                 MatrixXd J_first_clique, MatrixXd J_second_clique)
      : SapConstraint<double>(first_clique, second_clique, std::move(g),
                              std::move(J_first_clique),
                              std::move(J_second_clique)) {}

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
  std::unique_ptr<SapConstraint<double>> Clone() const final {
    return std::make_unique<TestConstraint>(*this);
  }
};

GTEST_TEST(SapConstraint, SingleCliqueConstraint) {
  TestConstraint c(12, Vector3d(1., 2., 3), J32);
  EXPECT_EQ(c.num_constraint_equations(), 3);
  EXPECT_EQ(c.num_cliques(), 1);
  EXPECT_EQ(c.first_clique(), 12);
  EXPECT_THROW(c.second_clique(), std::exception);
  EXPECT_EQ(c.constraint_function(), Vector3d(1., 2., 3));
  EXPECT_EQ(c.first_clique_jacobian().MakeDenseMatrix(), J32);
  EXPECT_THROW(c.second_clique_jacobian(), std::exception);
}

GTEST_TEST(SapConstraint, TwoCliquesConstraint) {
  TestConstraint c(11, 7, Vector3d(1., 2., 3), J32, J34);
  EXPECT_EQ(c.num_constraint_equations(), 3);
  EXPECT_EQ(c.num_cliques(), 2);
  EXPECT_EQ(c.first_clique(), 11);
  EXPECT_EQ(c.second_clique(), 7);
  EXPECT_EQ(c.constraint_function(), Vector3d(1., 2., 3));
  EXPECT_EQ(c.first_clique_jacobian().MakeDenseMatrix(), J32);
  EXPECT_EQ(c.second_clique_jacobian().MakeDenseMatrix(), J34);
}

GTEST_TEST(SapConstraint, SingleCliqueConstraintWrongArguments) {
  // Negative clique.
  EXPECT_THROW(TestConstraint c(-1, Vector2d(1., 2.), J32), std::exception);
  // g.size() != J.rows().
  EXPECT_THROW(TestConstraint c(12, Vector2d(1., 2.), J32), std::exception);
  // g.size() = 0.
  EXPECT_THROW(TestConstraint c(12, VectorXd(), J32), std::exception);
  // J.size() = 0.
  EXPECT_THROW(TestConstraint c(12, Vector2d(1., 2.), MatrixXd()),
               std::exception);
}

GTEST_TEST(SapConstraint, TwoCliquesConstraintWrongArguments) {
  // Negative first clique.
  EXPECT_THROW(TestConstraint c(-1, 7, Vector3d(1., 2., 3), J32, J34),
               std::exception);
  // Negative second clique.
  EXPECT_THROW(TestConstraint c(1, -1, Vector3d(1., 2., 3), J32, J34),
               std::exception);
  // Equal cliques.
  EXPECT_THROW(TestConstraint c(5, 5, Vector3d(1., 2., 3), J32, J34),
               std::exception);
  // g.size() != J.rows().
  EXPECT_THROW(TestConstraint c(11, 7, Vector2d(1., 2.), J32, J34),
               std::exception);
  // g.size() = 0.
  EXPECT_THROW(TestConstraint c(11, 7, VectorXd(), J32, J34), std::exception);
  // J_first.rows() != J_second.rows().
  EXPECT_THROW(
      TestConstraint c(11, 7, Vector3d(1., 2., 3), J32, MatrixXd::Zero(1, 4)),
      std::exception);
}

GTEST_TEST(SapConstraint, SingleCliqueConstraintClone) {
  TestConstraint c(12, Vector3d(1., 2., 3), J32);
  std::unique_ptr<SapConstraint<double>> clone = c.Clone();
  EXPECT_EQ(clone->num_constraint_equations(), 3);
  EXPECT_EQ(clone->num_cliques(), 1);
  EXPECT_EQ(clone->first_clique(), 12);
  EXPECT_THROW(clone->second_clique(), std::exception);
  EXPECT_EQ(clone->constraint_function(), Vector3d(1., 2., 3));
  EXPECT_EQ(clone->first_clique_jacobian().MakeDenseMatrix(), J32);
  EXPECT_THROW(clone->second_clique_jacobian(), std::exception);
}

GTEST_TEST(SapConstraint, TwoCliquesConstraintClone) {
  TestConstraint c(11, 7, Vector3d(1., 2., 3), J32, J34);
  std::unique_ptr<SapConstraint<double>> clone = c.Clone();
  EXPECT_EQ(clone->num_constraint_equations(), 3);
  EXPECT_EQ(clone->num_cliques(), 2);
  EXPECT_EQ(clone->first_clique(), 11);
  EXPECT_EQ(clone->second_clique(), 7);
  EXPECT_EQ(clone->constraint_function(), Vector3d(1., 2., 3));
  EXPECT_EQ(clone->first_clique_jacobian().MakeDenseMatrix(), J32);
  EXPECT_EQ(clone->second_clique_jacobian().MakeDenseMatrix(), J34);
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
