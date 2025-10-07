#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/multibody/contact_solvers/sap/expect_equal.h"

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
  // These constructor set up an arbitrary constraint for one and two cliques.
  TestConstraint(int clique, MatrixXd J)
      : SapConstraint<double>({clique, std::move(J)}, {}) {}

  TestConstraint(int first_clique, int second_clique, MatrixXd J_first_clique,
                 MatrixXd J_second_clique)
      : SapConstraint<double>({first_clique, std::move(J_first_clique),
                               second_clique, std::move(J_second_clique)},
                              {}) {}

 private:
  TestConstraint(const TestConstraint&) = default;

  // N.B no-op overloads to allow us compile this testing constraint. These
  // methods are only tested for specific derived classes, not in this file.
  std::unique_ptr<AbstractValue> DoMakeData(
      const double&, const Eigen::Ref<const VectorXd>&) const final {
    return nullptr;
  }
  void DoCalcData(const Eigen::Ref<const VectorXd>&,
                  AbstractValue*) const final {}
  double DoCalcCost(const AbstractValue&) const final { return 0.0; }
  void DoCalcImpulse(const AbstractValue&, EigenPtr<VectorXd>) const final {}
  void DoCalcCostHessian(const AbstractValue&, MatrixX<double>*) const final {}

  std::unique_ptr<SapConstraint<double>> DoClone() const final {
    return std::unique_ptr<TestConstraint>(new TestConstraint(*this));
  }
  std::unique_ptr<SapConstraint<double>> DoToDouble() const final {
    return this->Clone();
  }
};

GTEST_TEST(SapConstraint, SingleCliqueConstraint) {
  TestConstraint c(12, J32);
  EXPECT_EQ(c.num_constraint_equations(), 3);
  EXPECT_EQ(c.num_cliques(), 1);
  EXPECT_EQ(c.first_clique(), 12);
  EXPECT_THROW(c.second_clique(), std::exception);
  EXPECT_EQ(c.first_clique_jacobian().MakeDenseMatrix(), J32);
  EXPECT_THROW(c.second_clique_jacobian(), std::exception);
}

GTEST_TEST(SapConstraint, TwoCliquesConstraint) {
  TestConstraint c(11, 7, J32, J34);
  EXPECT_EQ(c.num_constraint_equations(), 3);
  EXPECT_EQ(c.num_cliques(), 2);
  EXPECT_EQ(c.first_clique(), 11);
  EXPECT_EQ(c.second_clique(), 7);
  EXPECT_EQ(c.first_clique_jacobian().MakeDenseMatrix(), J32);
  EXPECT_EQ(c.second_clique_jacobian().MakeDenseMatrix(), J34);
}

GTEST_TEST(SapConstraint, SingleCliqueConstraintWrongArguments) {
  // Negative clique.
  EXPECT_THROW(TestConstraint c(-1, J32), std::exception);
  // J.size() = 0.
  EXPECT_THROW(TestConstraint c(12, MatrixXd()), std::exception);
}

GTEST_TEST(SapConstraint, TwoCliquesConstraintWrongArguments) {
  // Negative first clique.
  EXPECT_THROW(TestConstraint c(-1, 7, J32, J34), std::exception);
  // Negative second clique.
  EXPECT_THROW(TestConstraint c(1, -1, J32, J34), std::exception);
  // Equal cliques.
  EXPECT_THROW(TestConstraint c(5, 5, J32, J34), std::exception);
  // J_first.rows() != J_second.rows().
  EXPECT_THROW(TestConstraint c(11, 7, J32, MatrixXd::Zero(1, 4)),
               std::exception);
}

GTEST_TEST(SapConstraint, SingleCliqueConstraintClone) {
  TestConstraint c(12, J32);
  std::unique_ptr<SapConstraint<double>> clone = c.Clone();
  ASSERT_NE(clone, nullptr);
  ExpectBaseIsEqual(c, *clone);
  std::unique_ptr<SapConstraint<double>> c_to_double = c.ToDouble();
  ASSERT_NE(c_to_double, nullptr);
  ExpectBaseIsEqual(c, *c_to_double);
}

GTEST_TEST(SapConstraint, TwoCliquesConstraintClone) {
  TestConstraint c(11, 7, J32, J34);
  std::unique_ptr<SapConstraint<double>> clone = c.Clone();
  ASSERT_NE(clone, nullptr);
  ExpectBaseIsEqual(c, *clone);
  std::unique_ptr<SapConstraint<double>> c_to_double = c.ToDouble();
  ASSERT_NE(c_to_double, nullptr);
  ExpectBaseIsEqual(c, *c_to_double);
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
