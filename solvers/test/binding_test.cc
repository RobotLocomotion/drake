#include "drake/solvers/binding.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/cost.h"

namespace drake {
namespace solvers {
namespace test {

using drake::symbolic::test::VarEqual;

GTEST_TEST(TestBinding, TestConstraint) {
  const symbolic::Variable x1("x1");
  const symbolic::Variable x2("x2");
  const symbolic::Variable x3("x3");
  auto bb_con = std::make_shared<BoundingBoxConstraint>(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());

  // Checks if the bound variables are stored in the right order.
  Binding<BoundingBoxConstraint> binding1(
      bb_con,
      {VectorDecisionVariable<2>(x3, x1), VectorDecisionVariable<1>(x2)});
  EXPECT_EQ(binding1.GetNumElements(), 3u);
  VectorDecisionVariable<3> var1_expected(x3, x1, x2);
  for (int i = 0; i < 3; ++i) {
    EXPECT_PRED2(VarEqual, binding1.variables()(i), var1_expected(i));
  }

  // Creates a binding with a single VectorDecisionVariable.
  Binding<BoundingBoxConstraint> binding2(
      bb_con, VectorDecisionVariable<3>(x3, x1, x2));
  EXPECT_EQ(binding2.GetNumElements(), 3u);
  for (int i = 0; i < 3; ++i) {
    EXPECT_PRED2(VarEqual, binding2.variables()(i), var1_expected(i));
  }
}

GTEST_TEST(TestBinding, TestCost) {
  // Tests binding with a cost.
  const symbolic::Variable x1("x1");
  const symbolic::Variable x2("x2");
  const symbolic::Variable x3("x3");
  const VectorDecisionVariable<3> x(x1, x2, x3);
  auto cost = std::make_shared<LinearCost>(Eigen::Vector3d(1, 2, 3), 1);
  Binding<LinearCost> binding(cost, x);
  EXPECT_EQ(binding.evaluator().get(), cost.get());
  EXPECT_EQ(binding.evaluator()->num_outputs(), 1);
  EXPECT_EQ(binding.GetNumElements(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_PRED2(VarEqual, binding.variables()(i), x(i));
  }
}

class DummyEvaluator : public EvaluatorBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummyEvaluator)

  DummyEvaluator() : EvaluatorBase(2, 3) {}

 protected:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    y->resize(2);
    (*y)(0) = x(1) * x(2);
    (*y)(1) = x(0) - x(1);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    y->resize(2);
    (*y)(0) = x(1) * x(2);
    (*y)(1) = x(0) - x(1);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override {
    y->resize(2);
    (*y)(0) = x(1) * x(2);
    (*y)(1) = x(0) - x(1);
  }
};

GTEST_TEST(TestBinding, TestEvaluator) {
  // Tests binding with an evaluator.
  const symbolic::Variable x1("x1");
  const symbolic::Variable x2("x2");
  const symbolic::Variable x3("x3");
  const VectorDecisionVariable<3> x(x1, x2, x3);
  const auto evaluator = std::make_shared<DummyEvaluator>();
  Binding<DummyEvaluator> binding(evaluator, x);
  EXPECT_EQ(binding.evaluator().get(), evaluator.get());
  EXPECT_EQ(binding.evaluator()->num_outputs(), 2);
  EXPECT_EQ(binding.GetNumElements(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_PRED2(VarEqual, binding.variables()(i), x(i));
  }
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
