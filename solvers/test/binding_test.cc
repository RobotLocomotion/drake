#include "drake/solvers/binding.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/common/text_logging.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/cost.h"

namespace drake {
namespace solvers {
namespace test {

using drake::symbolic::test::VarEqual;

class TestBinding : public ::testing::Test {
 public:
  TestBinding() {}

 protected:
  const symbolic::Variable x1_{"x1"};
  const symbolic::Variable x2_{"x2"};
  const symbolic::Variable x3_{"x3"};
};

TEST_F(TestBinding, TestConstraintConstruction) {
  auto bb_con1 = std::make_shared<BoundingBoxConstraint>(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());

  // Checks if the bound variables are stored in the right order.
  Binding<BoundingBoxConstraint> binding1(
      bb_con1,
      {VectorDecisionVariable<2>(x3_, x1_), VectorDecisionVariable<1>(x2_)});
  EXPECT_EQ(binding1.GetNumElements(), 3u);
  VectorDecisionVariable<3> var1_expected(x3_, x1_, x2_);
  for (int i = 0; i < 3; ++i) {
    EXPECT_PRED2(VarEqual, binding1.variables()(i), var1_expected(i));
  }

  // Creates a binding with a single VectorDecisionVariable.
  Binding<BoundingBoxConstraint> binding2(
      bb_con1, VectorDecisionVariable<3>(x3_, x1_, x2_));
  EXPECT_EQ(binding2.GetNumElements(), 3u);
  for (int i = 0; i < 3; ++i) {
    EXPECT_PRED2(VarEqual, binding2.variables()(i), var1_expected(i));
  }
}

TEST_F(TestBinding, TestPrinting) {
  auto bb_con1 = std::make_shared<BoundingBoxConstraint>(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());

  // Checks if the bound variables are stored in the right order.
  Binding<BoundingBoxConstraint> binding1(
      bb_con1,
      {VectorDecisionVariable<2>(x3_, x1_), VectorDecisionVariable<1>(x2_)});
  bb_con1->set_description("dummy bb");
  const std::string str_expected1 =
      "BoundingBoxConstraint described as 'dummy bb'\n"
      "0 <= x3 <= 1\n"
      "0 <= x1 <= 1\n"
      "0 <= x2 <= 1\n";
  EXPECT_EQ(fmt::format("{}", binding1), str_expected1);

  // Test to_string() for LinearEqualityConstraint binding.
  Eigen::Matrix2d Aeq;
  Aeq << 1, 2, 3, 4;
  auto linear_eq_constraint =
      std::make_shared<LinearEqualityConstraint>(Aeq, Eigen::Vector2d(1, 2));
  Binding<LinearEqualityConstraint> linear_eq_binding(
      linear_eq_constraint, VectorDecisionVariable<2>(x1_, x2_));
  const std::string str_expected2 =
      "LinearEqualityConstraint\n(x1 + 2 * x2) == 1\n(3 * x1 + 4 * x2) == 2\n";
  EXPECT_EQ(fmt::format("{}", linear_eq_binding), str_expected2);
  EXPECT_EQ(linear_eq_binding.to_string(), str_expected2);

  const Eigen::Matrix2d Ain = Aeq;
  auto linear_ineq_constraint = std::make_shared<LinearConstraint>(
      Ain, Eigen::Vector2d(1, 2), Eigen::Vector2d(2, 3));
  Binding<LinearConstraint> linear_binding(
      linear_ineq_constraint, Vector2<symbolic::Variable>(x1_, x2_));
  const std::string str_expected3 =
      "LinearConstraint\n1 <= (x1 + 2 * x2) <= 2\n2 <= (3 * x1 + 4 * x2) <= "
      "3\n";
  EXPECT_EQ(fmt::format("{}", linear_binding), str_expected3);
  EXPECT_EQ(linear_binding.to_string(), str_expected3);
}

TEST_F(TestBinding, TestHash) {
  auto bb_con1 = std::make_shared<BoundingBoxConstraint>(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());

  Binding<BoundingBoxConstraint> binding1(
      bb_con1,
      {VectorDecisionVariable<2>(x3_, x1_), VectorDecisionVariable<1>(x2_)});
  EXPECT_EQ(binding1, binding1);

  // Creates a binding with a single VectorDecisionVariable.
  Binding<BoundingBoxConstraint> binding2(
      bb_con1, VectorDecisionVariable<3>(x3_, x1_, x2_));
  EXPECT_EQ(binding1, binding2);
  // Cast both binding1 and binding2 to Binding<LinearConstraint>. They should
  // be equal after casting.
  const Binding<LinearConstraint> binding1_cast =
      internal::BindingDynamicCast<LinearConstraint>(binding1);
  const Binding<LinearConstraint> binding2_cast =
      internal::BindingDynamicCast<LinearConstraint>(binding2);
  EXPECT_EQ(binding1_cast, binding2_cast);
  // binding3 has different variables.
  Binding<BoundingBoxConstraint> binding3(
      bb_con1, VectorDecisionVariable<3>(x3_, x2_, x1_));
  EXPECT_NE(binding1, binding3);
  // bb_con2 has different address from bb_con1, although they have the same
  // bounds.
  auto bb_con2 = std::make_shared<BoundingBoxConstraint>(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());
  EXPECT_TRUE(CompareMatrices(bb_con2->lower_bound(), bb_con1->lower_bound()));
  EXPECT_TRUE(CompareMatrices(bb_con2->upper_bound(), bb_con1->upper_bound()));
  Binding<BoundingBoxConstraint> binding4(
      bb_con2, VectorDecisionVariable<3>(x3_, x1_, x2_));
  EXPECT_NE(binding4, binding1);
  EXPECT_NE(binding4, binding2);

  // Test using Binding as unordered_map key.
  std::unordered_map<Binding<BoundingBoxConstraint>, int> map;
  map.emplace(binding1, 1);
  EXPECT_EQ(map.at(binding1), 1);
  EXPECT_EQ(map.at(binding2), 1);
  EXPECT_EQ(map.find(binding3), map.end());
  map.emplace(binding3, 3);
  EXPECT_EQ(map.at(binding3), 3);
}

TEST_F(TestBinding, TestCost) {
  // Tests binding with a cost.
  const VectorDecisionVariable<3> x(x1_, x2_, x3_);
  // Test a linear cost binding.
  auto cost1 = std::make_shared<LinearCost>(Eigen::Vector3d(1, 2, 3), 1);
  Binding<LinearCost> binding1(cost1, x);
  EXPECT_EQ(binding1.evaluator().get(), cost1.get());
  EXPECT_EQ(binding1.evaluator()->num_outputs(), 1);
  EXPECT_EQ(binding1.GetNumElements(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_PRED2(VarEqual, binding1.variables()(i), x(i));
  }
  EXPECT_EQ(fmt::format("{}", binding1),
            "LinearCost (1 + x1 + 2 * x2 + 3 * x3)");

  // Test a quadratic cost binding.
  auto cost2 = std::make_shared<QuadraticCost>(Eigen::Matrix2d::Identity(),
                                               Eigen::Vector2d(2, 3), 1);
  Binding<QuadraticCost> binding2(cost2, x.head<2>());
  EXPECT_EQ(fmt::format("{}", binding2),
            "QuadraticCost (1 + 2 * x1 + 3 * x2 + 0.5 * pow(x1, 2) + 0.5 * "
            "pow(x2, 2))");
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

TEST_F(TestBinding, TestEvaluator) {
  // Tests binding with an evaluator.
  const VectorDecisionVariable<3> x(x1_, x2_, x3_);
  const auto evaluator = std::make_shared<DummyEvaluator>();
  evaluator->set_description("dummy");
  Binding<DummyEvaluator> binding(evaluator, x);
  EXPECT_EQ(binding.evaluator().get(), evaluator.get());
  EXPECT_EQ(binding.evaluator()->num_outputs(), 2);
  EXPECT_EQ(binding.GetNumElements(), 3);
  EXPECT_EQ(fmt::format("{}", binding),
            "DummyEvaluator described as 'dummy' with 3 decision variables "
            "x1 x2 x3\n");
  for (int i = 0; i < 3; ++i) {
    EXPECT_PRED2(VarEqual, binding.variables()(i), x(i));
  }
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
