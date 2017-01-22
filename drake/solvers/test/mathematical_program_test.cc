#include <typeinfo>

#include "gtest/gtest.h"

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/polynomial.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_variable.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/test/mathematical_program_test_util.h"

using Eigen::Dynamic;
using Eigen::Ref;
using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using drake::solvers::detail::VecIn;
using drake::solvers::detail::VecOut;

using std::numeric_limits;

namespace drake {
namespace solvers {
namespace test {

struct Movable {
  Movable() = default;
  Movable(Movable&&) = default;
  Movable(Movable const&) = delete;
  static size_t numInputs() { return 1; }
  static size_t numOutputs() { return 1; }
  template <typename ScalarType>
  void eval(VecIn<ScalarType> const&, VecOut<ScalarType>&) const {}
};

struct Copyable {
  Copyable() = default;
  Copyable(Copyable&&) = delete;
  Copyable(Copyable const&) = default;
  static size_t numInputs() { return 1; }
  static size_t numOutputs() { return 1; }
  template <typename ScalarType>
  void eval(VecIn<ScalarType> const&, VecOut<ScalarType>&) const {}
};

struct Unique {
  Unique() = default;
  Unique(Unique&&) = delete;
  Unique(Unique const&) = delete;
  static size_t numInputs() { return 1; }
  static size_t numOutputs() { return 1; }
  template <typename ScalarType>
  void eval(VecIn<ScalarType> const&, VecOut<ScalarType>&) const {}
};
// TODO(naveenoid) : tests need to be purged of Random initializations.

GTEST_TEST(testMathematicalProgram, testAddFunction) {
  MathematicalProgram prog;
  prog.NewContinuousVariables<1>();

  Movable movable;
  prog.AddCost(std::move(movable));
  prog.AddCost(Movable());

  Copyable copyable;
  prog.AddCost(copyable);

  Unique unique;
  prog.AddCost(std::cref(unique));
  prog.AddCost(std::make_shared<Unique>());
  prog.AddCost(std::unique_ptr<Unique>(new Unique));
}


GTEST_TEST(testMathematicalProgram, BoundingBoxTest2) {
  // Test the scalar version of the bounding box constraint methods.

  MathematicalProgram prog;
  auto x1 = prog.NewContinuousVariables<2, 2>("x1");
  MatrixXDecisionVariable x2(2, 2);
  x2 = x1;
  // Four different ways to construct an equivalent constraint.
  // 1. Imposes constraint on a static-sized matrix of decision variables.
  // 2. Imposes constraint on a list of vectors of decision variables.
  // 3. Imposes constraint on a dynamic-sized matrix of decision variables.
  // 4. Imposes constraint using a vector of lower/upper bound, as compared
  //    to the previous three cases which use a scalar lower/upper bound.
  auto constraint1 = prog.AddBoundingBoxConstraint(0, 1, x1);
  auto constraint2 =
      prog.AddBoundingBoxConstraint(0, 1, {x1.col(0), x1.col(1)});
  auto constraint3 = prog.AddBoundingBoxConstraint(0, 1, x2);
  auto constraint4 = prog.AddBoundingBoxConstraint(Eigen::Vector4d::Zero(),
                                                   Eigen::Vector4d::Ones());

  // Checks that the bound variables are correct.
  for (const auto& binding : prog.bounding_box_constraints()) {
    EXPECT_EQ(binding.GetNumElements(), 4u);
    VectorDecisionVariable<4> x_expected;
    x_expected << x1(0, 0), x1(1, 0), x1(0, 1), x1(1, 1);
    for (int i = 0; i < 4; ++i) {
      EXPECT_EQ(binding.variables()(i), x_expected(i));
    }
  }
  EXPECT_TRUE(
      CompareMatrices(constraint1->lower_bound(), constraint2->lower_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint2->lower_bound(), constraint3->lower_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint3->lower_bound(), constraint4->lower_bound()));

  EXPECT_TRUE(
      CompareMatrices(constraint1->upper_bound(), constraint2->upper_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint2->upper_bound(), constraint3->upper_bound()));
  EXPECT_TRUE(
      CompareMatrices(constraint3->upper_bound(), constraint4->upper_bound()));
}

/* A generic cost derived from Constraint class. This is meant for testing
 * adding a cost to optimization program, and the cost is in the form of a
 * derived class of Constraint.
 */
class GenericTrivialCost1 : public Constraint {
 public:
  GenericTrivialCost1()
      : Constraint(1, 3, Vector1d(std::numeric_limits<double>::infinity()),
                   Vector1d(std::numeric_limits<double>::infinity())),
        private_val_(2) {}

 protected:
  void DoEval(const Ref<const Eigen::VectorXd>& x, VectorXd& y) const override {
    y.resize(1);
    y(0) = x(0) * x(1) + x(2) / x(0) * private_val_;
  }

  void DoEval(const Ref<const TaylorVecXd>& x, TaylorVecXd& y) const override {
    y.resize(1);
    y(0) = x(0) * x(1) + x(2) / x(0) * private_val_;
  }

 private:
  // Add a private data member to make sure no slicing on this class, derived
  // from Constraint.
  double private_val_{0};
};

/* A generic cost. This class is meant for testing adding a cost to the
 * optimization program, by calling `MathematicalProgram::MakeCost` to
 * convert this class to a ConstraintImpl object.
 *
 */
class GenericTrivialCost2 {
 public:
  static size_t numInputs() { return 2; }
  static size_t numOutputs() { return 1; }

  template <typename ScalarType>
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  void eval(VecIn<ScalarType> const& x, VecOut<ScalarType>& y) const {
    DRAKE_ASSERT(static_cast<size_t>(x.rows()) == numInputs());
    DRAKE_ASSERT(static_cast<size_t>(y.rows()) == numOutputs());
    y(0) = x(0) * x(0) - x(1) * x(1) + 2;
  }
};

// Verifies if the added cost evaluates the same as the original cost.
// This function is supposed to test these costs added as a derived class
// from Constraint.
void VerifyAddedCost1(const MathematicalProgram& prog,
                      const std::shared_ptr<Constraint>& cost,
                      const Eigen::Ref<const Eigen::VectorXd>& x_value,
                      int num_generic_costs_expected) {
  EXPECT_EQ(static_cast<int>(prog.generic_costs().size()),
            num_generic_costs_expected);
  Eigen::VectorXd y, y_expected;
  prog.generic_costs().back().constraint()->Eval(x_value, y);
  cost->Eval(x_value, y_expected);
  EXPECT_TRUE(CompareMatrices(y, y_expected));
}

// Verifies if the added cost evaluates the same as the original cost.
// This function is supposed to test these costs added by converting
// a class to ConstraintImpl through MakeCost.
void VerifyAddedCost2(const MathematicalProgram& prog,
                      const GenericTrivialCost2& cost,
                      const std::shared_ptr<Constraint>& returned_cost,
                      const Eigen::Ref<const Eigen::Vector2d>& x_value,
                      int num_generic_costs_expected) {
  EXPECT_EQ(static_cast<int>(prog.generic_costs().size()),
            num_generic_costs_expected);
  Eigen::VectorXd y(1), y_expected(1), y_returned;
  prog.generic_costs().back().constraint()->Eval(x_value, y);
  cost.eval<double>(x_value, y_expected);
  EXPECT_TRUE(CompareMatrices(y, y_expected));
  returned_cost->Eval(x_value, y_returned);
  EXPECT_TRUE(CompareMatrices(y, y_returned));
}

GTEST_TEST(testMathematicalProgram, AddCostTest) {
  // Test if the costs are added correctly.

  // There are ways to add a generic cost
  // 1. Add Binding<Constraint>
  // 2. Add shared_ptr<Constraint> on a VectorDecisionVariable object.
  // 3. Add shared_ptr<Constraint> on a VariableRefList.
  // 4. Add a ConstraintImpl object on a VectorDecisionVariable object.
  // 4. Add a ConstraintImpl object on a VariableRefList object.
  // 5. Add a unique_ptr of object that can be converted to a ConstraintImpl
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  auto y = prog.NewContinuousVariables<2>("y");
  // No cost yet.
  int num_generic_costs = 0;
  EXPECT_EQ(static_cast<int>(prog.generic_costs().size()),
            num_generic_costs);
  EXPECT_EQ(prog.linear_costs().size(), 0u);

  std::shared_ptr<Constraint> generic_trivial_cost1 =
      std::make_shared<GenericTrivialCost1>();

  // Adds Binding<Constraint>
  prog.AddCost(Binding<Constraint>(
      generic_trivial_cost1, VectorDecisionVariable<3>(x(0), x(1), y(1))));
  ++num_generic_costs;
  VerifyAddedCost1(prog, generic_trivial_cost1, Eigen::Vector3d(1, 3, 5),
                   num_generic_costs);

  // Adds a std::shared_ptr<Constraint> on a VectorDecisionVariable object.
  prog.AddCost(generic_trivial_cost1,
               VectorDecisionVariable<3>(x(0), x(1), y(1)));
  ++num_generic_costs;
  VerifyAddedCost1(prog, generic_trivial_cost1, Eigen::Vector3d(1, 2, 3),
                   num_generic_costs);

  // Adds a std::shared_ptr<Constraint> on a VariableRefList object.
  prog.AddCost(generic_trivial_cost1, {x, y.tail<1>()});
  ++num_generic_costs;
  VerifyAddedCost1(prog, generic_trivial_cost1, Eigen::Vector3d(2, 3, 4),
                   num_generic_costs);

  GenericTrivialCost2 generic_trivial_cost2;

  // Add an object that can be converted to a ConstraintImpl object on a
  // VectorDecisionVariable object.
  auto returned_cost3 = prog.AddCost(generic_trivial_cost2,
                                     VectorDecisionVariable<2>(x(0), y(1)));
  ++num_generic_costs;
  VerifyAddedCost2(prog, generic_trivial_cost2, returned_cost3,
                   Eigen::Vector2d(1, 2), num_generic_costs);

  // Add an object that can be converted to a ConstraintImpl object on a
  // VariableRefList object.
  auto returned_cost4 =
      prog.AddCost(generic_trivial_cost2, {x.head<1>(), y.tail<1>()});
  ++num_generic_costs;
  VerifyAddedCost2(prog, generic_trivial_cost2, returned_cost4,
                   Eigen::Vector2d(1, 2), num_generic_costs);
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolic1) {
  // Add Linear Constraint: -10 <= 3 - 5*x0 + 10*x2 - 7*y1 <= 10
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(3, "x");
  auto y = prog.NewContinuousVariables(3, "y");
  const symbolic::Expression e{3 - 5 * x(0) + 10 * x(2) - 7 * y(1)};
  const double lb{-10};
  const double ub{+10};
  const auto binding = prog.AddLinearConstraint(e, lb, ub);

  // Check if the binding includes the correct linear constraint.
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const auto constraint_ptr = binding.constraint();
  EXPECT_EQ(constraint_ptr->num_constraints(), 1u);
  const symbolic::Expression Ax{(constraint_ptr->A() * var_vec)(0, 0)};
  const symbolic::Expression lb_in_ctr{constraint_ptr->lower_bound()[0]};
  const symbolic::Expression ub_in_ctr{constraint_ptr->upper_bound()[0]};
  EXPECT_TRUE((e - lb).EqualTo(Ax - lb_in_ctr));
  EXPECT_TRUE((e - ub).EqualTo(Ax - ub_in_ctr));
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolic2) {
  // Add Linear Constraint: -10 <= x0 <= 10
  // Note that this constraint is a bounding-box constraint which is a sub-class
  // of linear-constraint.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(3, "x");
  const symbolic::Expression e{x(0)};
  const auto binding = prog.AddLinearConstraint(e, -10, 10);

  // Check that the constraint in the binding is of BoundingBoxConstraint by
  // using dynamic_pointer_cast.
  const std::shared_ptr<BoundingBoxConstraint> constraint_ptr{
      std::dynamic_pointer_cast<BoundingBoxConstraint>(binding.constraint())};
  EXPECT_TRUE(constraint_ptr != nullptr);
  EXPECT_EQ(constraint_ptr->num_constraints(), 1u);

  // Check if the binding includes the correct linear constraint.
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const symbolic::Expression Ax{(constraint_ptr->A() * var_vec)(0, 0)};
  const symbolic::Expression lb_in_ctr{constraint_ptr->lower_bound()[0]};
  const symbolic::Expression ub_in_ctr{constraint_ptr->upper_bound()[0]};
  EXPECT_TRUE((e - -10).EqualTo(Ax - lb_in_ctr));
  EXPECT_TRUE((e - 10).EqualTo(Ax - ub_in_ctr));
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolic3) {
  // Add Linear Constraints
  //     3 <=  3 - 5*x0 +      + 10*x2        - 7*y1        <= 9
  //   -10 <=                       x2                      <= 10
  //    -7 <= -5 + 2*x0 + 3*x2         + 3*y0 - 2*y1 + 6*y2 <= 12
  //     2 <=                     2*x2                      <= 3
  //
  // Note: the second constraint, -10 <= x2 <= 10 is actually a bounding-box
  // constraint but We still process the four symbolic-constraints into a
  // single linear-constraint whose coefficient matrix is the following.
  //
  //         [-5 0 10 0 -7 0]
  //         [ 0 0  1 0  0 0]
  //         [ 2 3  0 3 -2 6]
  //         [ 0 0  2 0  0 0]
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(3, "x");
  auto y = prog.NewContinuousVariables(3, "y");
  Matrix<symbolic::Expression, 4, 1> M_e;
  Vector4d M_lb;
  Vector4d M_ub;

  // clang-format off
  M_e  <<  3 - 5 * x(0) + 10 * x(2) - 7 * y(1),
      +x(2),
      -5 + 2 * x(0) + 3 * x(2) + 3 * y(0) - 2 * y(1) + 6 * y(2),
      2 * x(2);
  M_lb <<  3,
      -10,
      -7,
       2;
  M_ub << -7,
      10,
      12,
      3;
  // clang-format on

  // Check if the binding includes the correct linear constraint.
  const auto binding = prog.AddLinearConstraint(M_e, M_lb, M_ub);
  const VectorXDecisionVariable& var_vec{binding.variables()};
  const auto constraint_ptr = binding.constraint();
  EXPECT_EQ(constraint_ptr->num_constraints(), 4u);
  const auto Ax = constraint_ptr->A() * var_vec;
  const auto lb_in_ctr = constraint_ptr->lower_bound();
  const auto ub_in_ctr = constraint_ptr->upper_bound();

  EXPECT_EQ(M_e - M_lb, Ax - lb_in_ctr);
  EXPECT_EQ(M_e - M_ub, Ax - ub_in_ctr);
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolic4) {
  // Check the linear constraint 2  <= 2 * x <= 4.
  // Note: this is a bounding box constraint
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  const symbolic::Expression e(2 * x(1));
  const auto& binding = prog.AddLinearConstraint(e, 2, 4);

  EXPECT_TRUE(prog.linear_constraints().empty());
  EXPECT_EQ(prog.bounding_box_constraints().size(), 1u);
  EXPECT_EQ(prog.bounding_box_constraints().back().constraint(),
            binding.constraint());
  EXPECT_EQ(prog.bounding_box_constraints().back().variables(),
            binding.variables());
  EXPECT_EQ(binding.variables(), VectorDecisionVariable<1>(x(1)));
  EXPECT_TRUE(
      CompareMatrices(binding.constraint()->lower_bound(), Vector1d(1)));
  EXPECT_TRUE(
      CompareMatrices(binding.constraint()->upper_bound(), Vector1d(2)));
}

GTEST_TEST(testMathematicalProgram, AddLinearConstraintSymbolic5) {
  // Check the linear constraint 2  <= -2 * x <= 4.
  // Note: this is a bounding box constraint
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>("x");
  const symbolic::Expression e(-2 * x(1));
  const auto& binding = prog.AddLinearConstraint(e, 2, 4);

  EXPECT_TRUE(prog.linear_constraints().empty());
  EXPECT_EQ(prog.bounding_box_constraints().size(), 1u);
  EXPECT_EQ(prog.bounding_box_constraints().back().constraint(),
            binding.constraint());
  EXPECT_EQ(prog.bounding_box_constraints().back().variables(),
            binding.variables());
  EXPECT_EQ(binding.variables(), VectorDecisionVariable<1>(x(1)));
  EXPECT_TRUE(
      CompareMatrices(binding.constraint()->lower_bound(), Vector1d(-2)));
  EXPECT_TRUE(
      CompareMatrices(binding.constraint()->upper_bound(), Vector1d(-1)));
}

namespace {
void CheckParsedSymbolicLorentzConeConstraint(
    MathematicalProgram* prog,
    const Eigen::Ref<
        const Eigen::Matrix<symbolic::Expression, Eigen::Dynamic, 1>>& e) {
  const auto& binding1 = prog->AddLorentzConeConstraint(e);
  const auto& binding2 = prog->lorentz_cone_constraints().back();

  EXPECT_EQ(binding1.constraint(), binding2.constraint());
  EXPECT_EQ(binding1.constraint()->A() * binding1.variables() +
                binding1.constraint()->b(),
            e);
  EXPECT_EQ(binding2.constraint()->A() * binding2.variables() +
                binding2.constraint()->b(),
            e);
}

void CheckParsedSymbolicRotatedLorentzConeConstraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& e) {
  const auto& binding1 = prog->AddRotatedLorentzConeConstraint(e);
  const auto& binding2 = prog->rotated_lorentz_cone_constraints().back();

  EXPECT_EQ(binding1.constraint(), binding2.constraint());
  EXPECT_EQ(binding1.constraint()->A() * binding1.variables() +
                binding1.constraint()->b(),
            e);
  EXPECT_EQ(binding2.constraint()->A() * binding2.variables() +
                binding2.constraint()->b(),
            e);
}
}  // namespace

GTEST_TEST(testMathematicalProgram, AddSymbolicLorentzConeConstraint1) {
  // Add Lorentz cone constraint:
  // x is in Lorentz cone
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  Matrix<symbolic::Expression, 3, 1> e;
  e << 1 * x(0), 1.0 * x(1), 1.0 * x(2);
  CheckParsedSymbolicLorentzConeConstraint(&prog, e);
}

GTEST_TEST(testMathematicalProgram, AddSymbolicLorentzConeConstraint2) {
  // Add Lorentz cone constraint:
  // x + [1, 2, 0] is in Lorentz cone.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  Matrix<symbolic::Expression, 3, 1> e;
  e << x(0) + 1, x(1) + 2, +x(2);
  CheckParsedSymbolicLorentzConeConstraint(&prog, e);
}

GTEST_TEST(testMathematicalProgram, AddSymbolicLorentzConeConstraint3) {
  // Add Lorentz cone constraint:
  // [2 * x(0) + 3 * x(2)]
  // [  - x(0) + 2 * x(2)]    is in Lorentz cone
  // [               x(2)]
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  Matrix<symbolic::Expression, 3, 1> e;
  // clang-format on
  e << 2 * x(0) + 3 * x(2), -x(0) + 2 * x(2), +x(2);
  // clang-format off;
  CheckParsedSymbolicLorentzConeConstraint(&prog, e);
}

GTEST_TEST(testMathematicalProgram, AddSymbolicLorentzConeConstraint4) {
  // Add Lorentz cone constraint:
  // [ 2 * x(0) + 3 * x(1) +            5]
  // [ 4 * x(0)            + 4 * x(2) - 7]
  // [                                 10]
  // [                       2 * x(2)    ]
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  Matrix<symbolic::Expression, 4, 1> e;
  // clang-format off
  e << 2 * x(0) + 3 * x(1) + 5,
       4 * x(0) + 4 * x(2) - 7,
       10,
       2 * x(2);

  // clang-format on
  CheckParsedSymbolicLorentzConeConstraint(&prog, e);
}

GTEST_TEST(testMathematicalProgram, AddSymbolicRotatedLorentzConeConstraint1) {
  // Add rotated Lorentz cone constraint:
  // x is in the rotated Lorentz cone constraint.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  Matrix<symbolic::Expression, 3, 1> e;
  e << +x(0), +x(1), +x(2);
  CheckParsedSymbolicRotatedLorentzConeConstraint(&prog, e);
}

GTEST_TEST(testMathematicalProgram, AddSymbolicRotatedLorentzConeConstraint2) {
  // Add rotated Lorentz cone constraint:
  // [x(0) + 2 * x(2)]
  // [x(0)           ] is in the rotated Lorentz cone
  // [           x(2)]
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>("x");
  Matrix<symbolic::Expression, 3, 1> e;
  e << x(0) + 2 * x(2), +x(0), +x(2);
  CheckParsedSymbolicRotatedLorentzConeConstraint(&prog, e);
}

GTEST_TEST(testMathematicalProgram, AddSymbolicRotatedLorentzConeConstraint3) {
  // Add rotated Lorentz cone constraint:
  // [x(0) + 1]
  // [x(1) + 2] is in the rotated Lorentz cone
  // [x(2)    ]
  // [x(3) - 1]
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<4>("x");
  Matrix<symbolic::Expression, 4, 1> e;
  e << x(0) + 1, x(1) + 2, +x(2), x(3) - 1;
  CheckParsedSymbolicRotatedLorentzConeConstraint(&prog, e);
}

GTEST_TEST(testMathematicalProgram, AddSymbolicRotatedLorentzConeConstraint4) {
  // Add rotated Lorentz cone constraint:
  // [2 * x(0) + 3 * x(2) + 3]
  // [    x(0) - 4 * x(2)    ] is in the rotated Lorentz cone
  // [           2 * x(2)    ]
  // [3 * x(0)            + 1]
  // [                      4]
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<4>("x");
  Matrix<symbolic::Expression, 5, 1> e;
  e << 2 * x(0) + 3 * x(2) + 3, x(0) - 4 * x(2), 2 * x(2), 3 * x(0) + 1, 4;
  CheckParsedSymbolicRotatedLorentzConeConstraint(&prog, e);
}

//
// Test that linear polynomial constraints get turned into linear constraints.
GTEST_TEST(testMathematicalProgram, linearPolynomialConstraint) {
  const Polynomiald x("x");
  MathematicalProgram problem;
  static const double kEpsilon = 1e-7;
  const auto x_var = problem.NewContinuousVariables(1);
  const std::vector<Polynomiald::VarType> var_mapping = {x.GetSimpleVariable()};
  std::shared_ptr<Constraint> resulting_constraint =
      problem.AddPolynomialConstraint(VectorXPoly::Constant(1, x), var_mapping,
                                      Vector1d::Constant(2),
                                      Vector1d::Constant(2));
  // Check that the resulting constraint is a LinearConstraint.
  EXPECT_NE(dynamic_cast<LinearConstraint*>(resulting_constraint.get()),
            nullptr);
  // Check that it gives the correct answer as well.
  problem.SetInitialGuessForAllVariables(drake::Vector1d(0));
  RunNonlinearProgram(problem, [&]() {
    EXPECT_NEAR(problem.GetSolution(x_var(0)), 2, kEpsilon);
  });
}

// The current windows CI build has no solver for generic constraints.  The
// DISABLED_ logic below ensures that we still at least get compile-time
// checking of the test and resulting template instantiations.
#if !defined(WIN32) && !defined(WIN64)
#define POLYNOMIAL_CONSTRAINT_TEST_NAME polynomialConstraint
#else
#define POLYNOMIAL_CONSTRAINT_TEST_NAME DISABLED_polynomialConstraint
#endif

// Simple test of polynomial constraints.
GTEST_TEST(testMathematicalProgram, POLYNOMIAL_CONSTRAINT_TEST_NAME) {
  static const double kInf = numeric_limits<double>::infinity();
  // Generic constraints in nlopt require a very generous epsilon.
  static const double kEpsilon = 1e-4;

  // Given a degenerate polynomial, get the trivial solution.
  {
    const Polynomiald x("x");
    MathematicalProgram problem;
    const auto x_var = problem.NewContinuousVariables(1);
    const std::vector<Polynomiald::VarType> var_mapping = {
        x.GetSimpleVariable()};
    problem.AddPolynomialConstraint(VectorXPoly::Constant(1, x), var_mapping,
                                    Vector1d::Constant(2),
                                    Vector1d::Constant(2));
    problem.SetInitialGuessForAllVariables(drake::Vector1d::Zero());
    RunNonlinearProgram(problem, [&]() {
      EXPECT_NEAR(problem.GetSolution(x_var(0)), 2, kEpsilon);
      // TODO(ggould-tri) test this with a two-sided constraint, once
      // the nlopt wrapper supports those.
    });
  }

  // Given a small univariate polynomial, find a low point.
  {
    const Polynomiald x("x");
    const Polynomiald poly = (x - 1) * (x - 1);
    MathematicalProgram problem;
    const auto x_var = problem.NewContinuousVariables(1);
    const std::vector<Polynomiald::VarType> var_mapping = {
        x.GetSimpleVariable()};
    problem.AddPolynomialConstraint(VectorXPoly::Constant(1, poly), var_mapping,
                                    Eigen::VectorXd::Zero(1),
                                    Eigen::VectorXd::Zero(1));
    problem.SetInitialGuessForAllVariables(drake::Vector1d::Zero());
    RunNonlinearProgram(problem, [&]() {
      EXPECT_NEAR(problem.GetSolution(x_var(0)), 1, 0.2);
      EXPECT_LE(poly.EvaluateUnivariate(problem.GetSolution(x_var(0))),
                kEpsilon);
    });
  }

  // Given a small multivariate polynomial, find a low point.
  {
    const Polynomiald x("x");
    const Polynomiald y("y");
    const Polynomiald poly = (x - 1) * (x - 1) + (y + 2) * (y + 2);
    MathematicalProgram problem;
    const auto xy_var = problem.NewContinuousVariables(2);
    const std::vector<Polynomiald::VarType> var_mapping = {
        x.GetSimpleVariable(), y.GetSimpleVariable()};
    problem.AddPolynomialConstraint(VectorXPoly::Constant(1, poly), var_mapping,
                                    Eigen::VectorXd::Zero(1),
                                    Eigen::VectorXd::Zero(1));
    problem.SetInitialGuessForAllVariables(Eigen::Vector2d::Zero());
    RunNonlinearProgram(problem, [&]() {
      EXPECT_NEAR(problem.GetSolution(xy_var(0)), 1, 0.2);
      EXPECT_NEAR(problem.GetSolution(xy_var(1)), -2, 0.2);
      std::map<Polynomiald::VarType, double> eval_point = {
          {x.GetSimpleVariable(), problem.GetSolution(xy_var(0))},
          {y.GetSimpleVariable(), problem.GetSolution(xy_var(1))}};
      EXPECT_LE(poly.EvaluateMultivariate(eval_point), kEpsilon);
    });
  }

  // Given two polynomial constraints, satisfy both.
  {
    // (x^4 - x^2 + 0.2 has two minima, one at 0.5 and the other at -0.5;
    // constrain x < 0 and EXPECT that the solver finds the negative one.)
    const Polynomiald x("x");
    const Polynomiald poly = x * x * x * x - x * x + 0.2;
    MathematicalProgram problem;
    const auto x_var = problem.NewContinuousVariables(1);
    problem.SetInitialGuess(x_var, Vector1d::Constant(-0.1));
    const std::vector<Polynomiald::VarType> var_mapping = {
        x.GetSimpleVariable()};
    VectorXPoly polynomials_vec(2, 1);
    polynomials_vec << poly, x;
    problem.AddPolynomialConstraint(polynomials_vec, var_mapping,
                                    Eigen::VectorXd::Constant(2, -kInf),
                                    Eigen::VectorXd::Zero(2));
    RunNonlinearProgram(problem, [&]() {
      EXPECT_NEAR(problem.GetSolution(x_var(0)), -0.7, 0.2);
      EXPECT_LE(poly.EvaluateUnivariate(problem.GetSolution(x_var(0))),
                kEpsilon);
    });
  }
}

//
// Test how an unconstrained QP is dispatched and solved:
//   - on the problem (x1 - 1)^2 + (x2 - 1)^2, with a min at
//     at (x1=1, x2=1).
//   - on the same problem plus the additional problem
//     (2*x2 - 5)^2 + (2*x3 - 2)^2, which, when combined
//     with the first problem, has min at (x1=1, x2=2, x3=1)
// The first case tests a single quadratic cost, and the
// second case tests multiple quadratic costs affecting
// different variable views. All fall under the
// umbrella of the Equality Constrained QP Solver.
GTEST_TEST(testMathematicalProgram, testUnconstrainedQPDispatch) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  MatrixXd Q(2, 2);
  // clang-format off
  Q << 1.0, 0.0,
       0.0, 1.0;
  // clang-format on
  VectorXd c(2);
  c << -1.0, -1.0;

  prog.AddQuadraticCost(Q, c);

  prog.SetInitialGuessForAllVariables(Eigen::Vector2d::Zero());
  prog.Solve();

  VectorXd expected_answer(2);
  expected_answer << 1.0, 1.0;
  auto x_value = prog.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(expected_answer, x_value, 1e-10,
                              MatrixCompareType::absolute));
  // There are no inequality constraints, and only quadratic costs,
  // so this should hold:
  CheckSolverType(prog, "Equality Constrained QP Solver");

  // Add one more variable and constrain a view into them.
  auto y = prog.NewContinuousVariables<1>("y");
  Q << 2.0, 0.0, 0.0, 2.0;
  c << -5.0, -2.0;
  VariableRefList vars;
  vars.push_back(x.segment<1>(1));
  vars.push_back(y);

  prog.AddQuadraticCost(Q, c, vars);
  prog.SetInitialGuessForAllVariables(Eigen::Vector3d::Zero());
  prog.Solve();
  expected_answer.resize(3);
  expected_answer << 1.0, 2.0, 1.0;
  VectorXd actual_answer(3);
  x_value = prog.GetSolution(x);
  const auto& y_value = prog.GetSolution(y);
  actual_answer << x_value, y_value;
  EXPECT_TRUE(CompareMatrices(expected_answer, actual_answer, 1e-10,
                              MatrixCompareType::absolute))
      << "\tExpected: " << expected_answer.transpose()
      << "\tActual: " << actual_answer.transpose();

  // Problem still has only quadratic costs, so solver should be the same.
  CheckSolverType(prog, "Equality Constrained QP Solver");
}

// Test how an equality-constrained QP is dispatched
//   - on the problem (x1 - 1)^2 + (x2 - 1)^2, with a min at
//     at (x1=1, x2=1), constrained with (x1 + x2 = 1).
//     The resulting constrained min is at (x1=0.5, x2=0.5).
//   - on the same problem with an additional variable x3,
//     with (2*x1 - x3 = 0). Resulting solution should be
//     (x1=0.5, x2=0.5, x3=1.0)
GTEST_TEST(testMathematicalProgram, testLinearlyConstrainedQPDispatch) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables(2);
  MatrixXd Q(2, 2);
  Q << 1, 0.0, 0.0, 1.0;
  VectorXd c(2);
  c << -1.0, -1.0;

  prog.AddQuadraticCost(Q, c);

  VectorXd constraint1(2);
  // x1 + x2 = 1
  constraint1 << 1, 1;
  prog.AddLinearEqualityConstraint(constraint1.transpose(), 1.0);

  prog.SetInitialGuessForAllVariables(Eigen::Vector2d::Zero());
  prog.Solve();

  VectorXd expected_answer(2);
  expected_answer << 0.5, 0.5;
  auto x_value = prog.GetSolution(x);
  EXPECT_TRUE(CompareMatrices(expected_answer, x_value, 1e-10,
                              MatrixCompareType::absolute));

  // This problem is now an Equality Constrained QP and should
  // use this solver:
  CheckSolverType(prog, "Equality Constrained QP Solver");

  // Add one more variable and constrain it in a different way
  auto y = prog.NewContinuousVariables(1);
  Vector2d constraint2(2);
  constraint2 << 2., -1.;
  // 2*x1 - x3 = 0, so x3 should wind up as 1.0
  VariableRefList vars;
  vars.push_back(x.segment(0, 1));
  vars.push_back(y);

  prog.AddLinearEqualityConstraint(constraint2.transpose(), 0.0, vars);
  prog.SetInitialGuessForAllVariables(Eigen::Vector3d::Zero());
  prog.Solve();
  expected_answer.resize(3);
  expected_answer << 0.5, 0.5, 1.0;
  VectorXd actual_answer(3);
  x_value = prog.GetSolution(x);
  auto y_value = prog.GetSolution(y);
  actual_answer << x_value, y_value;
  EXPECT_TRUE(CompareMatrices(expected_answer, actual_answer, 1e-10,
                              MatrixCompareType::absolute))
      << "\tExpected: " << expected_answer.transpose()
      << "\tActual: " << actual_answer.transpose();
}

// Solve an SOCP with Lorentz cone and rotated Lorentz cone constraint as a
// nonlinear optimization problem.
// The objective is to find the smallest distance from a hyperplane
// A * x = b to the origin.
// We can solve the following SOCP with Lorentz cone constraint
// min  t
//  s.t t >= sqrt(x'*x)
//      A * x = b.
// Alternatively, we can solve the following SOCP with rotated Lorentz cone
// constraint
// min t
// s.t t >= x'*x
//     A * x = b.
//
// The optimal solution of this equality constrained QP can be found using
// Lagrangian method. The optimal solution x* and Lagrangiam multiplier z*
// satisfy
// A_hat * [x*; z*] = [b; 0]
// where A_hat = [A 0; 2*I A'].
void MinDistanceFromPlaneToOrigin(const MatrixXd& A, const VectorXd b) {
  DRAKE_ASSERT(A.rows() == b.rows());
  const int xDim = A.cols();
  MathematicalProgram prog_lorentz;
  auto t_lorentz = prog_lorentz.NewContinuousVariables(1, "t");
  auto x_lorentz = prog_lorentz.NewContinuousVariables(xDim, "x");
  prog_lorentz.AddLorentzConeConstraint({t_lorentz, x_lorentz});
  prog_lorentz.AddLinearEqualityConstraint(A, b, x_lorentz);
  prog_lorentz.AddLinearCost(drake::Vector1d(1.0), t_lorentz);

  // A_hat = [A 0; 2*I A']
  MatrixXd A_hat(A.rows() + A.cols(), A.rows() + A.cols());
  A_hat.topLeftCorner(A.rows(), A.cols()) = A;
  A_hat.topRightCorner(A.rows(), A.rows()) = MatrixXd::Zero(A.rows(), A.rows());
  A_hat.bottomLeftCorner(A.cols(), A.cols()) =
      2 * MatrixXd::Identity(A.cols(), A.cols());
  A_hat.bottomRightCorner(A.cols(), A.rows()) = A.transpose();
  VectorXd b_hat(A.rows() + A.cols());
  b_hat << b, VectorXd::Zero(A.cols());
  VectorXd xz_expected = A_hat.colPivHouseholderQr().solve(b_hat);
  VectorXd x_expected = xz_expected.head(xDim);

  double cost_expected_lorentz = x_expected.norm();
  // NLopt needs a really good starting point to solve SOCP, while SNOPT and
  // IPOPT seems OK with starting point not so close to optimal solution.
  prog_lorentz.SetInitialGuess(t_lorentz,
                               drake::Vector1d(cost_expected_lorentz + 0.1));
  VectorXd x_lorentz_guess = x_expected + 0.1 * VectorXd::Ones(xDim);
  prog_lorentz.SetInitialGuess(x_lorentz, x_lorentz_guess);
  RunNonlinearProgram(prog_lorentz, [&]() {
    const auto& x_lorentz_value = prog_lorentz.GetSolution(x_lorentz);
    EXPECT_TRUE(CompareMatrices(x_lorentz_value, x_expected, 1E-5,
                                MatrixCompareType::absolute));
    const auto& t_lorentz_value = prog_lorentz.GetSolution(t_lorentz);
    EXPECT_NEAR(cost_expected_lorentz, t_lorentz_value(0), 1E-3);
  });

  MathematicalProgram prog_rotated_lorentz;
  auto t_rotated_lorentz = prog_rotated_lorentz.NewContinuousVariables(1, "t");
  auto x_rotated_lorentz =
      prog_rotated_lorentz.NewContinuousVariables(xDim, "x");
  auto slack_rotated_lorentz =
      prog_rotated_lorentz.NewContinuousVariables<1>("slack");
  prog_rotated_lorentz.AddRotatedLorentzConeConstraint(
      {t_rotated_lorentz, slack_rotated_lorentz, x_rotated_lorentz});
  prog_rotated_lorentz.AddLinearEqualityConstraint(A, b, x_rotated_lorentz);
  prog_rotated_lorentz.AddBoundingBoxConstraint(1.0, 1.0,
                                                slack_rotated_lorentz(0));
  prog_rotated_lorentz.AddLinearCost(drake::Vector1d(1.0), t_rotated_lorentz);

  double cost_expected_rotated_lorentz = x_expected.squaredNorm();
  // NLopt needs a really good starting point to solve SOCP, while SNOPT and
  // IPOPT seems OK with starting point not so close to optimal solution.
  prog_rotated_lorentz.SetInitialGuess(
      t_rotated_lorentz, drake::Vector1d(cost_expected_rotated_lorentz + 0.1));
  prog_rotated_lorentz.SetInitialGuess(slack_rotated_lorentz,
                                       drake::Vector1d(1.0));
  VectorXd x_rotated_lorentz_guess = x_expected + 0.1 * VectorXd::Ones(xDim);
  prog_rotated_lorentz.SetInitialGuess(x_rotated_lorentz,
                                       x_rotated_lorentz_guess);
  RunNonlinearProgram(prog_rotated_lorentz, [&]() {
    const auto& x_rotated_lorentz_value =
        prog_rotated_lorentz.GetSolution(x_rotated_lorentz);
    EXPECT_TRUE(CompareMatrices(x_rotated_lorentz_value, x_expected, 1E-5,
                                MatrixCompareType::absolute));
    const auto& t_rotated_lorentz_value =
        prog_rotated_lorentz.GetSolution(t_rotated_lorentz);
    EXPECT_NEAR(cost_expected_rotated_lorentz, t_rotated_lorentz_value(0),
                1E-3);
  });

  // Now add a constraint x'*x <= 2*x_expected'*x_expected to the problem.
  // The optimal solution and the costs are still the same, but now we test
  // Lorentz cone (rotated Lorentz cone) constraints with generic nonlinear
  // constraints.
  std::shared_ptr<QuadraticConstraint> quadratic_constraint(
      new QuadraticConstraint(MatrixXd::Identity(xDim, xDim),
                              VectorXd::Zero(xDim), 0,
                              x_expected.squaredNorm()));

  prog_lorentz.AddConstraint(quadratic_constraint, x_lorentz);
  RunNonlinearProgram(prog_lorentz, [&]() {
    const auto& x_lorentz_value = prog_lorentz.GetSolution(x_lorentz);
    EXPECT_TRUE(CompareMatrices(x_lorentz_value, x_expected, 1E-5,
                                MatrixCompareType::absolute));
    const auto& t_lorentz_value = prog_lorentz.GetSolution(t_lorentz);
    EXPECT_NEAR(cost_expected_lorentz, t_lorentz_value(0), 1E-3);
  });

  prog_rotated_lorentz.AddConstraint(quadratic_constraint, x_rotated_lorentz);
  RunNonlinearProgram(prog_rotated_lorentz, [&]() {
    const auto& x_rotated_lorentz_value =
        prog_rotated_lorentz.GetSolution(x_rotated_lorentz);
    EXPECT_TRUE(CompareMatrices(x_rotated_lorentz_value, x_expected, 1E-5,
                                MatrixCompareType::absolute));
    const auto& t_rotated_lorentz_value =
        prog_rotated_lorentz.GetSolution(t_rotated_lorentz);
    EXPECT_NEAR(cost_expected_rotated_lorentz, t_rotated_lorentz_value(0),
                1E-3);
  });
}

GTEST_TEST(testMathematicalProgram, testSolveSOCPasNLP) {
  MatrixXd A = Matrix<double, 1, 2>::Ones();
  VectorXd b = drake::Vector1d(2);
  MinDistanceFromPlaneToOrigin(A, b);

  A = Matrix<double, 2, 3>::Zero();
  A << 0, 1, 2, -1, 2, 3;
  b = Vector2d(1.0, 3.0);
  MinDistanceFromPlaneToOrigin(A, b);
}

}  // namespace test
}  // namespace solvers
}  // namespace drake
