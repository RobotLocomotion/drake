#include "drake/systems/framework/system_constraint.h"

#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace {

using Eigen::Vector2d;

// A hollow shell of a System.
class DummySystem final : public LeafSystem<double> {
 public:
  DummySystem() {}
};

GTEST_TEST(SystemConstraintBoundsTest, EqFactory) {
  auto dut = SystemConstraintBounds::Equality(2);
  EXPECT_EQ(dut.size(), 2);
  EXPECT_EQ(dut.type(), SystemConstraintType::kEquality);
  EXPECT_TRUE(CompareMatrices(dut.lower(), Vector2d::Zero()));
  EXPECT_TRUE(CompareMatrices(dut.upper(), Vector2d::Zero()));
}

GTEST_TEST(SystemConstraintBoundsTest, EqCtor) {
  const SystemConstraintBounds dut(Vector2d::Zero(), Vector2d::Zero());
  EXPECT_EQ(dut.size(), 2);
  EXPECT_EQ(dut.type(), SystemConstraintType::kEquality);
  EXPECT_TRUE(CompareMatrices(dut.lower(), Vector2d::Zero()));
  EXPECT_TRUE(CompareMatrices(dut.upper(), Vector2d::Zero()));
}

GTEST_TEST(SystemConstraintBoundsTest, EqNonzero) {
  const Vector2d b = Vector2d::Constant(2.0);
  // For now, we only allow b == 0; in the future, we might allow b != 0.
  EXPECT_THROW(SystemConstraintBounds(b, b), std::exception);
}

GTEST_TEST(SystemConstraintBoundsTest, Lower) {
  const double kInf = std::numeric_limits<double>::infinity();
  const SystemConstraintBounds dut(Vector2d::Zero(), nullopt);
  EXPECT_EQ(dut.size(), 2);
  EXPECT_EQ(dut.type(), SystemConstraintType::kInequality);
  EXPECT_TRUE(CompareMatrices(dut.lower(), Vector2d::Zero()));
  EXPECT_TRUE(CompareMatrices(dut.upper(), Vector2d::Constant(kInf)));
}

GTEST_TEST(SystemConstraintBoundsTest, Upper) {
  const double kInf = std::numeric_limits<double>::infinity();
  const SystemConstraintBounds dut(nullopt, Vector2d::Constant(1.0));
  EXPECT_EQ(dut.size(), 2);
  EXPECT_EQ(dut.type(), SystemConstraintType::kInequality);
  EXPECT_TRUE(CompareMatrices(dut.lower(), Vector2d::Constant(-kInf)));
  EXPECT_TRUE(CompareMatrices(dut.upper(), Vector2d::Constant(1.0)));
}

GTEST_TEST(SystemConstraintBoundsTest, BadSizes) {
  EXPECT_THROW(SystemConstraintBounds::Equality(-1), std::exception);
  EXPECT_THROW(
      SystemConstraintBounds(
          Vector1d::Constant(1.0),
          Vector2d::Constant(2.0)),
      std::exception);
}

// Just a simple test to call each of the public methods.
GTEST_TEST(SystemConstraintTest, Basic) {
  const DummySystem dummy_system;

  ContextConstraintCalc<double> calc = [](
      const Context<double>& context, Eigen::VectorXd* value) {
    *value = Vector1d(context.get_continuous_state_vector().GetAtIndex(1));
  };
  ContextConstraintCalc<double> calc2 = [](
      const Context<double>& context, Eigen::VectorXd* value) {
    *value =
        Eigen::Vector2d(context.get_continuous_state_vector().CopyToVector());
  };

  Eigen::VectorXd value;

  // Make a (linear) system just to make a valid context.
  LinearSystem<double> system(Eigen::Matrix2d::Identity(),
                              Eigen::Vector2d::Zero(), Eigen::MatrixXd(0, 2),
                              Eigen::MatrixXd(0, 1));
  auto context = system.CreateDefaultContext();

  const double tol = 1e-6;

  // Test equality constraint.
  SystemConstraint<double> equality_constraint(
      &dummy_system, calc, SystemConstraintBounds::Equality(1),
      "equality constraint");
  context->get_mutable_continuous_state_vector().SetAtIndex(1, 5.0);
  equality_constraint.Calc(*context, &value);
  EXPECT_EQ(value[0], 5.0);
  EXPECT_TRUE(CompareMatrices(equality_constraint.lower_bound(), Vector1d(0)));
  EXPECT_TRUE(CompareMatrices(equality_constraint.upper_bound(), Vector1d(0)));
  EXPECT_FALSE(equality_constraint.CheckSatisfied(*context, tol));

  context->get_mutable_continuous_state_vector().SetAtIndex(1, 0.0);
  equality_constraint.Calc(*context, &value);
  EXPECT_EQ(value[0], 0.0);
  EXPECT_TRUE(equality_constraint.CheckSatisfied(*context, tol));

  EXPECT_EQ(equality_constraint.size(), 1);
  EXPECT_EQ(equality_constraint.description(), "equality constraint");

  // Test inequality constraint.
  SystemConstraint<double> inequality_constraint(
      &dummy_system, calc2, { Eigen::Vector2d::Ones(), Eigen::Vector2d(4, 6) },
      "inequality constraint");
  EXPECT_TRUE(CompareMatrices(inequality_constraint.lower_bound(),
                              Eigen::Vector2d::Ones()));
  EXPECT_TRUE(CompareMatrices(inequality_constraint.upper_bound(),
                              Eigen::Vector2d(4, 6)));
  context->get_mutable_continuous_state_vector().SetAtIndex(0, 3.0);
  context->get_mutable_continuous_state_vector().SetAtIndex(1, 5.0);
  inequality_constraint.Calc(*context, &value);
  EXPECT_EQ(value[0], 3.0);
  EXPECT_EQ(value[1], 5.0);
  EXPECT_TRUE(inequality_constraint.CheckSatisfied(*context, tol));

  context->get_mutable_continuous_state_vector().SetAtIndex(1, -0.5);
  inequality_constraint.Calc(*context, &value);
  EXPECT_EQ(value[1], -0.5);
  EXPECT_FALSE(inequality_constraint.CheckSatisfied(*context, tol));

  EXPECT_EQ(inequality_constraint.size(), 2);
  EXPECT_EQ(inequality_constraint.description(), "inequality constraint");
}

ExternalSystemConstraint MakeFixedPointConstraint(
    const System<double>& shape) {
  auto dummy_context = shape.AllocateContext();

  // We can't handle discrete state yet.
  DRAKE_DEMAND(dummy_context->get_num_discrete_state_groups() == 0);

  // For fixed point, both xc_dot and v must be zero.
  // TODO(jwnimmer-tri) Do we *really* need to check v also?  Is there any case
  // where v is non-zero but xcdot is all zero?
  const int n_xc = dummy_context->get_continuous_state_vector().size();
  const int n_v = dummy_context->get_continuous_state().num_v();
  const int constraint_size = n_xc + n_v;
  return ExternalSystemConstraint::MakeForAllScalars(
      "fixed point", SystemConstraintBounds::Equality(constraint_size),
      [n_xc, n_v](const auto& system, const auto& context, auto* value) {
        const auto& xc = system.EvalTimeDerivatives(context).get_vector();
        const auto& v =
            context.get_continuous_state().get_generalized_velocity();
        value->segment(0, n_xc) = xc.CopyToVector();
        value->segment(n_xc, n_v) = v.CopyToVector();
      });
}

// Acceptance test the symbolic form a fixed-point constraint.
GTEST_TEST(ExternalSystemConstraintTest, FixedPoint) {
  // A double integrator.
  Eigen::Matrix2d A;
  A << 0.0, 1.0,
       0.0, 0.0;
  Eigen::Vector2d B;
  B << 0.0, 1.0;
  Eigen::MatrixXd C(0, 2);
  Eigen::MatrixXd D(0, 1);
  LinearSystem<double> system(A, B, C, D);

  system.AddConstraint(MakeFixedPointConstraint(system));
  auto symbolic = system.ToSymbolic();
  SystemSymbolicInspector inspector(*symbolic);
  const auto& symbolic_constraints = inspector.constraints();
  ASSERT_EQ(symbolic_constraints.size(), 1);
  EXPECT_EQ(symbolic_constraints.begin()->to_string(),
            "((u0_0 == 0) and (xc1 == 0))");
}

}  // namespace
}  // namespace systems
}  // namespace drake
