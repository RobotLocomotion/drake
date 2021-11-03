#include "drake/systems/framework/system_constraint.h"

#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/systems/framework/system_symbolic_inspector.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace {

using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorXd;

// A hollow shell of a System.
class DummySystem final : public LeafSystem<double> {
 public:
  DummySystem() {}
};

GTEST_TEST(SystemConstraintBoundsTest, DefaultCtor) {
  const SystemConstraintBounds dut;
  EXPECT_EQ(dut.size(), 0);
  EXPECT_TRUE(CompareMatrices(dut.lower(), VectorXd::Zero(0)));
  EXPECT_TRUE(CompareMatrices(dut.upper(), VectorXd::Zero(0)));
}

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
  const SystemConstraintBounds dut(Vector2d::Zero(), std::nullopt);
  EXPECT_EQ(dut.size(), 2);
  EXPECT_EQ(dut.type(), SystemConstraintType::kInequality);
  EXPECT_TRUE(CompareMatrices(dut.lower(), Vector2d::Zero()));
  EXPECT_TRUE(CompareMatrices(dut.upper(), Vector2d::Constant(kInf)));
}

GTEST_TEST(SystemConstraintBoundsTest, Upper) {
  const double kInf = std::numeric_limits<double>::infinity();
  const SystemConstraintBounds dut(std::nullopt, Vector2d::Constant(1.0));
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
  ContextConstraintCalc<double> calc = [](
      const Context<double>& context, Eigen::VectorXd* value) {
    *value = Vector1d(context.get_continuous_state_vector()[1]);
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
      &system, calc, SystemConstraintBounds::Equality(1),
      "equality constraint");
  context->get_mutable_continuous_state_vector()[1] = 5.0;
  equality_constraint.Calc(*context, &value);
  EXPECT_EQ(value[0], 5.0);
  EXPECT_TRUE(CompareMatrices(equality_constraint.lower_bound(), Vector1d(0)));
  EXPECT_TRUE(CompareMatrices(equality_constraint.upper_bound(), Vector1d(0)));
  EXPECT_FALSE(equality_constraint.CheckSatisfied(*context, tol));

  context->get_mutable_continuous_state_vector()[1] = 0.0;
  equality_constraint.Calc(*context, &value);
  EXPECT_EQ(value[0], 0.0);
  EXPECT_TRUE(equality_constraint.CheckSatisfied(*context, tol));

  EXPECT_EQ(equality_constraint.size(), 1);
  EXPECT_EQ(equality_constraint.description(), "equality constraint");

  // Test inequality constraint.
  SystemConstraint<double> inequality_constraint(
      &system, calc2, { Eigen::Vector2d::Ones(), Eigen::Vector2d(4, 6) },
      "inequality constraint");
  EXPECT_TRUE(CompareMatrices(inequality_constraint.lower_bound(),
                              Eigen::Vector2d::Ones()));
  EXPECT_TRUE(CompareMatrices(inequality_constraint.upper_bound(),
                              Eigen::Vector2d(4, 6)));
  context->get_mutable_continuous_state_vector()[0] = 3.0;
  context->get_mutable_continuous_state_vector()[1] = 5.0;
  inequality_constraint.Calc(*context, &value);
  EXPECT_EQ(value[0], 3.0);
  EXPECT_EQ(value[1], 5.0);
  EXPECT_TRUE(inequality_constraint.CheckSatisfied(*context, tol));

  context->get_mutable_continuous_state_vector()[1] = -0.5;
  inequality_constraint.Calc(*context, &value);
  EXPECT_EQ(value[1], -0.5);
  EXPECT_FALSE(inequality_constraint.CheckSatisfied(*context, tol));

  EXPECT_EQ(inequality_constraint.size(), 2);
  EXPECT_EQ(inequality_constraint.description(), "inequality constraint");
  EXPECT_EQ(&inequality_constraint.get_system(), &system);
}

class WrongOrderSystem final : public LeafSystem<double> {
 public:
  WrongOrderSystem() {
    this->DeclareEqualityConstraint([](const auto&, auto*) {}, 0, "dummy1");
    this->AddExternalConstraint(ExternalSystemConstraint{});
    this->DeclareEqualityConstraint([](const auto&, auto*) {}, 0, "dummy2");
  }
};

// We should fail-fast if constraints are added in the wrong order.
GTEST_TEST(SystemConstraintTest, WrongOrder) {
  DRAKE_EXPECT_THROWS_MESSAGE(
      WrongOrderSystem{},
      "System _ cannot add an internal constraint \\(named dummy2\\) after "
      "an external constraint \\(named empty\\) has already been added");
}

// For a state x, constrains xc_dot = 0 and xd_{n+1} = 0.
// TODO(jwnimmer-tri) This cannot handle discrete state yet.
// TODO(jwnimmer-tri) Make this constraint public (and not testonly) -- it is
// quite commonly used.
ExternalSystemConstraint ZeroStateDerivativeConstraint(
    const System<double>& shape) {
  auto dummy_context = shape.AllocateContext();

  // We don't handle discrete state yet, because to do so efficiently we'd
  // need a method like System::EvalDiscreteVariableUpdates, but only the Calc
  // flavor of that method currently exists.
  DRAKE_DEMAND(dummy_context->num_discrete_state_groups() == 0);
  const int n_xc = dummy_context->get_continuous_state_vector().size();
  return ExternalSystemConstraint::MakeForAllScalars(
      "xc_dot = 0 and xd_{n+1} = 0",
      SystemConstraintBounds::Equality(n_xc),
      [](const auto& system, const auto& context, auto* value) {
        const auto& xc = system.EvalTimeDerivatives(context).get_vector();
        *value = xc.CopyToVector();
      });
}

// For a state x, constrains velocity(x) = 0.  This can be useful to directly
// express decision variable bounds to a solver, even if an constraint on xcdot
// is already in effect.
// TODO(jwnimmer-tri) This cannot handle discrete state yet.
// TODO(jwnimmer-tri) Make this constraint public (and not testonly) -- it is
// quite commonly used.
ExternalSystemConstraint ZeroVelocityConstraint(
    const System<double>& shape) {
  auto dummy_context = shape.AllocateContext();

  // We can't handle discrete state yet, because of #9171.
  DRAKE_DEMAND(dummy_context->num_discrete_state_groups() == 0);
  const int num_v = dummy_context->get_continuous_state().num_v();
  return ExternalSystemConstraint::MakeForAllScalars(
      "zero velocity", SystemConstraintBounds::Equality(num_v),
      [](const auto& system, const auto& context, auto* value) {
        *value = context.get_continuous_state().get_generalized_velocity()
          .CopyToVector();
      });
}

class ExternalSystemConstraintTest : public ::testing::Test {
 protected:
  // This system_ is a double integrator.
  LinearSystem<double> system_{
    (Matrix2d{} << 0.0, 1.0, 0.0, 0.0).finished(),
    Vector2d(0.0, 1.0),
    MatrixXd(0, 2),
    MatrixXd(0, 1)};
};

// Acceptance test the symbolic form of a fixed-point constraint.
TEST_F(ExternalSystemConstraintTest, FixedPoint) {
  // Add fixed-point constraints.
  system_.AddExternalConstraint(ZeroStateDerivativeConstraint(system_));
  system_.AddExternalConstraint(ZeroVelocityConstraint(system_));

  // Check their symbolic form, thus proving that the constraints have
  // been scalar-converted correctly.
  auto symbolic = system_.ToSymbolic();
  const SystemSymbolicInspector inspector(*symbolic);
  ASSERT_EQ(inspector.constraints().size(), 2);
  EXPECT_EQ(make_conjunction(inspector.constraints()).to_string(),
            "((u0_0 == 0) and (xc1 == 0))");

  // Confirm that the description was preserved through scalar conversion.
  const auto& zero_vel = system_.get_constraint(SystemConstraintIndex{1});
  EXPECT_EQ(zero_vel.description(), "zero velocity");
}

// Sanity check the public accessors.
TEST_F(ExternalSystemConstraintTest, Accessors) {
  auto dut = ZeroVelocityConstraint(system_);
  EXPECT_EQ(dut.description(), "zero velocity");
  EXPECT_EQ(dut.bounds().type(), SystemConstraintType::kEquality);
  EXPECT_TRUE(dut.get_calc<double>());
  EXPECT_FALSE(dut.get_calc<float>());
}

// Check the double-only public constructor.
TEST_F(ExternalSystemConstraintTest, DoubleOnly) {
  const DummySystem dummy_system;
  auto dummy_context = dummy_system.CreateDefaultContext();

  const ExternalSystemConstraint dut(
      "desc", {std::nullopt, Vector2d::Constant(100.0)},
      [&](const System<double>& system, const Context<double>& context,
         VectorXd* value) {
        EXPECT_EQ(&system, &dummy_system);
        EXPECT_EQ(&context, dummy_context.get());
        *value = Vector2d::Constant(22.0);
      });
  EXPECT_EQ(dut.description(), "desc");
  EXPECT_EQ(dut.bounds().type(), SystemConstraintType::kInequality);
  EXPECT_TRUE(dut.get_calc<double>());
  EXPECT_FALSE(dut.get_calc<AutoDiffXd>());
  EXPECT_FALSE(dut.get_calc<symbolic::Expression>());

  VectorXd value;
  dut.get_calc<double>()(dummy_system, *dummy_context, &value);
  EXPECT_TRUE(CompareMatrices(value, Vector2d::Constant(22.0)));
}

// Check the non-symbolic factory function.
TEST_F(ExternalSystemConstraintTest, NonSymbolic) {
  const auto& dut = ExternalSystemConstraint::MakeForNonsymbolicScalars(
      "xcdot = 0", SystemConstraintBounds::Equality(2),
      [](const auto& system, const auto& context, auto* value) {
        const auto& xc = system.EvalTimeDerivatives(context).get_vector();
        *value = xc.CopyToVector();
      });
  EXPECT_EQ(dut.description(), "xcdot = 0");
  EXPECT_EQ(dut.bounds().type(), SystemConstraintType::kEquality);
  EXPECT_TRUE(dut.get_calc<double>());
  EXPECT_TRUE(dut.get_calc<AutoDiffXd>());
  EXPECT_FALSE(dut.get_calc<symbolic::Expression>());

  // Check that the generic lambda turned into a correct functor by calling it
  // and checking the result, for double.
  auto context_double = system_.CreateDefaultContext();
  system_.get_input_port().FixValue(context_double.get(), VectorXd::Zero(1));
  context_double->SetContinuousState((VectorXd(2) << 0.0, 22.0).finished());
  VectorXd value_double;
  dut.get_calc<double>()(system_, *context_double, &value_double);
  EXPECT_TRUE(CompareMatrices(value_double, Vector2d(22.0, 0.0)));

  // Check that the generic lambda turned into a correct functor by calling it
  // and checking the result, for AutoDiffXd.
  using T = AutoDiffXd;
  auto system_autodiff = system_.ToAutoDiffXd();
  auto context_autodiff = system_autodiff->CreateDefaultContext();
  system_autodiff->get_input_port().FixValue(context_autodiff.get(),
                                             VectorX<T>::Zero(1));
  context_autodiff->SetContinuousState(
      (VectorX<T>(2) << T{0.0}, T{22.0}).finished());
  VectorX<T> value_autodiff;
  dut.get_calc<T>()(
      *system_autodiff, *context_autodiff, &value_autodiff);
  EXPECT_EQ(value_autodiff[0].value(), 22.0);
  EXPECT_EQ(value_autodiff[1].value(), 0.0);
}

}  // namespace
}  // namespace systems
}  // namespace drake
