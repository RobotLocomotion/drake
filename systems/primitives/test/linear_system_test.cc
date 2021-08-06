#include "drake/systems/primitives/linear_system.h"

#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/examples/pendulum/pendulum_plant.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"
#include "drake/systems/primitives/test/affine_linear_test.h"

using std::make_unique;
using std::unique_ptr;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace systems {
namespace {

class LinearSystemPlusEmptyVectorPort final : public LinearSystem<double> {
 public:
  LinearSystemPlusEmptyVectorPort(
        const Eigen::Ref<const Eigen::MatrixXd>& A,
        const Eigen::Ref<const Eigen::MatrixXd>& B,
        const Eigen::Ref<const Eigen::MatrixXd>& C,
        const Eigen::Ref<const Eigen::MatrixXd>& D)
      : LinearSystem(SystemScalarConverter{}, A, B, C, D, 0.0) {
    this->DeclareInputPort(kUseDefaultName, kVectorValued, 0);
  }
};

GTEST_TEST(LinearSystemTestWithEmptyPort, EmptyPort) {
  // Set linear system matrices as simply as possible while using a non-empty
  // vector input.
  const int num_states = 1;
  const int num_inputs = 1;
  MatrixXd B(num_states, num_inputs);
  B << 0;
  const MatrixXd A = B;
  const MatrixXd C = B;
  const MatrixXd D = B;
  LinearSystemPlusEmptyVectorPort dut(A, B, C, D);

  // Get the system as a System<double>. This is necessary because the
  // grandparent (TimeVaryingAffineSystem) shadows System::get_input_port(int)
  // with TimeVaryingAffineSystem::get_input_port(), which makes the former
  // inaccessible.
  System<double>& system = dut;

  // Verify that the first two vector input ports have expected sizes.
  ASSERT_EQ(system.get_input_port(0).size(), num_inputs);
  ASSERT_EQ(system.get_input_port(1).size(), 0);

  // Verify that computing derivatives does not cause an exception to be thrown
  // when the empty vector port is unconnected.
  auto context = dut.CreateDefaultContext();
  auto derivatives = dut.AllocateTimeDerivatives();
  dut.get_input_port().FixValue(context.get(), 0.);
  DRAKE_EXPECT_NO_THROW(dut.CalcTimeDerivatives(*context, derivatives.get()));
}

class LinearSystemTest : public AffineLinearSystemTest {
 public:
  // Setup an arbitrary LinearSystem.
  LinearSystemTest() : AffineLinearSystemTest(0.0, 0.0, 0.0, 0.0) {}

  void Initialize() override {
    // Construct the system I/O objects.
    dut_ = make_unique<LinearSystem<double>>(A_, B_, C_, D_);
    dut_->set_name("test_linear_system");
    context_ = dut_->CreateDefaultContext();
    input_port_ = &dut_->get_input_port();
    state_ = &context_->get_mutable_continuous_state();
    derivatives_ = dut_->AllocateTimeDerivatives();
  }

 protected:
  // The Device Under Test (DUT) is a LinearSystem<double>.
  unique_ptr<LinearSystem<double>> dut_;
};

// Tests that the linear system is correctly setup.
TEST_F(LinearSystemTest, Construction) {
  EXPECT_EQ(1, context_->num_input_ports());
  EXPECT_EQ("test_linear_system", dut_->get_name());
  EXPECT_EQ(dut_->A(), A_);
  EXPECT_EQ(dut_->B(), B_);
  EXPECT_EQ(dut_->f0(), f0_);
  EXPECT_EQ(dut_->C(), C_);
  EXPECT_EQ(dut_->D(), D_);
  EXPECT_EQ(dut_->y0(), y0_);
  EXPECT_EQ(1, dut_->num_output_ports());
  EXPECT_EQ(1, dut_->num_input_ports());
}

// Tests that the derivatives are correctly computed.
TEST_F(LinearSystemTest, Derivatives) {
  Eigen::Vector2d u(1, 4);
  SetInput(u);

  Eigen::Vector2d x(0.1, 0.25);
  state_->SetFromVector(x);

  EXPECT_NE(derivatives_, nullptr);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  Eigen::VectorXd expected_derivatives = A_ * x + B_ * u;

  EXPECT_EQ(expected_derivatives, derivatives_->get_vector().CopyToVector());
}

// Tests that the outputs are correctly computed.
TEST_F(LinearSystemTest, Output) {
  // Sets the context's input port.
  Eigen::Vector2d u(5.6, -10.1);
  SetInput(u);

  // Sets the state.
  Eigen::Vector2d x(0.8, -22.1);
  state_->SetFromVector(x);

  Eigen::VectorXd expected_output = C_ * x + D_ * u;
  EXPECT_EQ(expected_output, dut_->get_output_port().Eval(*context_));
}

// Tests converting to different scalar types.
TEST_F(LinearSystemTest, ConvertScalarType) {
  EXPECT_TRUE(is_autodiffxd_convertible(*dut_, [&](const auto& converted) {
    EXPECT_EQ(converted.A(), A_);
    EXPECT_EQ(converted.B(), B_);
    EXPECT_EQ(converted.C(), C_);
    EXPECT_EQ(converted.D(), D_);
  }));
  EXPECT_TRUE(is_symbolic_convertible(*dut_, [&](const auto& converted) {
    EXPECT_EQ(converted.A(), A_);
    EXPECT_EQ(converted.B(), B_);
    EXPECT_EQ(converted.C(), C_);
    EXPECT_EQ(converted.D(), D_);
  }));
}

// [ẋ₁, ẋ₂]ᵀ = rotmat(t)*[x₁, x₂]ᵀ + u*[1, 1]ᵀ,
// [y₁, y₂] = [x₁, x₂] + u*[1, 1].
class SimpleTimeVaryingLinearSystem final
    : public TimeVaryingLinearSystem<double> {
 public:
  static constexpr int kNumStates = 2;
  static constexpr int kNumInputs = 1;
  static constexpr int kNumOutputs = 2;

  SimpleTimeVaryingLinearSystem()
      : TimeVaryingLinearSystem<double>(SystemScalarConverter{},  // BR
                                        kNumStates, kNumInputs, kNumOutputs,
                                        0.0 /* continuous-time */) {}

  ~SimpleTimeVaryingLinearSystem() override {}

  Eigen::MatrixXd A(const double& t) const override {
    using std::cos;
    using std::sin;
    Eigen::Matrix<double, kNumOutputs, kNumStates> mat;
    mat << cos(t), -sin(t), sin(t), cos(t);
    return mat;
  }
  Eigen::MatrixXd B(const double& t) const override {
    return Eigen::Matrix<double, kNumOutputs, kNumInputs>::Ones();
  }
  Eigen::MatrixXd C(const double& t) const override {
    return Eigen::Matrix<double, kNumStates, kNumStates>::Identity();
  }
  Eigen::MatrixXd D(const double& t) const override {
    return Eigen::Matrix<double, kNumOutputs, kNumInputs>::Ones();
  }
};

GTEST_TEST(SimpleTimeVaryingLinearSystemTest, ConstructorTest) {
  SimpleTimeVaryingLinearSystem sys;

  EXPECT_EQ(sys.num_output_ports(), 1);
  EXPECT_EQ(sys.num_input_ports(), 1);
  EXPECT_TRUE(CompareMatrices(sys.A(0.), Eigen::Matrix2d::Identity()));
  EXPECT_TRUE(CompareMatrices(sys.B(0.), Eigen::Matrix<double, 2, 1>::Ones()));
  EXPECT_TRUE(CompareMatrices(sys.C(0.), Eigen::Matrix2d::Identity()));
  EXPECT_TRUE(CompareMatrices(sys.D(0.), Eigen::Matrix<double, 2, 1>::Ones()));
}

class TestLinearizeFromAffine : public ::testing::Test {
 protected:
  void SetUp() {
    A_ << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    B_ << 10, 11, 12;
    f0_ << 13, 14, 15;
    C_ << 16, 17, 18, 19, 20, 21;
    D_ << 22, 23;
    y0_ << 24, 25;

    xstar_continuous_ = A_.colPivHouseholderQr().solve(-B_ * u0_ - f0_);
    xstar_discrete_ = (Eigen::Matrix3d::Identity() - A_)
                          .colPivHouseholderQr()
                          .solve(B_ * u0_ + f0_);

    continuous_system_.reset(
        new AffineSystem<double>(A_, B_, f0_, C_, D_, y0_));
    discrete_system_.reset(
        new AffineSystem<double>(A_, B_, f0_, C_, D_, y0_, time_period_));
  }

  Eigen::Matrix3d A_;
  Eigen::Matrix<double, 3, 1> B_;
  Eigen::Vector3d f0_;
  Eigen::Matrix<double, 2, 3> C_;
  Eigen::Vector2d D_;
  Eigen::Vector2d y0_;

  Eigen::Vector3d x0_{26, 27, 28};
  double u0_{29};
  Eigen::Vector3d xstar_continuous_;
  Eigen::Vector3d xstar_discrete_;

  const double time_period_ = 0.1;

  std::unique_ptr<AffineSystem<double>> continuous_system_;
  std::unique_ptr<AffineSystem<double>> discrete_system_;
};

// Test that linearizing a continuous-time affine system returns the original
// A,B,C,D matrices.
TEST_F(TestLinearizeFromAffine, ContinuousAtEquilibrium) {
  auto context = continuous_system_->CreateDefaultContext();
  continuous_system_->get_input_port().FixValue(context.get(), u0_);

  // Set x0 to the actual equilibrium point.
  context->get_mutable_continuous_state_vector().SetFromVector(
      xstar_continuous_);

  auto linearized_system = Linearize(*continuous_system_, *context);

  double tol = 1e-10;
  EXPECT_TRUE(CompareMatrices(A_, linearized_system->A(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(B_, linearized_system->B(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(C_, linearized_system->C(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(D_, linearized_system->D(), tol,
                              MatrixCompareType::absolute));

  std::unique_ptr<AffineSystem<double>> affine_system =
      FirstOrderTaylorApproximation(*continuous_system_, *context);
  EXPECT_TRUE(CompareMatrices(f0_, affine_system->f0(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(y0_, affine_system->y0(), tol,
                              MatrixCompareType::absolute));
}

// Test that linearizing a continuous-time affine system about a point that is
// not at equilibrium returns the original A,B,C,D matrices and affine terms.
TEST_F(TestLinearizeFromAffine, ContinuousAtNonEquilibrium) {
  auto context = continuous_system_->CreateDefaultContext();
  continuous_system_->get_input_port().FixValue(context.get(), u0_);
  context->get_mutable_continuous_state_vector().SetFromVector(x0_);

  // This Context is not an equilibrium point.
  EXPECT_THROW(Linearize(*continuous_system_, *context), std::runtime_error);

  // Obtain a linearization at this nonequilibrium condition.
  std::unique_ptr<AffineSystem<double>> affine_system =
      FirstOrderTaylorApproximation(*continuous_system_, *context);

  double tol = 1e-10;
  // We recover the original affine system.
  EXPECT_TRUE(CompareMatrices(A_, affine_system->A(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(B_, affine_system->B(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(C_, affine_system->C(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(D_, affine_system->D(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(f0_, affine_system->f0(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(y0_, affine_system->y0(), tol,
                              MatrixCompareType::absolute));
}

TEST_F(TestLinearizeFromAffine, DiscreteAtEquilibrium) {
  auto context = discrete_system_->CreateDefaultContext();
  discrete_system_->get_input_port().FixValue(context.get(), u0_);

  // Set x0 to the actual equilibrium point.
  systems::BasicVector<double>& xd =
      context->get_mutable_discrete_state().get_mutable_vector();
  xd.SetFromVector(xstar_discrete_);

  auto linearized_system = Linearize(*discrete_system_, *context);

  double tol = 1e-10;
  EXPECT_TRUE(CompareMatrices(A_, linearized_system->A(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(B_, linearized_system->B(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(C_, linearized_system->C(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(D_, linearized_system->D(), tol,
                              MatrixCompareType::absolute));
  EXPECT_EQ(time_period_, linearized_system->time_period());

  std::unique_ptr<AffineSystem<double>> affine_system =
      FirstOrderTaylorApproximation(*discrete_system_, *context);
  EXPECT_TRUE(CompareMatrices(f0_, affine_system->f0(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(y0_, affine_system->y0(), tol,
                              MatrixCompareType::absolute));
}

// Test that linearizing a discrete-time affine system about a point that is not
// at equilibrium returns the original A,B,C,D matrices and affine terms.
TEST_F(TestLinearizeFromAffine, DiscreteAtNonEquilibrium) {
  auto context = discrete_system_->CreateDefaultContext();
  discrete_system_->get_input_port().FixValue(context.get(), u0_);
  systems::BasicVector<double>& xd =
      context->get_mutable_discrete_state().get_mutable_vector();
  xd.SetFromVector(x0_);

  // This Context is not an equilibrium point.
  EXPECT_THROW(Linearize(*discrete_system_, *context), std::runtime_error);

  // Obtain a linearization at this nonequilibrium condition.
  std::unique_ptr<AffineSystem<double>> affine_system =
      FirstOrderTaylorApproximation(*discrete_system_, *context);

  double tol = 1e-10;
  // We recover the original affine system.
  EXPECT_TRUE(CompareMatrices(A_, affine_system->A(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(B_, affine_system->B(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(C_, affine_system->C(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(D_, affine_system->D(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(f0_, affine_system->f0(), tol,
                              MatrixCompareType::absolute));
  EXPECT_TRUE(CompareMatrices(y0_, affine_system->y0(), tol,
                              MatrixCompareType::absolute));
  EXPECT_EQ(time_period_, affine_system->time_period());
}

// A trivial system with discrete state that is not bound to a periodic update
// cycle.
class TestNonPeriodicSystem : public LeafSystem<double> {
 public:
  TestNonPeriodicSystem() {
    this->DeclareDiscreteState(1);
    this->DeclarePerStepEvent(PublishEvent<double>());
  }

  void DoCalcDiscreteVariableUpdates(
      const Context<double>& context,
      const std::vector<const DiscreteUpdateEvent<double>*>&,
      DiscreteValues<double>* discrete_state) const override {
    (*discrete_state)[0] = context.get_discrete_state(0).GetAtIndex(0) + 1;
  }
};

// A system with no state and an abstract input port.
template <typename T>
class EmptyStateSystemWithAbstractInput final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EmptyStateSystemWithAbstractInput);
  EmptyStateSystemWithAbstractInput()
      : LeafSystem<T>(SystemTypeTag<EmptyStateSystemWithAbstractInput>{}) {
    this->DeclareAbstractInputPort(
        "dummy", Value<std::vector<double>>() /* Arbitrary data type */);
  }

  // Scalar-converting copy constructor. See @ref system_scalar_conversion.
  template <typename U>
  explicit EmptyStateSystemWithAbstractInput(
      const EmptyStateSystemWithAbstractInput<U>&)
      : EmptyStateSystemWithAbstractInput<T>() {}
};

// A system with no state, a vector input port, and an abstract input port.
template <typename T>
class EmptyStateSystemWithMixedInputs final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EmptyStateSystemWithMixedInputs);
  EmptyStateSystemWithMixedInputs()
      : LeafSystem<T>(SystemTypeTag<EmptyStateSystemWithMixedInputs>{}) {
    this->DeclareVectorInputPort(
        kUseDefaultName, 1 /* scalar input */);
    this->DeclareAbstractInputPort(
        "dummy", Value<std::vector<double>>() /* Arbitrary data type */);
  }

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit EmptyStateSystemWithMixedInputs(
      const EmptyStateSystemWithMixedInputs<U>&)
      : EmptyStateSystemWithMixedInputs<T>() {}
};

// Confirm that IsDifferenceEquationSystem works as expected for the family
// of test systems we have here.
GTEST_TEST(TestSystem, IsDifferentEquationSystem) {
  double returned_time_period = 0.42;

  const Vector1d a(1);
  const Vector1d b(1);
  const Vector1d c(1);
  const Vector1d d(1);
  const LinearSystem<double> continuous_time_system(a, b, c, d);
  EXPECT_FALSE(
      continuous_time_system.IsDifferenceEquationSystem(&returned_time_period));
  // The argument value should not have changed.
  EXPECT_EQ(returned_time_period, 0.42);

  const double time_period = 0.1;
  const LinearSystem<double> discrete_time_system(a, b, c, d, time_period);
  EXPECT_TRUE(discrete_time_system.IsDifferenceEquationSystem());
  EXPECT_TRUE(
      discrete_time_system.IsDifferenceEquationSystem(&returned_time_period));
  EXPECT_EQ(returned_time_period, time_period);

  returned_time_period = 3.71;
  const TestNonPeriodicSystem non_periodic_system;
  EXPECT_FALSE(
      non_periodic_system.IsDifferenceEquationSystem(&returned_time_period));
  EXPECT_EQ(returned_time_period, 3.71);

  const EmptyStateSystemWithAbstractInput<double> empty_system1;
  EXPECT_FALSE(empty_system1.IsDifferenceEquationSystem());

  const EmptyStateSystemWithMixedInputs<double> empty_system2;
  EXPECT_FALSE(empty_system2.IsDifferenceEquationSystem());
}

// Test that linearizing a system with abstract input port throws an
// exception when trying to linearize that port.
GTEST_TEST(TestLinearize, LinearizingOnAbstractPortThrows) {
  EmptyStateSystemWithAbstractInput<double> system;
  auto context = system.CreateDefaultContext();
  DRAKE_EXPECT_THROWS_MESSAGE(Linearize(system, *context), std::logic_error,
      "Port requested for differentiation is abstract, and differentiation of "
      "abstract ports is not supported.");
}

// Test linearizing a system with mixed (vector and abstract) inputs.
GTEST_TEST(TestLinearize, LinearizingWithMixedInputs) {
  EmptyStateSystemWithMixedInputs<double> system;
  auto context = system.CreateDefaultContext();

  // First check without the vector-valued input port connected.
  DRAKE_EXPECT_THROWS_MESSAGE(Linearize(system, *context), std::logic_error,
                              "InputPort.*is not connected");

  // Now check with the vector-valued input port connected but the abstract
  // input port not yet connected.
  system.get_input_port(0).FixValue(context.get(), 0.0);
  DRAKE_EXPECT_NO_THROW(Linearize(system, *context));

  // Now check with the abstract input port connected.
  system.get_input_port(1).FixValue(context.get(), std::vector<double>());
  DRAKE_EXPECT_NO_THROW(Linearize(system, *context));
}

// Test that Linearize throws when called on a discrete but non-periodic system.
GTEST_TEST(TestLinearize, ThrowsWithNonPeriodicDiscreteSystem) {
  TestNonPeriodicSystem system;
  auto context = system.CreateDefaultContext();

  EXPECT_THROW(Linearize(system, *context), std::runtime_error);
}

// Test a few simple systems that are known to be controllable (or not).
GTEST_TEST(TestLinearize, Controllability) {
  Eigen::Matrix2d A;
  Eigen::Matrix<double, 2, 1> B;
  Eigen::Matrix<double, 0, 2> C;
  Eigen::Matrix<double, 0, 1> D;

  // Controllable system: x1dot = x2, x2dot = u (aka xddot = u).
  A << 0, 1, 0, 0;
  B << 0, 1;
  LinearSystem<double> sys1(A, B, C, D);

  EXPECT_TRUE(IsControllable(sys1));

  // Uncontrollable system: x1dot = u, x2dot = u.
  A << 0, 0, 0, 0;
  B << 1, 1;
  LinearSystem<double> sys2(A, B, C, D);

  EXPECT_FALSE(IsControllable(sys2));
}

// Test a few simple systems that are known to be observable (or not).
GTEST_TEST(TestLinearize, Observability) {
  Eigen::Matrix2d A;
  Eigen::Matrix<double, 2, 1> B;
  Eigen::Matrix<double, 2, 2> C;
  Eigen::Matrix<double, 2, 1> D;

  // x1dot = x2, x2dot = u (aka  xddot = u).
  A << 0, 1, 0, 0;
  B << 0, 1;
  D << 0, 0;

  // Observable: y = x.
  C << 1, 0, 0, 1;
  LinearSystem<double> sys1(A, B, C, D);
  EXPECT_TRUE(IsObservable(sys1));

  // Unobservable: y = x2;
  LinearSystem<double> sys2(A, B, C.bottomRows(1), D.bottomRows(1));
  EXPECT_FALSE(IsObservable(sys2));

  // Observable: y = x1;
  LinearSystem<double> sys3(A, B, C.topRows(1), D.topRows(1));
  EXPECT_TRUE(IsObservable(sys3));
}

class LinearSystemSymbolicTest : public ::testing::Test {
 public:
  LinearSystemSymbolicTest() = default;
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LinearSystemSymbolicTest)

 protected:
  void SetUp() override {
    // clang-format off
  A_  << 1, 0, 3,
        -4, 5, 0,
         7, 0, 9;
  B_  << 10, -7,
         12, 0,
         0,  15;
  C_  <<  1, 2,  0,
         -4, 0, -7;
  D_  << -3,  9,
          0, 13;
  x_ << x0_, x1_, x2_;
  u_ << u0_, u1_;
    // clang-format on
  }

  const symbolic::Variable x0_{"x0"};
  const symbolic::Variable x1_{"x1"};
  const symbolic::Variable x2_{"x2"};
  const symbolic::Variable u0_{"u0"};
  const symbolic::Variable u1_{"u1"};
  Eigen::MatrixXd A_{3, 3};
  Eigen::MatrixXd B_{3, 2};
  Eigen::MatrixXd C_{2, 3};
  Eigen::MatrixXd D_{2, 2};
  VectorX<symbolic::Variable> x_{3};
  VectorX<symbolic::Variable> u_{2};
};

TEST_F(LinearSystemSymbolicTest, MakeLinearSystem) {
  // Checks if MakeLinearSystem() parses the arguments and build a
  // system correctly.
  const auto dut = LinearSystem<double>::MakeLinearSystem(
      A_ * x_ + B_ * u_, C_ * x_ + D_ * u_, x_, u_, 10.0);
  EXPECT_EQ(dut->A(), A_);
  EXPECT_EQ(dut->B(), B_);
  EXPECT_EQ(dut->C(), C_);
  EXPECT_EQ(dut->D(), D_);
  EXPECT_EQ(dut->time_period(), 10.0);
}

// Adds quadratic terms to check if we have an exception. Note that we have
// similar testcases in drake/common/test/symbolic_decompose_test.cc file but we
// believe that having redundancy is not bad in testing.
TEST_F(LinearSystemSymbolicTest, MakeLinearSystemException1) {
  VectorX<symbolic::Expression> extra_terms(3);
  // clang-format off
  extra_terms << x0_ * x0_,
                 x1_ * x1_,
                 x2_ * x2_;
  // clang-format on
  EXPECT_THROW(
      LinearSystem<double>::MakeLinearSystem(extra_terms + A_ * x_ + B_ * u_,
                                             C_ * x_ + D_ * u_, x_, u_, 10.0),
      std::runtime_error);
}

// Adds bilinear terms to check if we have an exception.
TEST_F(LinearSystemSymbolicTest, MakeLinearSystemException2) {
  VectorX<symbolic::Expression> extra_terms(3);
  // clang-format off
  extra_terms << x0_ * u0_,
                 x1_ * u1_,
                 x2_ * u0_;
  // clang-format on
  EXPECT_THROW(
      LinearSystem<double>::MakeLinearSystem(extra_terms + A_ * x_ + B_ * u_,
                                             C_ * x_ + D_ * u_, x_, u_, 10.0),
      std::runtime_error);
}

// Adds nonlinear terms to check if we have an exception.
TEST_F(LinearSystemSymbolicTest, MakeLinearSystemException3) {
  VectorX<symbolic::Expression> extra_terms(3);
  // clang-format off
  extra_terms << sin(x0_),
                 cos(x1_),
                 log(u0_);
  // clang-format on
  EXPECT_THROW(
      LinearSystem<double>::MakeLinearSystem(extra_terms + A_ * x_ + B_ * u_,
                                             C_ * x_ + D_ * u_, x_, u_, 10.0),
      std::runtime_error);
}

// Adds constant terms to check if we have an exception.
TEST_F(LinearSystemSymbolicTest, MakeLinearSystemException4) {
  VectorX<symbolic::Expression> extra_terms(3);
  // clang-format off
  extra_terms << -1,
                  1,
                  M_PI;
  // clang-format on
  EXPECT_THROW(
      LinearSystem<double>::MakeLinearSystem(extra_terms + A_ * x_ + B_ * u_,
                                             C_ * x_ + D_ * u_, x_, u_, 10.0),
      std::runtime_error);
}

TEST_F(LinearSystemTest, LinearizeSystemWithParameters) {
  examples::pendulum::PendulumPlant<double> pendulum;
  auto context = pendulum.CreateDefaultContext();
  auto input = examples::pendulum::PendulumInput<double>();
  input.set_tau(0.0);
  pendulum.get_input_port().FixValue(context.get(), input);
  examples::pendulum::PendulumState<double>& state =
      dynamic_cast<examples::pendulum::PendulumState<double>&>(
          context->get_mutable_continuous_state_vector());
  state.set_theta(0.0);
  state.set_thetadot(0.0);

  std::unique_ptr<LinearSystem<double>> linearized_pendulum =
      Linearize(pendulum, *context);

  const auto params =
      dynamic_cast<const examples::pendulum::PendulumParams<double>*>(
          &context->get_numeric_parameter(0));
  EXPECT_TRUE(params);

  // Compare against manual linearization of the pendulum dynamics.
  using std::pow;
  const double inertia = params->mass() * pow(params->length(), 2.0);
  Eigen::Matrix2d A;
  A << 0, 1.0, -params->gravity() / params->length(),
      -params->damping() / inertia;
  Eigen::Vector2d B(0.0, 1.0 / inertia);
  Eigen::Matrix2d C = Eigen::Matrix2d::Identity();
  Eigen::Vector2d D = Eigen::Vector2d::Zero();

  const double tol = 1e-6;
  EXPECT_TRUE(CompareMatrices(linearized_pendulum->A(), A, tol));
  EXPECT_TRUE(CompareMatrices(linearized_pendulum->B(), B, tol));
  EXPECT_TRUE(CompareMatrices(linearized_pendulum->C(), C, tol));
  EXPECT_TRUE(CompareMatrices(linearized_pendulum->D(), D, tol));
}

GTEST_TEST(LinearizeTest, NoState) {
  const double tol = 1e-6;
  const Eigen::Matrix<double, 0, 0> A;
  const Eigen::Matrix<double, 0, 2> B;
  const Eigen::Matrix<double, 0, 1> f0;
  const Eigen::Matrix<double, 2, 0> C;
  Eigen::Matrix2d D;
  D << 1, 2, 3, 4;
  Eigen::Vector2d y0;
  y0 << 5, 6;

  // Check that I get back the original system (continuous time).
  const AffineSystem<double> sys(A, B, f0, C, D, y0);
  auto context = sys.CreateDefaultContext();
  sys.get_input_port().FixValue(context.get(), Eigen::Vector2d{7, 8});

  const auto taylor_sys = FirstOrderTaylorApproximation(sys, *context);
  EXPECT_TRUE(CompareMatrices(taylor_sys->A(), A, tol));
  EXPECT_TRUE(CompareMatrices(taylor_sys->B(), B, tol));
  EXPECT_TRUE(CompareMatrices(taylor_sys->f0(), f0, tol));
  EXPECT_TRUE(CompareMatrices(taylor_sys->C(), C, tol));
  EXPECT_TRUE(CompareMatrices(taylor_sys->D(), D, tol));
  EXPECT_TRUE(CompareMatrices(taylor_sys->y0(), y0, tol));

  // Check that I get back the original system (discrete time).
  const AffineSystem<double> sys2(A, B, f0, C, D, y0, 0.1);
  auto context2 = sys2.CreateDefaultContext();
  sys2.get_input_port().FixValue(context2.get(), Eigen::Vector2d{7, 8});

  const auto taylor_sys2 = FirstOrderTaylorApproximation(sys2, *context2);
  EXPECT_TRUE(CompareMatrices(taylor_sys2->A(), A, tol));
  EXPECT_TRUE(CompareMatrices(taylor_sys2->B(), B, tol));
  EXPECT_TRUE(CompareMatrices(taylor_sys2->f0(), f0, tol));
  EXPECT_TRUE(CompareMatrices(taylor_sys2->C(), C, tol));
  EXPECT_TRUE(CompareMatrices(taylor_sys2->D(), D, tol));
  EXPECT_TRUE(CompareMatrices(taylor_sys2->y0(), y0, tol));
}

// Simple linear system with multiple vector inputs and outputs.
template <typename T>
class MimoSystem final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MimoSystem);

  explicit MimoSystem(bool is_discrete)
      : LeafSystem<T>(SystemTypeTag<MimoSystem>{}),
        is_discrete_(is_discrete) {
    this->DeclareInputPort(kUseDefaultName, kVectorValued, 1);
    this->DeclareInputPort(kUseDefaultName, kVectorValued, 3);

    if (is_discrete) {
      this->DeclareDiscreteState(2);
      this->DeclarePeriodicDiscreteUpdate(0.1, 0.0);
    } else {
      this->DeclareContinuousState(2);
    }
    this->DeclareVectorOutputPort(kUseDefaultName, 1, &MimoSystem::CalcOutput0);
    this->DeclareVectorOutputPort(kUseDefaultName, 3, &MimoSystem::CalcOutput1);

    A_ << 1, 2, 3, 4;
    B0_ << 5, 6;
    B1_ << 7, 8, 9, 10, 11, 12;
    C0_ << 13, 14;
    C1_ << 15, 16, 17, 18, 19, 19.5;
    D00_ << 20;
    D01_ << 21, 22, 23;
    D10_ << 24, 25, 26;
    D11_ << 27, 28, 29, 30, 31, 32, 33, 34, 35;
  }

  template <typename U>
  explicit MimoSystem(const MimoSystem<U>& other)
      : MimoSystem<T>(other.is_discrete_) {}

  Vector2<T> get_state_vector(const Context<T>& context) const {
    if (is_discrete_) {
      return context.get_discrete_state(0).CopyToVector();
    }
    return context.get_continuous_state_vector().CopyToVector();
  }

  void DoCalcTimeDerivatives(const Context<T>& context,
                             ContinuousState<T>* derivatives) const final {
    Vector1<T> u0 = this->get_input_port(0).Eval(context);
    Vector3<T> u1 = this->get_input_port(1).Eval(context);
    Vector2<T> x = get_state_vector(context);

    derivatives->SetFromVector(A_ * x + B0_ * u0 + B1_ * u1);
  }

  void DoCalcDiscreteVariableUpdates(
      const Context<T>& context,
      const std::vector<const DiscreteUpdateEvent<T>*>&,
      DiscreteValues<T>* discrete_state) const final {
    Vector1<T> u0 = this->get_input_port(0).Eval(context);
    Vector3<T> u1 = this->get_input_port(1).Eval(context);
    Vector2<T> x = get_state_vector(context);

    discrete_state->set_value(0, A_ * x + B0_ * u0 + B1_ * u1);
  }

  void CalcOutput0(const Context<T>& context, BasicVector<T>* output) const {
    Vector1<T> u0 = this->get_input_port(0).Eval(context);
    Vector3<T> u1 = this->get_input_port(1).Eval(context);
    Vector2<T> x = get_state_vector(context);

    output->SetFromVector(C0_ * x + D00_ * u0 + D01_ * u1);
  }

  void CalcOutput1(const Context<T>& context, BasicVector<T>* output) const {
    Vector1<T> u0 = this->get_input_port(0).Eval(context);
    Vector3<T> u1 = this->get_input_port(1).Eval(context);
    Vector2<T> x = get_state_vector(context);

    output->SetFromVector(C1_ * x + D10_ * u0 + D11_ * u1);
  }

  Eigen::Matrix<double, 2, 2> A_;
  Eigen::Matrix<double, 2, 1> B0_;
  Eigen::Matrix<double, 2, 3> B1_;
  Eigen::Matrix<double, 1, 2> C0_;
  Eigen::Matrix<double, 3, 2> C1_;
  Eigen::Matrix<double, 1, 1> D00_;
  Eigen::Matrix<double, 1, 3> D01_;
  Eigen::Matrix<double, 3, 1> D10_;
  Eigen::Matrix<double, 3, 3> D11_;
  const bool is_discrete_{false};
};

void TestMimo(bool is_discrete) {
  MimoSystem<double> sys(is_discrete);
  auto context = sys.CreateDefaultContext();
  // System is linear, so input and state values don't matter.
  sys.get_input_port(0).FixValue(context.get(), 0.0);
  sys.get_input_port(1).FixValue(context.get(), Eigen::Vector3d::Zero());

  const double tol = 1e-8;

  // First-input, first output.
  auto sys00 = Linearize(sys, *context, InputPortIndex{0}, OutputPortIndex{0});
  EXPECT_TRUE(CompareMatrices(sys00->A(), sys.A_, tol));
  EXPECT_TRUE(CompareMatrices(sys00->B(), sys.B0_, tol));
  EXPECT_TRUE(CompareMatrices(sys00->C(), sys.C0_, tol));
  EXPECT_TRUE(CompareMatrices(sys00->D(), sys.D00_, tol));

  auto sys01 = Linearize(sys, *context, InputPortIndex{0}, OutputPortIndex{1});
  EXPECT_TRUE(CompareMatrices(sys01->A(), sys.A_, tol));
  EXPECT_TRUE(CompareMatrices(sys01->B(), sys.B0_, tol));
  EXPECT_TRUE(CompareMatrices(sys01->C(), sys.C1_, tol));
  EXPECT_TRUE(CompareMatrices(sys01->D(), sys.D10_, tol));

  auto sys10 = Linearize(sys, *context, InputPortIndex{1}, OutputPortIndex{0});
  EXPECT_TRUE(CompareMatrices(sys10->A(), sys.A_, tol));
  EXPECT_TRUE(CompareMatrices(sys10->B(), sys.B1_, tol));
  EXPECT_TRUE(CompareMatrices(sys10->C(), sys.C0_, tol));
  EXPECT_TRUE(CompareMatrices(sys10->D(), sys.D01_, tol));

  auto sys11 = Linearize(sys, *context, InputPortIndex{1}, OutputPortIndex{1});
  EXPECT_TRUE(CompareMatrices(sys11->A(), sys.A_, tol));
  EXPECT_TRUE(CompareMatrices(sys11->B(), sys.B1_, tol));
  EXPECT_TRUE(CompareMatrices(sys11->C(), sys.C1_, tol));
  EXPECT_TRUE(CompareMatrices(sys11->D(), sys.D11_, tol));
}

GTEST_TEST(LinearizeTest, TestInputOutputPorts) {
  // Continuous-time version.
  TestMimo(false);

  // Discrete-time version.
  TestMimo(true);
}

GTEST_TEST(LinearSystemBespokeTest, InfiniteRecursionDuringCalc) {
  // Ensure we do not segfault when connecting a double-integrator with a
  // "controller", both implemented using LinearSystem (#12706).
  DiagramBuilder<double> builder;

  // Create a simple double-integrator plant.
  MatrixXd Ap(2, 2);
  Ap << 0, 1, 0, 0;
  MatrixXd Bp(2, 1);
  Bp << 0, 1;
  MatrixXd Cp = MatrixXd::Identity(2, 2);
  MatrixXd Dp = MatrixXd::Zero(2, 1);
  auto* plant = builder.AddSystem(
      std::make_unique<LinearSystem<double>>(Ap, Bp, Cp, Dp));

  // Create a simple PD-controller.
  MatrixXd Ac(0, 0);
  MatrixXd Bc(0, 2);
  MatrixXd Cc(1, 0);
  MatrixXd Dc(1, 2);
  Dc << -1, -1;
  auto* controller = builder.AddSystem(
      std::make_unique<LinearSystem<double>>(Ac, Bc, Cc, Dc));

  // Connect, build, and allocate a context.
  builder.Connect(controller->get_output_port(), plant->get_input_port());
  builder.Connect(plant->get_output_port(), controller->get_input_port());
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  // Compute the output, namely to ensure that we do not segfault.
  const VectorXd y =
      plant->get_output_port().Eval(plant->GetMyContextFromRoot(*context));
  const VectorXd y_expected = VectorXd::Zero(2);
  EXPECT_TRUE(CompareMatrices(y, y_expected));

  // Check feedthrough.
  EXPECT_FALSE(plant->HasDirectFeedthrough(0));
  EXPECT_TRUE(controller->HasDirectFeedthrough(0));
}

class DoubleOnlyLinearSystem final : public LinearSystem<double> {
 public:
  DoubleOnlyLinearSystem(
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::MatrixXd>& B,
      const Eigen::Ref<const Eigen::MatrixXd>& C,
      const Eigen::Ref<const Eigen::MatrixXd>& D)
      : LinearSystem(SystemScalarConverter{}, A, B, C, D, 0.0) {}
};

// Tests that non-feedthrough is available without symbolic expressions.
GTEST_TEST(LinearSystemBespokeTest, NonSymbolicFeedthrough) {
  const MatrixXd ones = MatrixXd::Ones(1, 1);
  const MatrixXd zeros = MatrixXd::Zero(1, 1);
  const DoubleOnlyLinearSystem dut1(ones, ones, ones, ones);
  const DoubleOnlyLinearSystem dut2(ones, ones, ones, zeros);
  EXPECT_EQ(dut1.ToSymbolicMaybe(), nullptr);
  EXPECT_EQ(dut2.ToSymbolicMaybe(), nullptr);
  EXPECT_TRUE(dut1.HasAnyDirectFeedthrough());
  EXPECT_FALSE(dut2.HasAnyDirectFeedthrough());
}

}  // namespace
}  // namespace systems
}  // namespace drake
