#include "drake/systems/primitives/affine_system.h"

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"
#include "drake/systems/primitives/test/affine_linear_test.h"

using std::make_unique;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace {

class AffineSystemTest : public AffineLinearSystemTest {
 public:
  // Setup an arbitrary AffineSystem.
  AffineSystemTest() : AffineLinearSystemTest(-4.5, 6.5, 3.5, -7.6) {}

  void Initialize() override {
    // Construct the system I/O objects.
    dut_ = make_unique<AffineSystem<double>>(A_, B_, f0_, C_, D_, y0_);
    dut_->set_name("test_affine_system");
    context_ = dut_->CreateDefaultContext();
    input_vector_ = make_unique<BasicVector<double>>(2 /* size */);
    system_output_ = dut_->AllocateOutput(*context_);
    state_ = &context_->get_mutable_continuous_state();
    derivatives_ = dut_->AllocateTimeDerivatives();
    updates_ = dut_->AllocateDiscreteVariables();
  }

 protected:
  // The Device Under Test is an AffineSystem<double>.
  unique_ptr<AffineSystem<double>> dut_;
};

// Tests that the affine system is correctly setup.
TEST_F(AffineSystemTest, Construction) {
  EXPECT_EQ(1, context_->get_num_input_ports());
  EXPECT_EQ("test_affine_system", dut_->get_name());
  EXPECT_EQ(dut_->A(), A_);
  EXPECT_EQ(dut_->B(), B_);
  EXPECT_EQ(dut_->C(), C_);
  EXPECT_EQ(dut_->D(), D_);
  EXPECT_EQ(dut_->f0(), f0_);
  EXPECT_EQ(dut_->y0(), y0_);
  EXPECT_EQ(dut_->get_num_output_ports(), 1);
  EXPECT_EQ(dut_->get_num_input_ports(), 1);

  // Test TimeVaryingAffineSystem accessor methods.
  const double t = 3.5;
  EXPECT_TRUE(CompareMatrices(dut_->A(t), A_));
  EXPECT_TRUE(CompareMatrices(dut_->B(t), B_));
  EXPECT_TRUE(CompareMatrices(dut_->f0(t), f0_));
  EXPECT_TRUE(CompareMatrices(dut_->C(t), C_));
  EXPECT_TRUE(CompareMatrices(dut_->D(t), D_));
  EXPECT_TRUE(CompareMatrices(dut_->y0(t), y0_));
}

// Tests that the derivatives are correctly computed.
TEST_F(AffineSystemTest, Derivatives) {
  Eigen::Vector2d u(1, 4);
  SetInput(u);

  Eigen::Vector2d x(0.1, 0.25);
  state_->SetFromVector(x);

  EXPECT_NE(derivatives_, nullptr);
  dut_->CalcTimeDerivatives(*context_, derivatives_.get());

  Eigen::VectorXd expected_derivatives(2);
  expected_derivatives = A_ * x + B_ * u + f0_;

  EXPECT_TRUE(CompareMatrices(
      expected_derivatives, derivatives_->get_vector().CopyToVector(), 1e-10));
}

// Tests that the updates are correctly (not) computed.
TEST_F(AffineSystemTest, Updates) {
  EXPECT_TRUE(context_->has_only_continuous_state());
  EXPECT_NE(updates_, nullptr);
  EXPECT_EQ(updates_->num_groups(), 0);
}

// Tests that the outputs are correctly computed.
TEST_F(AffineSystemTest, Output) {
  // Sets the context's input port.
  Eigen::Vector2d u(5.6, -10.1);
  SetInput(u);

  // Sets the state.
  Eigen::Vector2d x(0.8, -22.1);
  state_->SetFromVector(x);

  dut_->CalcOutput(*context_, system_output_.get());

  Eigen::VectorXd expected_output(2);

  expected_output = C_ * x + D_ * u + y0_;

  EXPECT_TRUE(CompareMatrices(
      expected_output, system_output_->get_vector_data(0)->get_value(), 1e-10));
}

// Tests converting to different scalar types.
TEST_F(AffineSystemTest, ConvertScalarType) {
  EXPECT_TRUE(is_autodiffxd_convertible(*dut_, [&](const auto& converted) {
    EXPECT_EQ(converted.A(), A_);
    EXPECT_EQ(converted.B(), B_);
    EXPECT_EQ(converted.f0(), f0_);
    EXPECT_EQ(converted.C(), C_);
    EXPECT_EQ(converted.D(), D_);
    EXPECT_EQ(converted.y0(), y0_);
  }));
  EXPECT_TRUE(is_symbolic_convertible(*dut_, [&](const auto& converted) {
    EXPECT_EQ(converted.A(), A_);
    EXPECT_EQ(converted.B(), B_);
    EXPECT_EQ(converted.f0(), f0_);
    EXPECT_EQ(converted.C(), C_);
    EXPECT_EQ(converted.D(), D_);
    EXPECT_EQ(converted.y0(), y0_);
  }));
}

class FeedthroughAffineSystemTest : public ::testing::Test {
 public:
  void SetDCornerElement(double d_1_1_element) {
    d_1_1_element_ = d_1_1_element;
  }

  void InitialiseSystem() {
    // Setup an arbitrary AffineSystem which is feedthrough if and only if
    // the elements of D matrix are exactly 0.
    Eigen::MatrixXd A_(
        AffineLinearSystemTest::make_2x2_matrix(1.5, 2.7, 3.5, -4.9));
    Eigen::MatrixXd B_(
        AffineLinearSystemTest::make_2x2_matrix(4.9, -5.1, 6.8, 7.2));
    Eigen::VectorXd f0_(AffineLinearSystemTest::make_2x1_vector(0, 0));
    Eigen::MatrixXd C_(
        AffineLinearSystemTest::make_2x2_matrix(1.1, 2.5, -3.8, 4.6));
    Eigen::MatrixXd D_(
        AffineLinearSystemTest::make_2x2_matrix(d_1_1_element_, 0, 0, 0));
    Eigen::VectorXd y0_(AffineLinearSystemTest::make_2x1_vector(0, 0));
    dut_ = make_unique<AffineSystem<double>>(A_, B_, f0_, C_, D_, y0_);
    dut_->set_name("test_feedtroughaffine_system");
  }

 protected:
  // The Device Under Test is an AffineSystem<double>.
  unique_ptr<AffineSystem<double>> dut_;
  double d_1_1_element_{0.0};
};

// Tests that the system does not render as a direct feedthrough
// when all elements of D are exactly 0.
TEST_F(FeedthroughAffineSystemTest, NoFeedthroughTest) {
  SetDCornerElement(0.0);
  InitialiseSystem();
  EXPECT_FALSE(dut_->HasAnyDirectFeedthrough());
}

// Tests that the system renders as a direct feedthrough
// when a single element of D is set to 1e-12.
TEST_F(FeedthroughAffineSystemTest, FeedthroughTest) {
  SetDCornerElement(1e-12);
  InitialiseSystem();
  EXPECT_TRUE(dut_->HasAnyDirectFeedthrough());
}

// Tests the discrete-time update.
GTEST_TEST(DiscreteAffineSystemTest, DiscreteTime) {
  Eigen::Matrix3d A;
  Eigen::Matrix<double, 3, 1> B;
  Eigen::Vector3d f0;
  Eigen::Matrix<double, 2, 3> C;
  Eigen::Vector2d D;
  Eigen::Vector2d y0;
  A << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  B << 10, 11, 12;
  f0 << 13, 14, 15;
  C << 16, 17, 18, 19, 20, 21;
  D << 22, 23;
  y0 << 24, 25;
  AffineSystem<double> system(A, B, f0, C, D, y0, 1.0);
  auto context = system.CreateDefaultContext();
  EXPECT_TRUE(context->has_only_discrete_state());

  Eigen::Vector3d x0(26, 27, 28);

  context->get_mutable_discrete_state(0).SetFromVector(x0);
  double u0 = 29;
  context->FixInputPort(0, Vector1d::Constant(u0));

  auto update = system.AllocateDiscreteVariables();
  system.CalcDiscreteVariableUpdates(*context, update.get());

  EXPECT_TRUE(CompareMatrices(update->get_vector(0).CopyToVector(),
                              A * x0 + B * u0 + f0));

  // Test TimeVaryingAffineSystem accessor methods.
  const double t = 3.0;
  EXPECT_TRUE(CompareMatrices(system.A(t), A));
  EXPECT_TRUE(CompareMatrices(system.B(t), B));
  EXPECT_TRUE(CompareMatrices(system.f0(t), f0));
  EXPECT_TRUE(CompareMatrices(system.C(t), C));
  EXPECT_TRUE(CompareMatrices(system.D(t), D));
  EXPECT_TRUE(CompareMatrices(system.y0(t), y0));

  // Compare the calculated output against the expected output.
  auto system_output = system.AllocateOutput(*context);
  system.CalcOutput(*context, system_output.get());
  EXPECT_TRUE(CompareMatrices(system_output->get_vector_data(0)->get_value(),
                              C * x0 + D * u0 + y0));
}

// [ẋ₁, ẋ₂]ᵀ = rotmat(t)*[x₁, x₂]ᵀ + u*[1, 1]ᵀ,
// [y₁, y₂] = [x₁, x₂] + (u + 1)*[1, 1].
class SimpleTimeVaryingAffineSystem : public TimeVaryingAffineSystem<double> {
 public:
  static constexpr int kNumStates = 2;
  static constexpr int kNumInputs = 1;
  static constexpr int kNumOutputs = 2;

  explicit SimpleTimeVaryingAffineSystem(double time_period)
      : TimeVaryingAffineSystem(
            SystemScalarConverter{},  // BR
            kNumStates, kNumInputs, kNumOutputs, time_period) {}
  ~SimpleTimeVaryingAffineSystem() override {}

  Eigen::MatrixXd A(const double& t) const override {
    Eigen::Matrix<double, kNumOutputs, kNumStates> mat;
    mat << std::cos(t), -std::sin(t), std::sin(t), std::cos(t);
    return mat;
  }
  Eigen::MatrixXd B(const double& t) const override {
    return Eigen::Matrix<double, kNumOutputs, kNumInputs>::Ones();
  }
  Eigen::VectorXd f0(const double& t) const override {
    return Eigen::Matrix<double, kNumOutputs, 1>::Zero();
  }
  Eigen::MatrixXd C(const double& t) const override {
    return Eigen::Matrix2d::Identity();
  }
  Eigen::MatrixXd D(const double& t) const override {
    return Eigen::Matrix<double, kNumOutputs, kNumInputs>::Ones();
  }
  Eigen::VectorXd y0(const double& t) const override {
    return Eigen::Matrix<double, kNumOutputs, 1>::Ones();
  }
};

GTEST_TEST(SimpleTimeVaryingAffineSystemTest, EvalTest) {
  SimpleTimeVaryingAffineSystem sys(0.0);  // A continuous-time system.
  const double t = 2.5;
  Eigen::Vector2d x(1, 2);

  auto context = sys.CreateDefaultContext();
  context->set_time(t);
  context->get_mutable_continuous_state_vector().SetFromVector(x);
  context->FixInputPort(0, BasicVector<double>::Make(42.0));

  auto derivs = sys.AllocateTimeDerivatives();
  sys.CalcTimeDerivatives(*context, derivs.get());
  EXPECT_TRUE(CompareMatrices(sys.A(t) * x + 42.0 * sys.B(t),
                              derivs->CopyToVector()));

  auto output = sys.AllocateOutput(*context);
  sys.CalcOutput(*context, output.get());
  EXPECT_TRUE(CompareMatrices(x + sys.y0(t) + 42.0 * sys.D(t),
                              output->get_vector_data(0)->CopyToVector()));
}

GTEST_TEST(SimpleTimeVaryingAffineSystemTest, DiscreteEvalTest) {
  SimpleTimeVaryingAffineSystem sys(1.0);  // A discrete-time system.
  const double t = 2.5;
  Eigen::Vector2d x(1, 2);

  auto context = sys.CreateDefaultContext();
  context->set_time(t);
  context->get_mutable_discrete_state().get_mutable_vector().SetFromVector(x);
  context->FixInputPort(0, BasicVector<double>::Make(42.0));

  auto updates = sys.AllocateDiscreteVariables();
  sys.CalcDiscreteVariableUpdates(*context, updates.get());
  EXPECT_TRUE(CompareMatrices(sys.A(t) * x + 42.0 * sys.B(t),
                              updates->get_vector().CopyToVector()));

  auto output = sys.AllocateOutput(*context);
  sys.CalcOutput(*context, output.get());
  EXPECT_TRUE(CompareMatrices(x + sys.y0(t) + 42.0 * sys.D(t),
                              output->get_vector_data(0)->CopyToVector()));
}

// Checks that a time-varying affine system will fail if the matrices do not
// match the specified number of states.
class IllegalTimeVaryingAffineSystem : public SimpleTimeVaryingAffineSystem {
 public:
  IllegalTimeVaryingAffineSystem() : SimpleTimeVaryingAffineSystem(0.0) {}
  ~IllegalTimeVaryingAffineSystem() override {}

  Eigen::MatrixXd A(const double& t) const override {
    Eigen::Matrix<double, 3, 2> mat;
    mat << std::cos(t), -std::sin(t), std::sin(t), std::cos(t), 0, 1;
    return mat;
  }
};

GTEST_TEST(IllegalTimeVaryingAffineSystemTest, EvalDeathTest) {
  IllegalTimeVaryingAffineSystem sys;
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  const double t = 2.5;

  auto context = sys.CreateDefaultContext();
  context->set_time(t);

  auto derivatives = sys.AllocateTimeDerivatives();
  ASSERT_DEATH(sys.CalcTimeDerivatives(*context, derivatives.get()), "rows");
}

class AffineSystemSymbolicTest : public ::testing::Test {
 public:
  AffineSystemSymbolicTest() = default;
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AffineSystemSymbolicTest)

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
  f0_ << 3,
        -7,
         2;
  y0_ << 6,
        -7;
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
  Eigen::VectorXd f0_{3};
  Eigen::VectorXd y0_{2};
  VectorX<symbolic::Variable> x_{3};
  VectorX<symbolic::Variable> u_{2};
};

TEST_F(AffineSystemSymbolicTest, MakeAffineSystem) {
  // Checks if MakeAffineSystem() parses the arguments and build a
  // system correctly.
  const auto dut = AffineSystem<double>::MakeAffineSystem(
      A_ * x_ + B_ * u_ + f0_, C_ * x_ + D_ * u_ + y0_, x_, u_, 10.0);
  EXPECT_EQ(dut->A(), A_);
  EXPECT_EQ(dut->B(), B_);
  EXPECT_EQ(dut->C(), C_);
  EXPECT_EQ(dut->D(), D_);
  EXPECT_EQ(dut->f0(), f0_);
  EXPECT_EQ(dut->y0(), y0_);
  EXPECT_EQ(dut->time_period(), 10.0);
}

// Adds quadratic terms to check if we have an exception. Note that we have
// similar testcases in drake/common/test/symbolic_decompose_test.cc file but we
// believe that having redundancy is not bad in testing.
TEST_F(AffineSystemSymbolicTest, MakeAffineSystemException1) {
  VectorX<symbolic::Expression> extra_terms(3);
  // clang-format off
  extra_terms << x0_ * x0_,
                 x1_ * x1_,
                 x2_ * x2_;
  // clang-format on
  EXPECT_THROW(AffineSystem<double>::MakeAffineSystem(
                   extra_terms + A_ * x_ + B_ * u_ + f0_,
                   C_ * x_ + D_ * u_ + y0_, x_, u_, 10.0),
               std::runtime_error);
}

// Adds bilinear terms to check if we have an exception.
TEST_F(AffineSystemSymbolicTest, MakeAffineSystemException2) {
  VectorX<symbolic::Expression> extra_terms(3);
  // clang-format off
  extra_terms << x0_ * u0_,
                 x1_ * u1_,
                 x2_ * u0_;
  // clang-format on
  EXPECT_THROW(AffineSystem<double>::MakeAffineSystem(
                   extra_terms + A_ * x_ + B_ * u_ + f0_,
                   C_ * x_ + D_ * u_ + y0_, x_, u_, 10.0),
               std::runtime_error);
}

// Adds nonlinear terms to check if we have an exception.
TEST_F(AffineSystemSymbolicTest, MakeAffineSystemException3) {
  VectorX<symbolic::Expression> extra_terms(3);
  // clang-format off
  extra_terms << sin(x0_),
                 cos(x1_),
                 log(u0_);
  // clang-format on
  EXPECT_THROW(AffineSystem<double>::MakeAffineSystem(
                   extra_terms + A_ * x_ + B_ * u_ + f0_,
                   C_ * x_ + D_ * u_ + y0_, x_, u_, 10.0),
               std::runtime_error);
}
}  // namespace
}  // namespace systems
}  // namespace drake
