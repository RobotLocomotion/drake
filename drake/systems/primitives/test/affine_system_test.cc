#include "drake/systems/primitives/affine_system.h"

#include "drake/common/eigen_matrix_compare.h"
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
    state_ = context_->get_mutable_continuous_state();
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
  EXPECT_EQ(updates_->size(), 0);
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
  EXPECT_FALSE(dut_->has_any_direct_feedthrough());
}

// Tests that the system renders as a direct feedthrough
// when a single element of D is set to 1e-12.
TEST_F(FeedthroughAffineSystemTest, FeedthroughTest) {
  SetDCornerElement(1e-12);
  InitialiseSystem();
  EXPECT_TRUE(dut_->has_any_direct_feedthrough());
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

  context->get_mutable_discrete_state(0)->SetFromVector(x0);
  double u0 = 29;
  context->FixInputPort(0, Vector1d::Constant(u0));

  auto update = system.AllocateDiscreteVariables();
  DiscreteEvent<double> update_event;
  update_event.action = DiscreteEvent<double>::kDiscreteUpdateAction;

  system.CalcDiscreteVariableUpdates(*context, update_event, update.get());

  EXPECT_TRUE(CompareMatrices(update->get_discrete_state(0)->CopyToVector(),
                              A * x0 + B * u0 + f0));

  // Test TimeVaryingAffineSystem accessor methods.
  const double t = 3.0;
  EXPECT_TRUE(CompareMatrices(system.A(t), A));
  EXPECT_TRUE(CompareMatrices(system.B(t), B));
  EXPECT_TRUE(CompareMatrices(system.f0(t), f0));
  EXPECT_TRUE(CompareMatrices(system.C(t), C));
  EXPECT_TRUE(CompareMatrices(system.D(t), D));
  EXPECT_TRUE(CompareMatrices(system.y0(t), y0));
}

// xdot = rotmat(t)*x, y = x;
class SimpleTimeVaryingAffineSystem : public TimeVaryingAffineSystem<double>,
                                      public ::testing::Test {
 public:
  SimpleTimeVaryingAffineSystem() : TimeVaryingAffineSystem(2, 0, 2) {}

  Eigen::MatrixXd A(const double& t) const override {
    Eigen::Matrix2d mat;
    mat << std::cos(t), -std::sin(t), std::sin(t), std::cos(t);
    return mat;
  }
  Eigen::MatrixXd B(const double& t) const override {
    return Eigen::Matrix<double, 2, 0>();
  }
  Eigen::VectorXd f0(const double& t) const override {
    return Eigen::Matrix<double, 2, 1>::Zero();
  }
  Eigen::MatrixXd C(const double& t) const override {
    return Eigen::Matrix2d::Identity();
  }
  Eigen::MatrixXd D(const double& t) const override {
    return Eigen::Matrix<double, 2, 0>();
  }
  Eigen::VectorXd y0(const double& t) const override {
    return Eigen::Matrix<double, 2, 1>::Zero();
  }
};

TEST_F(SimpleTimeVaryingAffineSystem, EvalTest) {
  const double t = 2.5;
  Eigen::Matrix2d A;
  A << std::cos(t), -std::sin(t), std::sin(t), std::cos(t);
  Eigen::Vector2d x(1, 2);

  auto context = CreateDefaultContext();
  context->set_time(t);
  context->get_mutable_continuous_state_vector()->SetFromVector(x);

  auto derivs = AllocateTimeDerivatives();
  CalcTimeDerivatives(*context, derivs.get());
  EXPECT_TRUE(CompareMatrices(A * x, derivs->CopyToVector()));

  auto output = AllocateOutput(*context);
  CalcOutput(*context, output.get());
  EXPECT_TRUE(CompareMatrices(x, output->get_vector_data(0)->CopyToVector()));
}

// Checks that a time-varying affine system will fail if the matrices do not
// match the specified number of states.
class IllegalTimeVaryingAffineSystem : public SimpleTimeVaryingAffineSystem {
  Eigen::MatrixXd A(const double& t) const override {
    Eigen::Matrix<double, 3, 2> mat;
    mat << std::cos(t), -std::sin(t), std::sin(t), std::cos(t), 0, 1;
    return mat;
  }
};

TEST_F(IllegalTimeVaryingAffineSystem, EvalDeathTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  const double t = 2.5;

  auto context = CreateDefaultContext();
  context->set_time(t);

  auto derivatives = AllocateTimeDerivatives();
  ASSERT_DEATH(CalcTimeDerivatives(*context, derivatives.get()), "rows");
}

}  // namespace
}  // namespace systems
}  // namespace drake
