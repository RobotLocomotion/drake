#include "drake/systems/primitives/piecewise_polynomial_affine_system.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"
#include "drake/systems/primitives/test/piecewise_linear_affine_test.h"

using std::make_unique;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace {

using test::kDiscreteTimeStep;
using test::ExampleAffineTimeVaryingData;

enum ConstructorType { FromContinuous, FromDiscrete };

class PiecewisePolynomialAffineSystemTest
    : public ::testing::TestWithParam<double> {
 public:
  void SetUp() override {
    std::tie(ltv_data_, mat_data_) = ExampleAffineTimeVaryingData();

    // Set up an arbitrary PiecewisePolynomialAffineSystem in discrete time.
    const TimeVaryingData data(mat_data_.Avec, mat_data_.Bvec, mat_data_.f0vec,
                               mat_data_.Cvec, mat_data_.Dvec, mat_data_.y0vec,
                               kDiscreteTimeStep);
    if (this->GetParam() == ConstructorType::FromContinuous) {
      dut_ = make_unique<PiecewisePolynomialAffineSystem<double>>(data);
    } else if (this->GetParam() == ConstructorType::FromDiscrete) {
      time_period_ = kDiscreteTimeStep;
      dut_ = make_unique<PiecewisePolynomialAffineSystem<double>>(
          data, kDiscreteTimeStep);
    }
    context_ = dut_->CreateDefaultContext();
    input_vector_ = make_unique<BasicVector<double>>(2 /* size */);
    system_output_ = dut_->AllocateOutput(*context_);
    continuous_state_ = &context_->get_mutable_continuous_state();
    discrete_state_ = &context_->get_mutable_discrete_state();
    derivatives_ = dut_->AllocateTimeDerivatives();
    updates_ = dut_->AllocateDiscreteVariables();
  }

 protected:
  // The Device Under Test (DUT) is a PiecewisePolynomialAffineSystem<double>.
  unique_ptr<PiecewisePolynomialAffineSystem<double>> dut_;
  unique_ptr<Context<double>> context_;
  unique_ptr<SystemOutput<double>> system_output_;

  ContinuousState<double>* continuous_state_{nullptr};
  DiscreteValues<double>* discrete_state_{nullptr};
  unique_ptr<BasicVector<double>> input_vector_;
  unique_ptr<ContinuousState<double>> derivatives_;
  unique_ptr<DiscreteValues<double>> updates_;

  TimeVaryingData ltv_data_;
  test::MatrixData mat_data_;
  double time_period_{0.};  // Defaults to continuous-time.
};

TEST_P(PiecewisePolynomialAffineSystemTest, Constructor) {
  EXPECT_EQ(1, context_->get_num_input_ports());
  EXPECT_EQ(dut_->A(0.), ltv_data_.A.value(0.));
  EXPECT_EQ(dut_->B(0.), ltv_data_.B.value(0.));
  EXPECT_EQ(dut_->C(0.), ltv_data_.C.value(0.));
  EXPECT_EQ(dut_->D(0.), ltv_data_.D.value(0.));
  EXPECT_EQ(dut_->f0(0.), ltv_data_.f0.value(0.));
  EXPECT_EQ(dut_->y0(0.), ltv_data_.y0.value(0.));
  EXPECT_EQ(dut_->time_period(), time_period_);
  EXPECT_EQ(1, dut_->get_num_output_ports());
  EXPECT_EQ(1, dut_->get_num_input_ports());
}

TEST_P(PiecewisePolynomialAffineSystemTest, KnotPointConsistency) {
  for (int i{0}; i < static_cast<int>(mat_data_.times.size()); ++i) {
    EXPECT_TRUE(
        CompareMatrices(dut_->A(mat_data_.times[i]), mat_data_.Avec[i]));
    EXPECT_TRUE(
        CompareMatrices(dut_->B(mat_data_.times[i]), mat_data_.Bvec[i]));
    EXPECT_TRUE(
        CompareMatrices(dut_->C(mat_data_.times[i]), mat_data_.Cvec[i]));
    EXPECT_TRUE(
        CompareMatrices(dut_->D(mat_data_.times[i]), mat_data_.Dvec[i]));
    EXPECT_TRUE(
        CompareMatrices(dut_->f0(mat_data_.times[i]), mat_data_.f0vec[i]));
    EXPECT_TRUE(
        CompareMatrices(dut_->y0(mat_data_.times[i]), mat_data_.y0vec[i]));
  }
}

// Tests that the derivatives and discrete updates are correctly computed.
TEST_P(PiecewisePolynomialAffineSystemTest, DiscreteUpdates) {
  // Sets the context's input port.
  Eigen::Vector2d u(7, 42);
  input_vector_->get_mutable_value() << u;
  context_->FixInputPort(0, std::move(input_vector_));

  EXPECT_NE(derivatives_, nullptr);
  EXPECT_NE(updates_, nullptr);

  // Sets the state.
  Eigen::Vector2d x(0.101, 3.7);
  if (time_period_ == 0.) {
    continuous_state_->SetFromVector(x);
  } else {
    discrete_state_->get_mutable_vector(0).SetFromVector(x);
  }

  const double tol = 1e-10;
  for (const double t : mat_data_.times) {
    context_->set_time(t);
    const Eigen::Matrix2d A = ltv_data_.A.value(t);
    const Eigen::Matrix2d B = ltv_data_.B.value(t);
    const Eigen::Vector2d f0 = ltv_data_.f0.value(t);
    if (time_period_ == 0.) {
      dut_->CalcTimeDerivatives(*context_, derivatives_.get());
      EXPECT_TRUE(CompareMatrices(
          A * x + B * u + f0, derivatives_->get_vector().CopyToVector(), tol));
    } else {
      dut_->CalcDiscreteVariableUpdates(*context_, updates_.get());
      EXPECT_TRUE(CompareMatrices(
          A * x + B * u + f0, updates_->get_vector(0).CopyToVector(), tol));
    }
  }
}

// Tests that the outputs are correctly computed.
TEST_P(PiecewisePolynomialAffineSystemTest, Output) {
  // Sets the context's input port.
  Eigen::Vector2d u(5.6, -10.1);
  input_vector_->get_mutable_value() << u;
  context_->FixInputPort(0, std::move(input_vector_));

  // Sets the state.
  Eigen::Vector2d x(0.8, -22.1);
  if (time_period_ == 0.) {
    continuous_state_->SetFromVector(x);
  } else {
    discrete_state_->get_mutable_vector(0).SetFromVector(x);
  }

  const double tol = 1e-10;
  for (const double t : mat_data_.times) {
    context_->set_time(t);

    dut_->CalcOutput(*context_, system_output_.get());
    const Eigen::Matrix2d C = ltv_data_.C.value(t);
    const Eigen::Matrix2d D = ltv_data_.D.value(t);
    const Eigen::Vector2d y0 = ltv_data_.y0.value(t);

    EXPECT_TRUE(CompareMatrices(C * x + D * u + y0,
                                system_output_->get_vector_data(0)->get_value(),
                                tol));
  }
}

// Tests that conversion to different scalar types is possible.
TEST_P(PiecewisePolynomialAffineSystemTest, ScalarTypeConversion) {
  EXPECT_TRUE(is_autodiffxd_convertible(*dut_, [&](const auto& converted) {
    EXPECT_EQ(converted.A(0.), ltv_data_.A.value(0.));
    EXPECT_EQ(converted.B(0.), ltv_data_.B.value(0.));
    EXPECT_EQ(converted.C(0.), ltv_data_.C.value(0.));
    EXPECT_EQ(converted.D(0.), ltv_data_.D.value(0.));
    EXPECT_EQ(converted.time_period(), time_period_);
  }));
  EXPECT_FALSE(is_symbolic_convertible(*dut_));
}

INSTANTIATE_TEST_CASE_P(Constructor, PiecewisePolynomialAffineSystemTest,
                        testing::Values(ConstructorType::FromContinuous,
                                        ConstructorType::FromDiscrete));

}  // namespace
}  // namespace systems
}  // namespace drake
