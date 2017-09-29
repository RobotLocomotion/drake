#include "drake/systems/primitives/time_varying_linear_system.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

using std::make_unique;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace {

enum ConstructorType {
  FromDataContinuous,
  FromStructContinuous,
  FromStructDiscrete
};

class TimeVaryingLinearSystemTest : public ::testing::TestWithParam<double> {
 public:
  void SetUp() override {
    // Set up an arbitrary TimeVaryingLinearSystem.
    const std::vector<double> times{0., 1.};
    std::vector<Eigen::MatrixXd> Avec(times.size());
    std::vector<Eigen::MatrixXd> Bvec(times.size());
    std::vector<Eigen::MatrixXd> Cvec(times.size());
    std::vector<Eigen::MatrixXd> Dvec(times.size());
    Eigen::Matrix2d A0;
    A0 << 1, 2, 3, 4;
    Eigen::Matrix2d B0;
    B0 << 5, 6, 7, 8;
    Eigen::Matrix2d C0;
    C0 << 9, 10, 11, 12;
    Eigen::Matrix2d D0;
    D0 << 13, 14, 15, 16;
    for (int i{0}; i < static_cast<int>(times.size()); ++i) {
      Avec[i] = A0 + i * Eigen::Matrix2d::Ones();
      Bvec[i] = B0 + i * Eigen::Matrix2d::Ones();
      Cvec[i] = C0 + i * Eigen::Matrix2d::Ones();
      Dvec[i] = D0 + i * Eigen::Matrix2d::Ones();
    }
    const auto Apoly = PiecewisePolynomial<double>::FirstOrderHold(times, Avec);
    const auto Bpoly = PiecewisePolynomial<double>::FirstOrderHold(times, Bvec);
    const auto Cpoly = PiecewisePolynomial<double>::FirstOrderHold(times, Cvec);
    const auto Dpoly = PiecewisePolynomial<double>::FirstOrderHold(times, Dvec);
    data_.A = PiecewisePolynomialTrajectory(Apoly);
    data_.B = PiecewisePolynomialTrajectory(Bpoly);
    data_.C = PiecewisePolynomialTrajectory(Cpoly);
    data_.D = PiecewisePolynomialTrajectory(Dpoly);

    // Construct the system I/O objects.
    if (this->GetParam() == ConstructorType::FromDataContinuous) {
      dut_ = std::make_unique<TimeVaryingLinearSystem<double>>(data_);
    } else if (this->GetParam() == ConstructorType::FromStructContinuous) {
      dut_ = std::make_unique<TimeVaryingLinearSystem<double>>(
          data_.A, data_.B, data_.C, data_.D);
    } else if (this->GetParam() == ConstructorType::FromStructDiscrete) {
      time_period_ = 0.1;
      dut_ = std::make_unique<TimeVaryingLinearSystem<double>>(
          data_.A, data_.B, data_.C, data_.D, time_period_);
    }
    context_ = dut_->CreateDefaultContext();
    input_vector_ = std::make_unique<BasicVector<double>>(2 /* size */);
    system_output_ = dut_->AllocateOutput(*context_);
    continuous_state_ = context_->get_mutable_continuous_state();
    discrete_state_ = context_->get_mutable_discrete_state();
    derivatives_ = dut_->AllocateTimeDerivatives();
    updates_ = dut_->AllocateDiscreteVariables();
  }

 protected:
  // The Device Under Test (DUT) is a TimeVaryingLinearSystem<double>.
  std::unique_ptr<TimeVaryingLinearSystem<double>> dut_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> system_output_;

  ContinuousState<double>* continuous_state_{nullptr};
  DiscreteValues<double>* discrete_state_{nullptr};
  std::unique_ptr<BasicVector<double>> input_vector_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
  std::unique_ptr<DiscreteValues<double>> updates_;

  LinearTimeVaryingData data_;
  double time_period_{0.};  // Defaults to continuous-time.
};

TEST_P(TimeVaryingLinearSystemTest, Constructor) {
  EXPECT_EQ(1, context_->get_num_input_ports());
  EXPECT_EQ(dut_->A(0.), data_.A.value(0.));
  EXPECT_EQ(dut_->B(0.), data_.B.value(0.));
  EXPECT_EQ(dut_->C(0.), data_.C.value(0.));
  EXPECT_EQ(dut_->D(0.), data_.D.value(0.));
  EXPECT_EQ(dut_->f0(0.), Eigen::Vector2d::Zero());
  EXPECT_EQ(dut_->y0(0.), Eigen::Vector2d::Zero());
  EXPECT_EQ(dut_->time_period(), time_period_);
  EXPECT_EQ(1, dut_->get_num_output_ports());
  EXPECT_EQ(1, dut_->get_num_input_ports());
}

// Tests that the derivatives are correctly computed.
TEST_P(TimeVaryingLinearSystemTest, Derivatives) {
  Eigen::Vector2d u(7, 42);
  input_vector_->get_mutable_value() << u;
  context_->FixInputPort(0, std::move(input_vector_));

  EXPECT_NE(derivatives_, nullptr);
  EXPECT_NE(updates_, nullptr);

  Eigen::Vector2d x(0.101, 3.7);

  if (time_period_ == 0.) {
    continuous_state_->SetFromVector(x);
  } else {
    discrete_state_->get_mutable_vector(0)->SetFromVector(x);
  }

  std::vector<double> times{0., 1e-5, 0.25, 0.5, 0.75, 1.};
  for (const double t : times) {
    context_->set_time(t);
    if (time_period_ == 0.) {
      dut_->CalcTimeDerivatives(*context_, derivatives_.get());
      const Eigen::VectorXd expected_derivatives =
          data_.A.value(t) * x + data_.B.value(t) * u;
      EXPECT_EQ(expected_derivatives,
                derivatives_->get_vector().CopyToVector());
    } else {
      dut_->CalcDiscreteVariableUpdates(*context_, updates_.get());
      const Eigen::VectorXd expected_updates =
          data_.A.value(t) * x + data_.B.value(t) * u;
      EXPECT_EQ(expected_updates, updates_->get_vector(0)->CopyToVector());
    }
  }
}

// Tests that the outputs are correctly computed.
TEST_P(TimeVaryingLinearSystemTest, Output) {
  // Sets the context's input port.
  Eigen::Vector2d u(5.6, -10.1);
  input_vector_->get_mutable_value() << u;
  context_->FixInputPort(0, std::move(input_vector_));

  // Sets the state.
  Eigen::Vector2d x(0.8, -22.1);
  if (time_period_ == 0.) {
    continuous_state_->SetFromVector(x);
  } else {
    discrete_state_->get_mutable_vector(0)->SetFromVector(x);
  }

  std::vector<double> times{0., 1e-5, 0.25, 0.5, 0.75, 1.};
  Eigen::VectorXd expected_output(2);
  for (const double t : times) {
    context_->set_time(t);
    dut_->CalcOutput(*context_, system_output_.get());
    expected_output = data_.C.value(t) * x + data_.D.value(t) * u;
    EXPECT_EQ(expected_output, system_output_->get_vector_data(0)->get_value());
  }
}

// Tests that conversion to different scalar types is possible.
TEST_P(TimeVaryingLinearSystemTest, ScalarTypeConversion) {
  EXPECT_TRUE(is_autodiffxd_convertible(*dut_, [&](const auto& converted) {
    EXPECT_EQ(converted.A(0.), data_.A.value(0.));
    EXPECT_EQ(converted.B(0.), data_.B.value(0.));
    EXPECT_EQ(converted.C(0.), data_.C.value(0.));
    EXPECT_EQ(converted.D(0.), data_.D.value(0.));
    EXPECT_EQ(converted.time_period(), time_period_);
  }));
  EXPECT_FALSE(is_symbolic_convertible(*dut_));
}

INSTANTIATE_TEST_CASE_P(Constructor, TimeVaryingLinearSystemTest,
                        testing::Values(ConstructorType::FromDataContinuous,
                                        ConstructorType::FromStructContinuous,
                                        ConstructorType::FromStructDiscrete));

}  // namespace
}  // namespace systems
}  // namespace drake
