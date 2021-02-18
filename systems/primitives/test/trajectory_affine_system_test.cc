#include "drake/systems/primitives/trajectory_affine_system.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"
#include "drake/systems/primitives/test/trajectory_linear_affine_test.h"

using std::make_unique;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace {

using test::TestSystemMatrixTrajectories;

enum class ConstructorType { FromContinuous, FromDiscrete };

class TrajectoryAffineSystemTest
    : public ::testing::TestWithParam<ConstructorType> {
 public:
  void SetUp() override {
    if (this->GetParam() == ConstructorType::FromContinuous) {
      dut_ = make_unique<TrajectoryAffineSystem<double>>(data_.A, data_.B,
          data_.f0, data_.C, data_.D, data_.y0);
    } else if (this->GetParam() == ConstructorType::FromDiscrete) {
      time_period_ = data_.kDiscreteTimeStep;
      dut_ = make_unique<TrajectoryAffineSystem<double>>(
          data_.A, data_.B, data_.f0, data_.C, data_.D, data_.y0,
          data_.kDiscreteTimeStep);
    }
    context_ = dut_->CreateDefaultContext();
    continuous_state_ = &context_->get_mutable_continuous_state();
    discrete_state_ = &context_->get_mutable_discrete_state();
    derivatives_ = dut_->AllocateTimeDerivatives();
    updates_ = dut_->AllocateDiscreteVariables();
  }

 protected:
  // The Device Under Test (DUT) is a TrajectoryAffineSystem<double>.
  unique_ptr<TrajectoryAffineSystem<double>> dut_;
  unique_ptr<Context<double>> context_;

  ContinuousState<double>* continuous_state_{nullptr};
  DiscreteValues<double>* discrete_state_{nullptr};
  unique_ptr<ContinuousState<double>> derivatives_;
  unique_ptr<DiscreteValues<double>> updates_;

  TestSystemMatrixTrajectories data_;
  double time_period_{0.};  // Defaults to continuous-time.
};

TEST_P(TrajectoryAffineSystemTest, Constructor) {
  EXPECT_EQ(1, context_->num_input_ports());
  EXPECT_EQ(dut_->A(0.), data_.A.value(0.));
  EXPECT_EQ(dut_->B(0.), data_.B.value(0.));
  EXPECT_EQ(dut_->C(0.), data_.C.value(0.));
  EXPECT_EQ(dut_->D(0.), data_.D.value(0.));
  EXPECT_EQ(dut_->f0(0.), data_.f0.value(0.));
  EXPECT_EQ(dut_->y0(0.), data_.y0.value(0.));
  EXPECT_EQ(dut_->time_period(), time_period_);
  EXPECT_EQ(1, dut_->num_output_ports());
  EXPECT_EQ(1, dut_->num_input_ports());
}

TEST_P(TrajectoryAffineSystemTest, KnotPointConsistency) {
  for (int i{0}; i < static_cast<int>(data_.times.size()); ++i) {
    EXPECT_TRUE(
        CompareMatrices(dut_->A(data_.times[i]), data_.Avec[i]));
    EXPECT_TRUE(
        CompareMatrices(dut_->B(data_.times[i]), data_.Bvec[i]));
    EXPECT_TRUE(
        CompareMatrices(dut_->C(data_.times[i]), data_.Cvec[i]));
    EXPECT_TRUE(
        CompareMatrices(dut_->D(data_.times[i]), data_.Dvec[i]));
    EXPECT_TRUE(
        CompareMatrices(dut_->f0(data_.times[i]), data_.f0vec[i]));
    EXPECT_TRUE(
        CompareMatrices(dut_->y0(data_.times[i]), data_.y0vec[i]));
  }
}

// Tests that the derivatives and discrete updates are correctly computed.
TEST_P(TrajectoryAffineSystemTest, DiscreteUpdates) {
  // Sets the context's input port.
  Eigen::Vector2d u(7, 42);
  dut_->get_input_port().FixValue(context_.get(), u);

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
  for (const double t : data_.times) {
    context_->SetTime(t);
    const Eigen::Matrix2d A = data_.A.value(t);
    const Eigen::Matrix2d B = data_.B.value(t);
    const Eigen::Vector2d f0 = data_.f0.value(t);
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
TEST_P(TrajectoryAffineSystemTest, Output) {
  // Sets the context's input port.
  Eigen::Vector2d u(5.6, -10.1);
  dut_->get_input_port().FixValue(context_.get(), u);

  // Sets the state.
  Eigen::Vector2d x(0.8, -22.1);
  if (time_period_ == 0.) {
    continuous_state_->SetFromVector(x);
  } else {
    discrete_state_->get_mutable_vector(0).SetFromVector(x);
  }

  const double tol = 1e-10;
  for (const double t : data_.times) {
    context_->SetTime(t);

    const Eigen::Matrix2d C = data_.C.value(t);
    const Eigen::Matrix2d D = data_.D.value(t);
    const Eigen::Vector2d y0 = data_.y0.value(t);
    EXPECT_TRUE(CompareMatrices(C * x + D * u + y0,
                                dut_->get_output_port().Eval(*context_),
                                tol));
  }
}

// Tests that conversion to different scalar types is possible.
TEST_P(TrajectoryAffineSystemTest, ScalarTypeConversion) {
  EXPECT_TRUE(is_autodiffxd_convertible(*dut_, [&](const auto& converted) {
    EXPECT_EQ(converted.A(0.), data_.A.value(0.));
    EXPECT_EQ(converted.B(0.), data_.B.value(0.));
    EXPECT_EQ(converted.C(0.), data_.C.value(0.));
    EXPECT_EQ(converted.D(0.), data_.D.value(0.));
    EXPECT_EQ(converted.time_period(), time_period_);
  }));
  EXPECT_FALSE(is_symbolic_convertible(*dut_));
}

INSTANTIATE_TEST_SUITE_P(Constructor, TrajectoryAffineSystemTest,
                        testing::Values(ConstructorType::FromContinuous,
                                        ConstructorType::FromDiscrete));

}  // namespace
}  // namespace systems
}  // namespace drake
