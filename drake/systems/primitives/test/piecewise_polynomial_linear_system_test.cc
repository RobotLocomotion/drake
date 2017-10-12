#include "drake/systems/primitives/piecewise_polynomial_linear_system.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

using std::make_unique;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace {

static constexpr double kDiscreteTimeStep = 0.1;

// A helper for accessing the underlying PiecewisePolynomialTrajectory data.
struct MatrixData {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MatrixData)

  MatrixData() {}
  /// Fully-parameterized constructor.
  MatrixData(const std::vector<double>& times_in,
             const std::vector<Eigen::MatrixXd>& Avec_in,
             const std::vector<Eigen::MatrixXd>& Bvec_in,
             const std::vector<Eigen::MatrixXd>& Cvec_in,
             const std::vector<Eigen::MatrixXd>& Dvec_in)
      : times(times_in),
        Avec(Avec_in),
        Bvec(Bvec_in),
        Cvec(Cvec_in),
        Dvec(Dvec_in) {}
  std::vector<double> times{};
  std::vector<Eigen::MatrixXd> Avec;
  std::vector<Eigen::MatrixXd> Bvec;
  std::vector<Eigen::MatrixXd> Cvec;
  std::vector<Eigen::MatrixXd> Dvec;
};

// Define a simple LinearTimeVaryingData struct via member assignment.
std::pair<LinearTimeVaryingData, MatrixData> ExampleLinearTimeVaryingData() {
  const std::vector<double> times{0., kDiscreteTimeStep, 2 * kDiscreteTimeStep};
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

  LinearTimeVaryingData result;
  result.A = PiecewisePolynomialTrajectory(Apoly);
  result.B = PiecewisePolynomialTrajectory(Bpoly);
  result.C = PiecewisePolynomialTrajectory(Cpoly);
  result.D = PiecewisePolynomialTrajectory(Dpoly);

  return std::make_pair(result, MatrixData{times, Avec, Bvec, Cvec, Dvec});
}

GTEST_TEST(LinearTimeVaryingData, PiecewisePolynomialConstructor) {
  LinearTimeVaryingData ppt_data;
  MatrixData mat_data;
  std::tie(ppt_data, mat_data) = ExampleLinearTimeVaryingData();

  // Construct a LinearTimeVaryingData struct from PiecewisePolynomials.
  const LinearTimeVaryingData dut_from_pp =
      LinearTimeVaryingData(PiecewisePolynomial<double>::FirstOrderHold(
                                mat_data.times, mat_data.Avec),
                            PiecewisePolynomial<double>::FirstOrderHold(
                                mat_data.times, mat_data.Bvec),
                            PiecewisePolynomial<double>::FirstOrderHold(
                                mat_data.times, mat_data.Cvec),
                            PiecewisePolynomial<double>::FirstOrderHold(
                                mat_data.times, mat_data.Dvec));
  for (const double t :
       ppt_data.A.get_piecewise_polynomial().getSegmentTimes()) {
    EXPECT_TRUE(CompareMatrices(dut_from_pp.A.value(t), ppt_data.A.value(t)));
    EXPECT_TRUE(CompareMatrices(dut_from_pp.B.value(t), ppt_data.B.value(t)));
    EXPECT_TRUE(CompareMatrices(dut_from_pp.C.value(t), ppt_data.C.value(t)));
    EXPECT_TRUE(CompareMatrices(dut_from_pp.D.value(t), ppt_data.D.value(t)));
  }
}

GTEST_TEST(LinearTimeVaryingData, PiecewisePolynomialTrajectoryConstructor) {
  LinearTimeVaryingData data;
  std::tie(data, std::ignore) = ExampleLinearTimeVaryingData();

  // Construct a LinearTimeVaryingData struct from
  // PiecewisePolynomialTrajectories.
  const LinearTimeVaryingData dut_from_ppt =
      LinearTimeVaryingData(data.A, data.B, data.C, data.D);
  for (const double t : data.A.get_piecewise_polynomial().getSegmentTimes()) {
    EXPECT_TRUE(CompareMatrices(dut_from_ppt.A.value(t), data.A.value(t)));
    EXPECT_TRUE(CompareMatrices(dut_from_ppt.B.value(t), data.B.value(t)));
    EXPECT_TRUE(CompareMatrices(dut_from_ppt.C.value(t), data.C.value(t)));
    EXPECT_TRUE(CompareMatrices(dut_from_ppt.D.value(t), data.D.value(t)));
  }
}

enum ConstructorType {
  FromDataContinuous,
  FromStructContinuous,
  FromStructDiscrete
};

class PiecewisePolynomialLinearSystemTest
    : public ::testing::TestWithParam<double> {
 public:
  void SetUp() override {
    std::tie(ltv_data_, mat_data_) = ExampleLinearTimeVaryingData();

    // Set up an arbitrary PiecewisePolynomialLinearSystem.
    if (this->GetParam() == ConstructorType::FromDataContinuous) {
      dut_ =
          std::make_unique<PiecewisePolynomialLinearSystem<double>>(ltv_data_);
    } else if (this->GetParam() == ConstructorType::FromStructContinuous) {
      dut_ = std::make_unique<PiecewisePolynomialLinearSystem<double>>(
          ltv_data_.A, ltv_data_.B, ltv_data_.C, ltv_data_.D);
    } else if (this->GetParam() == ConstructorType::FromStructDiscrete) {
      time_period_ = kDiscreteTimeStep;
      dut_ = std::make_unique<PiecewisePolynomialLinearSystem<double>>(
          mat_data_.Avec, mat_data_.Bvec, mat_data_.Cvec, mat_data_.Dvec,
          time_period_);
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
  // The Device Under Test (DUT) is a PiecewisePolynomialLinearSystem<double>.
  std::unique_ptr<PiecewisePolynomialLinearSystem<double>> dut_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<SystemOutput<double>> system_output_;

  ContinuousState<double>* continuous_state_{nullptr};
  DiscreteValues<double>* discrete_state_{nullptr};
  std::unique_ptr<BasicVector<double>> input_vector_;
  std::unique_ptr<ContinuousState<double>> derivatives_;
  std::unique_ptr<DiscreteValues<double>> updates_;

  LinearTimeVaryingData ltv_data_;
  MatrixData mat_data_;
  double time_period_{0.};  // Defaults to continuous-time.
};

TEST_P(PiecewisePolynomialLinearSystemTest, Constructor) {
  EXPECT_EQ(1, context_->get_num_input_ports());
  EXPECT_EQ(dut_->A(0.), ltv_data_.A.value(0.));
  EXPECT_EQ(dut_->B(0.), ltv_data_.B.value(0.));
  EXPECT_EQ(dut_->C(0.), ltv_data_.C.value(0.));
  EXPECT_EQ(dut_->D(0.), ltv_data_.D.value(0.));
  EXPECT_EQ(dut_->time_period(), time_period_);
  EXPECT_EQ(1, dut_->get_num_output_ports());
  EXPECT_EQ(1, dut_->get_num_input_ports());
}

TEST_P(PiecewisePolynomialLinearSystemTest, KnotPointConsistency) {
  for (int i{0}; i < static_cast<int>(mat_data_.times.size()); ++i) {
    EXPECT_TRUE(
        CompareMatrices(dut_->A(mat_data_.times[i]), mat_data_.Avec[i]));
    EXPECT_TRUE(
        CompareMatrices(dut_->B(mat_data_.times[i]), mat_data_.Bvec[i]));
    EXPECT_TRUE(
        CompareMatrices(dut_->C(mat_data_.times[i]), mat_data_.Cvec[i]));
    EXPECT_TRUE(
        CompareMatrices(dut_->D(mat_data_.times[i]), mat_data_.Dvec[i]));
  }
}

// Tests that the derivatives are correctly computed.
TEST_P(PiecewisePolynomialLinearSystemTest, Derivatives) {
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

  std::vector<double> times{0., 1e-5 * kDiscreteTimeStep,
                            0.25 * kDiscreteTimeStep, kDiscreteTimeStep,
                            1.5 * kDiscreteTimeStep};
  for (const double t : times) {
    context_->set_time(t);
    const Eigen::Matrix2d A = ltv_data_.A.value(t);
    const Eigen::Matrix2d B = ltv_data_.B.value(t);
    if (time_period_ == 0.) {
      dut_->CalcTimeDerivatives(*context_, derivatives_.get());
      EXPECT_EQ(A * x + B * u, derivatives_->get_vector().CopyToVector());
    } else {
      dut_->CalcDiscreteVariableUpdates(*context_, updates_.get());
      EXPECT_EQ(A * x + B * u, updates_->get_vector(0)->CopyToVector());
    }
  }
}

// Tests that the outputs are correctly computed.
TEST_P(PiecewisePolynomialLinearSystemTest, Output) {
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

  std::vector<double> times{0., 1e-5 * kDiscreteTimeStep,
                            0.25 * kDiscreteTimeStep, kDiscreteTimeStep,
                            1.5 * kDiscreteTimeStep};
  Eigen::VectorXd expected_output(2);
  for (const double t : times) {
    context_->set_time(t);
    dut_->CalcOutput(*context_, system_output_.get());
    const Eigen::Matrix2d C = ltv_data_.C.value(t);
    const Eigen::Matrix2d D = ltv_data_.D.value(t);
    EXPECT_EQ(C * x + D * u, system_output_->get_vector_data(0)->get_value());
  }
}

// Tests that conversion to different scalar types is possible.
TEST_P(PiecewisePolynomialLinearSystemTest, ScalarTypeConversion) {
  EXPECT_TRUE(is_autodiffxd_convertible(*dut_, [&](const auto& converted) {
    EXPECT_EQ(converted.A(0.), ltv_data_.A.value(0.));
    EXPECT_EQ(converted.B(0.), ltv_data_.B.value(0.));
    EXPECT_EQ(converted.C(0.), ltv_data_.C.value(0.));
    EXPECT_EQ(converted.D(0.), ltv_data_.D.value(0.));
    EXPECT_EQ(converted.time_period(), time_period_);
  }));
  EXPECT_FALSE(is_symbolic_convertible(*dut_));
}

INSTANTIATE_TEST_CASE_P(Constructor, PiecewisePolynomialLinearSystemTest,
                        testing::Values(ConstructorType::FromDataContinuous,
                                        ConstructorType::FromStructContinuous,
                                        ConstructorType::FromStructDiscrete));

}  // namespace
}  // namespace systems
}  // namespace drake
