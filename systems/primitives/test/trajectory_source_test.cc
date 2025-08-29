#include "drake/systems/primitives/trajectory_source.h"

#include <algorithm>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/trajectories/bezier_curve.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

using Eigen::Matrix;
using Eigen::MatrixXd;
using std::make_unique;

namespace drake {
namespace systems {

using symbolic::Expression;
using trajectories::PiecewisePolynomial;

namespace {

class TrajectorySourceTest : public ::testing::Test {
 protected:
  const size_t kDerivativeOrder = 5;

  void BuildSource(PiecewisePolynomial<double> pp) {
    kppTraj_ = make_unique<PiecewisePolynomial<double>>(pp);
    source_ = make_unique<TrajectorySource<double>>(*kppTraj_, kDerivativeOrder,
                                                    true);
    context_ = source_->CreateDefaultContext();
    input_ = make_unique<BasicVector<double>>(3 /* length */);
  }

  std::unique_ptr<PiecewisePolynomial<double>> kppTraj_;
  std::unique_ptr<TrajectorySource<double>> source_;
  std::unique_ptr<Context<double>> context_;
  std::unique_ptr<BasicVector<double>> input_;
};

TEST_F(TrajectorySourceTest, OutputTest) {
  const Polynomiald y = Polynomiald("y");
  const Polynomiald yyyy = (y * y * y * y);
  const std::vector<Polynomiald> p_vec{yyyy};
  BuildSource(PiecewisePolynomial<double>(p_vec, {0.0, 3}));

  ASSERT_EQ(0, context_->num_input_ports());

  const double kTestTime = 1.75;
  context_->SetTime(kTestTime);

  const auto& output = source_->get_output_port().Eval(*context_);

  int len = kppTraj_->rows();
  EXPECT_EQ(kppTraj_->value(kTestTime), output.segment(0, len));

  for (size_t i = 1; i <= kDerivativeOrder; ++i) {
    EXPECT_TRUE(CompareMatrices(kppTraj_->derivative(i).value(kTestTime),
                                output.segment(len * i, len), 1e-10,
                                MatrixCompareType::absolute));
  }

  // Test first derivative (which is in second segment):
  // f`(yyyy) = f`(y^4) = 4 * y^3.  y = kTestTime.
  EXPECT_NEAR(output.segment(len, len)(0), 21.4375, 1e-6);

  Eigen::MatrixXd samples(1, 2);
  samples << 0, 1;
  PiecewisePolynomial<double> pp = PiecewisePolynomial<double>::FirstOrderHold(
      Eigen::Vector2d(0, 2), samples);
  source_->UpdateTrajectory(pp);
  context_->SetTime(kTestTime);  // Set the time again to invalidate the cache.

  const auto& output2 = source_->get_output_port().Eval(*context_);
  EXPECT_EQ(pp.value(kTestTime), output2.segment(0, len));
  for (size_t i = 1; i <= kDerivativeOrder; ++i) {
    EXPECT_TRUE(CompareMatrices(pp.derivative(i).value(kTestTime),
                                output2.segment(len * i, len), 1e-10,
                                MatrixCompareType::absolute));
  }
}

// Tests that ConstantVectorSource allocates no state variables in the context_.
TEST_F(TrajectorySourceTest, ConstantVectorSourceIsStateless) {
  BuildSource(PiecewisePolynomial<double>(MatrixXd::Constant(2, 1, 1.5)));
  EXPECT_EQ(0, context_->num_continuous_states());
}

TEST_F(TrajectorySourceTest, Clone) {
  const int kRows = 2;
  BuildSource(PiecewisePolynomial<double>(MatrixXd::Constant(kRows, 1, 1.5)));
  std::unique_ptr<System<double>> copy;
  EXPECT_NO_THROW(copy = source_->Clone());
  ASSERT_TRUE(copy != nullptr);
  EXPECT_EQ(copy->get_output_port().size(), kRows * (1 + kDerivativeOrder));
  auto new_context = copy->CreateDefaultContext();
  EXPECT_EQ(copy->get_output_port().Eval(*new_context)[0], 1.5);
}

template <typename T>
void TestScalar(bool zero_derivatives_beyond_limits) {
  auto pp = PiecewisePolynomial<T>::ZeroOrderHold(Vector3<T>{0, 1, 2},
                                                  RowVector3<T>{1.2, 3, 1.5});
  TrajectorySource<T> source(pp, 1, zero_derivatives_beyond_limits);
  auto context = source.CreateDefaultContext();
  context->SetTime(0.5);
  EXPECT_NEAR(ExtractDoubleOrThrow(source.get_output_port().Eval(*context))[0],
              1.2, 1e-14);
}

GTEST_TEST(AdditionalTrajectorySourceTests, Scalars) {
  for (bool zero_derivatives_beyond_limits : {true, false}) {
    TestScalar<double>(zero_derivatives_beyond_limits);
    TestScalar<AutoDiffXd>(zero_derivatives_beyond_limits);
    TestScalar<symbolic::Expression>(zero_derivatives_beyond_limits);
  }
}

GTEST_TEST(AdditionalTrajectorySourceTests, ScalarConversion) {
  auto pp = PiecewisePolynomial<double>::ZeroOrderHold(
      Eigen::Vector3d{0, 1, 2}, Eigen::RowVector3d{1.2, 3, 1.5});
  for (bool zero_derivatives_beyond_limits : {true, false}) {
    // AutoDiffXd.
    TrajectorySource<double> source(pp, 1, zero_derivatives_beyond_limits);
    EXPECT_TRUE(is_autodiffxd_convertible(source));
    auto source_ad = System<double>::ToAutoDiffXd(source);
    auto context_ad = source_ad->CreateDefaultContext();
    context_ad->SetTime(0.5);
    EXPECT_NEAR(
        ExtractDoubleOrThrow(source_ad->get_output_port().Eval(*context_ad))[0],
        1.2, 1e-14);

    // Taking the gradient w.r.t to time throws.
    context_ad->SetTime(AutoDiffXd(0.5, Eigen::Vector2d(1, 2)));
    DRAKE_EXPECT_THROWS_MESSAGE(source_ad->get_output_port().Eval(*context_ad),
                                ".*UpdateTrajectory.*");

    // UpdateTrajectory can resolve it.
    auto pp_ad = PiecewisePolynomial<AutoDiffXd>::ZeroOrderHold(
        Vector3<AutoDiffXd>{0, 1, 2}, RowVector3<AutoDiffXd>{1.2, 3, 1.5});
    source_ad->UpdateTrajectory(pp_ad);
    EXPECT_NEAR(
        ExtractDoubleOrThrow(source_ad->get_output_port().Eval(*context_ad))[0],
        1.2, 1e-14);

    // Symbolic.
    EXPECT_TRUE(is_symbolic_convertible(source));
    auto source_sym = System<double>::ToSymbolic(source);
    auto context_sym = source_sym->CreateDefaultContext();
    context_sym->SetTime(0.5);
    EXPECT_NEAR(ExtractDoubleOrThrow(
                    source_sym->get_output_port().Eval(*context_sym))[0],
                1.2, 1e-14);

    // When time is symbolic, the output port eval throws.
    symbolic::Variable t("t");
    context_sym->SetTime(t);
    DRAKE_EXPECT_THROWS_MESSAGE(
        source_sym->get_output_port().Eval(*context_sym), ".*symbolic time.*");

    // UpdateTrajectory can resolve it. (We use BezierCurve here because
    // PiecewiseTrajectory doesn't currently support non-double Expression for
    // time).
    trajectories::BezierCurve<Expression> curve_sym(
        0.0, 2.0, RowVector2<Expression>{1.2, 3});
    source_sym->UpdateTrajectory(curve_sym);
    Expression expected = (1.2 * (1.0 - (min(2.0, max(0.0, t)) / 2.0)) +
                           3.0 * (min(2.0, max(0.0, t)) / 2.0));
    EXPECT_TRUE(
        source_sym->get_output_port().Eval(*context_sym)[0].EqualTo(expected));
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
