#include "drake/systems/primitives/linear_transform_density.h"

#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace systems {
const double kEps = std::numeric_limits<double>::epsilon();

template <typename T>
void TestConstructor() {
  LinearTransformDensity<T> dut(RandomDistribution::kUniform,
                                2 /* input_size */, 3 /* output_size */);
  EXPECT_EQ(dut.num_input_ports(), 3);
  EXPECT_EQ(dut.num_output_ports(), 2);
  EXPECT_EQ(dut.get_distribution(), RandomDistribution::kUniform);

  EXPECT_EQ(dut.get_input_port_w_in().size(), 2);
  EXPECT_EQ(dut.get_input_port_w_in().get_random_type(),
            RandomDistribution::kUniform);
  EXPECT_EQ(dut.get_input_port_A().size(), 6);
  EXPECT_EQ(dut.get_input_port_b().size(), 3);
  EXPECT_EQ(dut.get_output_port_w_out().size(), 3);
  EXPECT_EQ(dut.get_output_port_w_out_density().size(), 1);
}

GTEST_TEST(LinearTransformDensityTest, Constructor) {
  TestConstructor<double>();
  TestConstructor<AutoDiffXd>();
}

GTEST_TEST(LinearTransformDensityTest, ToAutoDiff) {
  auto dut = std::make_unique<LinearTransformDensity<double>>(
      RandomDistribution::kUniform, 2, 3);
  EXPECT_TRUE(is_autodiffxd_convertible(*dut, [&](const auto& converted) {
    EXPECT_EQ(converted.get_input_port_w_in().size(),
              dut->get_input_port_w_in().size());
    EXPECT_EQ(converted.get_input_port_A().size(),
              dut->get_input_port_A().size());
    EXPECT_EQ(converted.get_input_port_b().size(),
              dut->get_input_port_b().size());
    EXPECT_EQ(converted.get_output_port_w_out().size(),
              dut->get_output_port_w_out().size());
    EXPECT_EQ(converted.get_output_port_w_out_density().size(),
              dut->get_output_port_w_out_density().size());
  }));
}

template <typename T>
void TestCalcOutput() {
  LinearTransformDensity<T> dut(RandomDistribution::kUniform, 2, 3);
  auto context = dut.CreateDefaultContext();

  Eigen::Matrix<T, 3, 2> A;
  // clang-format off
  A << 1., 2.,
       3., 4.,
       5., 6.;
  // clang-format on
  const FixedInputPortValue& port_A_val = dut.FixConstantA(context.get(), A);
  EXPECT_EQ(port_A_val.template get_vector_value<T>().size(), 6);
  // Confirm that A is stored in column-major order.
  const auto& port_A_vector =
      port_A_val.template get_vector_value<T>().get_value();
  EXPECT_EQ(port_A_vector(0), 1.);
  EXPECT_EQ(port_A_vector(1), 3.);
  EXPECT_EQ(port_A_vector(2), 5.);
  EXPECT_EQ(port_A_vector(3), 2.);
  EXPECT_EQ(port_A_vector(4), 4.);
  EXPECT_EQ(port_A_vector(5), 6.);
  Vector2<T> w_in(2, 3);
  dut.get_input_port_w_in().FixValue(context.get(), w_in);

  // Test when port b is not connected.
  const auto w_out_no_b = dut.get_output_port_w_out().Eval(*context);
  const Vector3<T> w_out_no_b_expected = A * w_in;
  EXPECT_TRUE(CompareMatrices(w_out_no_b, w_out_no_b_expected));

  // Test with port b connected.
  Vector3<T> b(-1, -2, -3);
  const auto& port_b_val = dut.FixConstantB(context.get(), b);
  EXPECT_EQ(port_b_val.template get_vector_value<T>().size(), 3);

  const auto w_out = dut.get_output_port_w_out().Eval(*context);
  const Vector3<T> w_out_expected = A * w_in + b;
  EXPECT_TRUE(CompareMatrices(w_out, w_out_expected, 10 * kEps));
}

GTEST_TEST(LinearTransformDensityTest, CalcOutput) {
  TestCalcOutput<double>();
  TestCalcOutput<AutoDiffXd>();
}

template <typename T>
void TestCalcOutputDensity() {
  LinearTransformDensity<T> dut(RandomDistribution::kUniform, 2, 2);
  auto context = dut.CreateDefaultContext();

  Eigen::Matrix<T, 2, 2> A;
  // clang-format off
  A << 1., 2.,
       3., 4.;
  // clang-format on
  dut.FixConstantA(context.get(), A);
  Vector2<T> w_in(2, 3);
  dut.get_input_port_w_in().FixValue(context.get(), w_in);

  // Test when port b is not connected.
  const auto w_out_density_no_b =
      dut.get_output_port_w_out_density().Eval(*context);
  const T w_out_density_no_b_expected = dut.CalcDensity(*context);
  EXPECT_TRUE(CompareMatrices(w_out_density_no_b,
                              Vector1<T>(w_out_density_no_b_expected)));

  // Test with port b connected.
  Vector2<T> b(-1, -2);
  dut.FixConstantB(context.get(), b);

  const auto w_out_density = dut.get_output_port_w_out_density().Eval(*context);
  const T w_out_density_expected = dut.CalcDensity(*context);
  EXPECT_TRUE(
      CompareMatrices(w_out_density, Vector1<T>(w_out_density_expected)));
}

GTEST_TEST(LinearTransformDensityTest, TestCalcOutputDensity) {
  TestCalcOutputDensity<double>();
  TestCalcOutputDensity<AutoDiffXd>();
}

template <typename T>
void CheckDensity(RandomDistribution distribution,
                  const std::vector<Eigen::Vector2d>& nonzero_prob_inputs,
                  const std::vector<Eigen::Vector2d>& zero_prob_inputs) {
  LinearTransformDensity<T> dut(distribution, 2, 2);
  auto context = dut.CreateDefaultContext();

  for (const auto& w_in_double : nonzero_prob_inputs) {
    const Vector2<T> w_in = w_in_double.cast<T>();
    dut.get_input_port_w_in().FixValue(context.get(), w_in);
    dut.FixConstantA(context.get(), Eigen::Matrix2d::Identity().cast<T>());
    T density = dut.CalcDensity(*context);
    // With A = I, We know the density of the output is just the product of the
    // input density.
    T density_expected = CalcProbabilityDensity<T>(distribution, w_in);
    EXPECT_NEAR(ExtractDoubleOrThrow(density),
                ExtractDoubleOrThrow(density_expected), 10 * kEps);
    // With A = diag(2, -3), we know the density of the output is the density of
    // the input divided by 6.
    Matrix2<T> A = Vector2<T>(2, -3).asDiagonal();
    dut.FixConstantA(context.get(), A);
    density = dut.CalcDensity(*context);
    density_expected = CalcProbabilityDensity<T>(distribution, w_in) / 6;
    EXPECT_NEAR(ExtractDoubleOrThrow(density),
                ExtractDoubleOrThrow(density_expected), 10 * kEps);

    // With A = [1, 2, 3, 4], the density of the output is the density of the
    // input divided by 2.
    A << T(1), T(2), T(3), T(4);
    dut.FixConstantA(context.get(), A);
    density = dut.CalcDensity(*context);
    density_expected = CalcProbabilityDensity<T>(distribution, w_in) / 2;
    EXPECT_NEAR(ExtractDoubleOrThrow(density),
                ExtractDoubleOrThrow(density_expected), 10 * kEps);

    // Set b to an arbitrary value, the density is unchanged.
    dut.FixConstantB(context.get(), Vector2<T>(10, 20));
    density = dut.CalcDensity(*context);
    EXPECT_NEAR(ExtractDoubleOrThrow(density),
                ExtractDoubleOrThrow(density_expected), 10 * kEps);
  }

  for (const auto& w_in_double : zero_prob_inputs) {
    dut.get_input_port_w_in().FixValue(context.get(), w_in_double.cast<T>());
    T density = dut.CalcDensity(*context);
    EXPECT_EQ(density, 0.);
  }
}

template <typename T>
void CheckGaussianDensity() {
  CheckDensity<T>(RandomDistribution::kGaussian,
                  {Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 5),
                   Eigen::Vector2d(-0.5, 1.8)},
                  {});
  // Test CalcDensity with a Gaussian distribution
  LinearTransformDensity<T> dut(RandomDistribution::kGaussian, 2, 2);
  auto context = dut.CreateDefaultContext();
  dut.get_input_port_w_in().FixValue(context.get(), Vector2<T>(0.1, 0.5));
  // Now test non-diagonal A.
  Matrix2<T> A;
  A << 1, 2, 3, 4;
  // We know that w_out = A * w_in + b is also a Gaussian distribution, with
  // mean b and variance AAᵀ. We can compute the density as
  // exp(-0.5 (w_out - b)ᵀ(AAᵀ)⁻¹(w_out-b)) / (sqrt((2π)ᵏ*det(AAᵀ))
  // where k is the dimension of w_out.
  dut.FixConstantA(context.get(), A);
  Vector2<T> b(2, 3);
  dut.FixConstantB(context.get(), b);
  T density = dut.CalcDensity(*context);
  auto w_out = dut.get_output_port_w_out().Eval(*context);
  Eigen::LDLT<Matrix2<T>> ldlt_solver;
  Matrix2<T> cov = A * A.transpose();
  ldlt_solver.compute(cov);
  double density_expected =
      std::exp(ExtractDoubleOrThrow(
          -0.5 * (w_out - b).dot(ldlt_solver.solve(w_out - b)))) /
      std::sqrt(std::pow(2 * M_PI, 2) *
                ExtractDoubleOrThrow(cov.determinant()));
  EXPECT_NEAR(ExtractDoubleOrThrow(density), density_expected, 10 * kEps);
}

GTEST_TEST(LinearTransformDensityTest, CalcDensityGaussian) {
  CheckGaussianDensity<double>();
  CheckGaussianDensity<AutoDiffXd>();
}

GTEST_TEST(LinearTransformDensityTest, CalcDensityUniform) {
  const std::vector<Eigen::Vector2d> nonzero_prob_inputs{
      {Eigen::Vector2d(0.5, 0.2), Eigen::Vector2d(10 * kEps, 0.9),
       Eigen::Vector2d(0.3, 1. - 10 * kEps)}};
  const std::vector<Eigen::Vector2d> zero_prob_inputs{
      {Eigen::Vector2d(-0.1, 0.5), Eigen::Vector2d(0.2, 1.4),
       Eigen::Vector2d(-0.5, 1.5)}};
  CheckDensity<double>(RandomDistribution::kUniform, nonzero_prob_inputs,
                       zero_prob_inputs);
  CheckDensity<AutoDiffXd>(RandomDistribution::kUniform, nonzero_prob_inputs,
                           zero_prob_inputs);
}

GTEST_TEST(LinearTransformDensityTest, CalcDensityExponential) {
  const std::vector<Eigen::Vector2d> nonzero_prob_inputs{
      {Eigen::Vector2d(0.5, 1.2), Eigen::Vector2d(10 * kEps, 0.9),
       Eigen::Vector2d(0.3, 1.6)}};
  const std::vector<Eigen::Vector2d> zero_prob_inputs{
      {Eigen::Vector2d(-0.1, 0.5), Eigen::Vector2d(0.5, -1.4),
       Eigen::Vector2d(-0.5, 1.5)}};
  CheckDensity<double>(RandomDistribution::kExponential, nonzero_prob_inputs,
                       zero_prob_inputs);
  CheckDensity<AutoDiffXd>(RandomDistribution::kExponential,
                           nonzero_prob_inputs, zero_prob_inputs);
}

GTEST_TEST(LinearTransformDensityTest, NoninvertibleTransform) {
  // Test with A being non-invertible.
  for (RandomDistribution distribution :
       {RandomDistribution::kUniform, RandomDistribution::kGaussian,
        RandomDistribution::kExponential}) {
    // Test with input_size != output_size.
    LinearTransformDensity<double> dut1(distribution, 2, 3);
    auto context1 = dut1.CreateDefaultContext();
    Eigen::Matrix<double, 3, 2> A1;
    A1 << 1, 2, 3, 4, 5, 6;
    dut1.FixConstantA(context1.get(), A1);
    dut1.get_input_port_w_in().FixValue(context1.get(),
                                        Eigen::Vector2d(0.5, 0.6));
    DRAKE_EXPECT_THROWS_MESSAGE(dut1.CalcDensity(*context1), std::runtime_error,
                                ".* to compute the density.*");

    // Test with input_size == output_size but A being non-invertible.
    LinearTransformDensity<double> dut2(distribution, 2, 2);
    auto context2 = dut2.CreateDefaultContext();
    Eigen::Matrix2d A2;
    A2 << 1, 2, 2, 4;
    dut2.FixConstantA(context2.get(), A2);
    dut2.get_input_port_w_in().FixValue(context2.get(),
                                        Eigen::Vector2d(0.5, 0.6));
    DRAKE_EXPECT_THROWS_MESSAGE(dut2.CalcDensity(*context2), std::runtime_error,
                                ".* to compute the density.*");
  }
}

// Compute the gradient ∂pr(x)/∂x where x is the sample, pr(x) is the
// probability density function of x.
// TODO(hongkai.dai): this function is useful, move it out of test.
Eigen::VectorXd CalcRandomSourceDensityGradient(
    RandomDistribution distribution,
    const Eigen::Ref<const Eigen::VectorXd>& sample) {
  switch (distribution) {
    case RandomDistribution::kUniform: {
      return Eigen::VectorXd::Zero(sample.rows());
    }
    case RandomDistribution::kGaussian: {
      // The density of a single Gaussian variable is
      // p = exp(-0.5x²)/sqrt(2π)
      // The gradient dp/dx is -x*p
      const double p = CalcProbabilityDensity<double>(distribution, sample);
      return -sample * p;
    }
    case RandomDistribution::kExponential: {
      // The density of a single Exponential variable is
      // p = exp(-x) if x>=0
      // p = 0 if x < 0
      // The gradient is
      // dpdx = -p
      const double p = CalcProbabilityDensity<double>(distribution, sample);
      return Eigen::VectorXd::Constant(sample.rows(), -p);
    }
  }
  DRAKE_UNREACHABLE();
}

void CheckDensityGradient(RandomDistribution distribution,
                          const Eigen::Vector2d w_in_val) {
  // Test CalcDensity with AutoDiffXd. Make sure that we can compute the
  // gradient of the density correctly.
  LinearTransformDensity<AutoDiffXd> dut(distribution, 2, 2);
  auto context = dut.CreateDefaultContext();
  // First compute the gradient of the density p w.r.t b.
  // since p = pr(A⁻¹(w_out - b)) / |det(A)|
  // ∂p/∂b = 1/|det(A)| * ∂pr/∂w_in * (-A⁻¹)
  auto b = math::InitializeAutoDiff(Eigen::Vector2d(2, 4));
  Matrix2<AutoDiffXd> A =
      Eigen::Vector2d(2, -3).cast<AutoDiffXd>().asDiagonal();
  Eigen::Matrix2d A_val = math::ExtractValue(A);
  dut.FixConstantA(context.get(), A);
  dut.FixConstantB(context.get(), b);
  Vector2<AutoDiffXd> w_in = w_in_val.cast<AutoDiffXd>();
  dut.get_input_port_w_in().FixValue(context.get(), w_in);
  auto density = dut.CalcDensity(*context);
  Eigen::Vector2d ddensity_db_expected =
      (1 / std::abs(A_val.determinant()) *
       CalcRandomSourceDensityGradient(distribution, w_in_val).transpose() *
       -A_val.inverse())
          .transpose();
  Eigen::VectorXd density_grad = density.derivatives().rows() == 0
                                     ? Eigen::Vector2d::Zero()
                                     : density.derivatives();
  EXPECT_TRUE(CompareMatrices(density_grad, ddensity_db_expected, 10 * kEps));

  // Compute the gradient of the density w.r.t A.
  // Since p = pr(A⁻¹(w_out-b)) / |det(A)|, we know
  // ∂p/∂A(i, j) = ∂pr(w_in)/∂w_in*(-A⁻¹*∂A/∂A(i,j) * A⁻¹) * (w_out-b) /
  // |det(A)| - pr(w_in)*|det(A)|⁻²∂|det(A)|/∂A(i,j)
  A(0, 0).value() = 2;
  A(1, 0).value() = 1;
  A(0, 1).value() = 3;
  A(1, 1).value() = -4;
  A(0, 0).derivatives() = Eigen::Vector4d(1, 0, 0, 0);
  A(1, 0).derivatives() = Eigen::Vector4d(0, 1, 0, 0);
  A(0, 1).derivatives() = Eigen::Vector4d(0, 0, 1, 0);
  A(1, 1).derivatives() = Eigen::Vector4d(0, 0, 0, 1);
  dut.FixConstantA(context.get(), A);
  A_val = math::ExtractValue(A);
  b = Eigen::Vector2d(2, 4).cast<AutoDiffXd>();
  dut.FixConstantB(context.get(), b);
  density = dut.CalcDensity(*context);
  Eigen::Matrix2d A_inv = A_val.inverse();
  // The gradient ∂|det(A)|/∂A(i,j)
  // |det(A)| = -A(0, 0) * A(1, 1) + A(0, 1) * A(1, 0)
  std::array<std::array<double, 2>, 2> dA_abs_det_dAij;
  std::array<std::array<Eigen::Matrix2d, 2>, 2> dA_dAij;
  dA_abs_det_dAij[0][0] = -A(1, 1).value();
  dA_abs_det_dAij[0][1] = A(1, 0).value();
  dA_abs_det_dAij[1][0] = A(0, 1).value();
  dA_abs_det_dAij[1][1] = -A(0, 0).value();
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      dA_dAij[i][j] = Eigen::Matrix2d::Zero();
      dA_dAij[i][j](i, j) = 1.;
    }
  }
  Eigen::Vector2d w_out_val = math::ExtractValue(
      dut.get_output_port_w_out().Eval(*context));
  Eigen::Vector2d b_val = math::ExtractValue(b);
  const double A_det_abs = std::abs(A_val.determinant());
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      const double dp_dAij =
          CalcRandomSourceDensityGradient(distribution, w_in_val)
                  .dot(-A_inv * dA_dAij[i][j] * A_inv * (w_out_val - b_val)) /
              A_det_abs -
          CalcProbabilityDensity<double>(distribution, w_in_val) *
              std::pow(A_det_abs, -2) * dA_abs_det_dAij[i][j];
      const double density_derivative_actual =
        density.derivatives().rows() == 0 ? 0.0 :
        density.derivatives()(i + 2 * j);
      EXPECT_NEAR(density_derivative_actual, dp_dAij, 10 * kEps);
    }
  }
}

GTEST_TEST(LinearTransformDensityTest, CalcDensityGradient) {
  // Uniform distribution
  // input has zero probability.
  CheckDensityGradient(RandomDistribution::kUniform, Eigen::Vector2d(-1, -2));
  // input has zero probability.
  CheckDensityGradient(RandomDistribution::kUniform, Eigen::Vector2d(0.2, 1.5));
  // input has non-zero probability
  CheckDensityGradient(RandomDistribution::kUniform, Eigen::Vector2d(0.1, 0.2));

  // Gaussian distribution
  CheckDensityGradient(RandomDistribution::kGaussian,
                       Eigen::Vector2d(0.2, 0.5));
  CheckDensityGradient(RandomDistribution::kGaussian,
                       Eigen::Vector2d(-0.2, 0.5));

  // Exponential distribution
  // w_in has zero probability.
  CheckDensityGradient(RandomDistribution::kExponential,
                       Eigen::Vector2d(-0.2, 0.5));
  // w_in has a non-zero probability.
  CheckDensityGradient(RandomDistribution::kExponential,
                       Eigen::Vector2d(0.2, 0.5));
}

}  // namespace systems
}  // namespace drake
