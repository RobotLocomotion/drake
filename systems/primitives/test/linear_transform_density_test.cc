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
  EXPECT_EQ(dut.num_output_ports(), 1);
  EXPECT_EQ(dut.get_distribution(), RandomDistribution::kUniform);

  EXPECT_EQ(dut.get_input_port_w_in().size(), 2);
  EXPECT_EQ(dut.get_input_port_A().size(), 6);
  EXPECT_EQ(dut.get_input_port_b().size(), 3);
  EXPECT_EQ(dut.get_output_port(0).size(), 3);
}

GTEST_TEST(LinearTransformDensityTest, Constructor) {
  TestConstructor<double>();
  TestConstructor<AutoDiffXd>();
}

GTEST_TEST(LinearTransformDensityTest, ToAutoDiff) {
  auto dut = std::make_unique<LinearTransformDensity<double>>(
      RandomDistribution::kUniform, 2, 3);
  EXPECT_TRUE(is_autodiffxd_convertible(*dut));
}

template <typename T>
void TestCalcOutput() {
  LinearTransformDensity<T> dut(RandomDistribution::kUniform, 2, 3);
  auto context = dut.CreateDefaultContext();

  Eigen::Matrix<T, 3, 2> A;
  A << T(1.), T(2.), T(3.), T(4.), T(5.), T(6.);
  const auto& port_A_val = dut.FixConstantA(context.get(), A);
  EXPECT_EQ(port_A_val.template get_vector_value<T>().size(), 6);
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(ExtractDoubleOrThrow(
                  port_A_val.template get_vector_value<T>().get_value()(i)),
              ExtractDoubleOrThrow(*(A.data() + i)));
  }
  Vector2<T> w_in(2, 3);
  dut.get_input_port_w_in().FixValue(context.get(), w_in);

  // Test when port b is not connected.
  const auto w_out_no_b = dut.get_output_port().Eval(*context);
  const Vector3<T> w_out_no_b_expected = A * w_in;
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(ExtractDoubleOrThrow(w_out_no_b(i)),
              ExtractDoubleOrThrow(w_out_no_b_expected(i)));
  }

  // Test with port b connected.
  Vector3<T> b(-1, -2, -3);
  const auto& port_b_val = dut.FixConstantB(context.get(), b);
  EXPECT_EQ(port_b_val.template get_vector_value<T>().size(), 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(ExtractDoubleOrThrow(
                  port_b_val.template get_vector_value<T>().get_value()(i)),
              -i - 1);
  }

  const auto w_out = dut.get_output_port().Eval(*context);
  const Vector3<T> w_out_expected = A * w_in + b;
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(ExtractDoubleOrThrow(w_out(i)),
              ExtractDoubleOrThrow(w_out_expected(i)));
  }
}

GTEST_TEST(LinearTransformDensityTest, CalcOutput) {
  TestCalcOutput<double>();
  TestCalcOutput<AutoDiffXd>();
}

template <typename T>
void CheckGaussianDensity() {
  // Test CalcDensity with a Gaussian distribution
  LinearTransformDensity<T> dut(RandomDistribution::kGaussian, 2, 2);
  auto context = dut.CreateDefaultContext();
  Vector2<T> w_in(0, 0);
  dut.get_input_port_w_in().FixValue(context.get(), w_in);
  Matrix2<T> A = Matrix2<T>::Identity();
  dut.FixConstantA(context.get(), A);
  // Leave b disconnected.
  T density = dut.CalcDensity(*context);
  // I know that w_out has mean 0 and unit variance.
  double density_expected = 1. / (2 * M_PI);
  EXPECT_NEAR(ExtractDoubleOrThrow(density), density_expected, 10 * kEps);

  // Now fix the b port to a non-zero value. The density should remain the same.
  Vector2<T> b(1., 2.);
  dut.FixConstantB(context.get(), b);
  EXPECT_NEAR(ExtractDoubleOrThrow(dut.CalcDensity(*context)),
              ExtractDoubleOrThrow(density), 10 * kEps);

  // Now test with diagonal scaling A.
  A = Vector2<T>(2, -3).asDiagonal();
  dut.FixConstantA(context.get(), A);
  density = dut.CalcDensity(*context);
  density_expected =
      1. /
      (std::sqrt(std::pow(2 * M_PI, 2) *
                 (ExtractDoubleOrThrow((A * A.transpose()).determinant()))));
  EXPECT_NEAR(ExtractDoubleOrThrow(density), density_expected, 10 * kEps);

  // Move w_in away from 0.
  w_in << T(1.), T(2.);
  dut.get_input_port_w_in().FixValue(context.get(), w_in);
  // Compute the density of w_in for each dimension.
  double density_w_in =
      std::exp(-0.5) / std::sqrt(2 * M_PI) * std::exp(-2) / std::sqrt(2 * M_PI);
  density = dut.CalcDensity(*context);
  // The density of w_out should be scaled the input density by 6 (the absolute
  // value of product of A's diagonal entries).
  EXPECT_NEAR(ExtractDoubleOrThrow(density), density_w_in / 6, 10 * kEps);

  // Now test non-diagonal A.
  A << 1, 2, 3, 4;
  // We know that w_out = A * w_in + b is also a Gaussian distribution, with
  // mean b and variance AAᵀ. We can compute the density as
  // exp(-0.5 (w_out - b)ᵀ(AAᵀ)⁻¹(w_out-b)) / (sqrt((2π)ᵏ*det(AAᵀ))
  dut.FixConstantA(context.get(), A);
  density = dut.CalcDensity(*context);
  auto w_out = dut.get_output_port().Eval(*context);
  Eigen::LDLT<Matrix2<T>> ldlt_solver;
  Matrix2<T> cov = A * A.transpose();
  ldlt_solver.compute(cov);
  density_expected =
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

template <typename T>
void CheckUniformDensity() {
  LinearTransformDensity<T> dut(RandomDistribution::kUniform, 2, 2);
  auto context = dut.CreateDefaultContext();

  Matrix2<T> A;
  A << T(1.), T(2.), T(3.), T(-5.);
  dut.FixConstantA(context.get(), A);
  // Leave port b disconnected.
  // w_in has a non-zero probability.
  Vector2<T> w_in(0.1, 0.4);
  dut.get_input_port_w_in().FixValue(context.get(), w_in);
  T density = dut.CalcDensity(*context);
  using std::abs;
  T density_expected = T(1.) / abs(A.determinant());
  EXPECT_NEAR(ExtractDoubleOrThrow(density),
              ExtractDoubleOrThrow(density_expected), 10 * kEps);

  // Connect port b
  dut.FixConstantB(context.get(), Vector2<T>(1, 2));
  density = dut.CalcDensity(*context);
  EXPECT_NEAR(ExtractDoubleOrThrow(density),
              ExtractDoubleOrThrow(density_expected), 10 * kEps);

  // w_in has zero probability.
  for (const Vector2<T>& w_in_zero_prob :
       {Vector2<T>(-0.2, 0.5), Vector2<T>(0.1, 1.2), Vector2<T>(-0.2, 1.3)}) {
    dut.get_input_port_w_in().FixValue(context.get(), w_in_zero_prob);
    density = dut.CalcDensity(*context);
    EXPECT_EQ(ExtractDoubleOrThrow(density), 0);
  }
}

GTEST_TEST(LinearTransformDensityTest, CalcDensityUniform) {
  CheckUniformDensity<double>();
  CheckUniformDensity<AutoDiffXd>();
}

template <typename T>
void CheckExponentialDensity() {
  LinearTransformDensity<T> dut(RandomDistribution::kExponential, 2, 2);
  auto context = dut.CreateDefaultContext();
  Matrix2<T> A;
  A << T(1.), T(2.), T(3.), T(-5.);
  dut.FixConstantA(context.get(), A);

  // Leave port b disconnected.
  // w_in has a non-zero probability.
  Vector2<T> w_in(1., 2.);
  dut.get_input_port_w_in().FixValue(context.get(), w_in);
  T density = dut.CalcDensity(*context);
  using std::abs;
  using std::exp;
  T density_expected = exp(-w_in(0)) * exp(-w_in(1)) / abs(A.determinant());
  EXPECT_NEAR(ExtractDoubleOrThrow(density),
              ExtractDoubleOrThrow(density_expected), 10 * kEps);

  // Connect b port. The density is unchanged.
  dut.FixConstantB(context.get(), Vector2<T>(0.5, 1.5));
  density = dut.CalcDensity(*context);
  EXPECT_NEAR(ExtractDoubleOrThrow(density),
              ExtractDoubleOrThrow(density_expected), 10 * kEps);

  // Test w_in with zero probability.
  for (const auto& w_in_zero_prob :
       {Vector2<T>(-0.1, 1.2), Vector2<T>(0.5, -1.5), Vector2<T>(-0.2, 0.8)}) {
    dut.get_input_port_w_in().FixValue(context.get(), w_in_zero_prob);
    density = dut.CalcDensity(*context);
    EXPECT_EQ(ExtractDoubleOrThrow(density), 0);
  }
}

GTEST_TEST(LinearTransformDensityTest, CalcDensityExponential) {
  CheckExponentialDensity<double>();
  CheckExponentialDensity<AutoDiffXd>();
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

// Compute the probability density of drawing a sample from a distribution
// TODO(hongkai.dai): this function is useful, move it out of test.
double CalcRandomSourceDensity(
    RandomDistribution distribution,
    const Eigen::Ref<const Eigen::VectorXd>& sample) {
  switch (distribution) {
    case RandomDistribution::kUniform: {
      if ((sample.array() < 0).any() || (sample.array() > 1).any()) {
        return 0.;
      } else {
        return 1.;
      }
    }
    case RandomDistribution::kGaussian: {
      return ((-0.5 * sample.array() * sample.array()).exp() /
              std::sqrt(2 * M_PI))
          .prod();
    }
    case RandomDistribution::kExponential: {
      if ((sample.array() < 0).any()) {
        return 0;
      } else {
        return (-sample.array()).exp().prod();
      }
    }
  }
  DRAKE_UNREACHABLE();
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
      const double p = CalcRandomSourceDensity(distribution, sample);
      return -sample * p;
    }
    case RandomDistribution::kExponential: {
      // The density of a single Exponential variable is
      // p = exp(-x) if x>=0
      // p = 0 if x < 0
      // The gradient is
      // dpdx = -p
      const double p = CalcRandomSourceDensity(distribution, sample);
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
  auto b = math::initializeAutoDiff(Eigen::Vector2d(2, 4));
  Matrix2<AutoDiffXd> A =
      Eigen::Vector2d(2, -3).cast<AutoDiffXd>().asDiagonal();
  Eigen::Matrix2d A_val = math::autoDiffToValueMatrix(A);
  dut.FixConstantA(context.get(), A);
  dut.FixConstantB(context.get(), b);
  Vector2<AutoDiffXd> w_in = w_in_val.cast<AutoDiffXd>();
  dut.get_input_port_w_in().FixValue(context.get(), w_in);
  auto density = dut.CalcDensity(*context);
  Eigen::Vector2d ddensity_db =
      (1 / std::abs(A_val.determinant()) *
       CalcRandomSourceDensityGradient(distribution,
                                       math::autoDiffToValueMatrix(w_in))
           .transpose() *
       -A_val.inverse())
          .transpose();
  Eigen::VectorXd density_grad = density.derivatives().rows() == 0
                                     ? Eigen::Vector2d::Zero()
                                     : density.derivatives();
  EXPECT_TRUE(CompareMatrices(density_grad, ddensity_db, 10 * kEps));

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
  A_val = math::autoDiffToValueMatrix(A);
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
  Eigen::Vector2d w_out_val =
      math::autoDiffToValueMatrix(dut.get_output_port().Eval(*context));
  Eigen::Vector2d b_val = math::autoDiffToValueMatrix(b);
  const double A_det_abs = std::abs(A_val.determinant());
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      const double dp_dAij =
          CalcRandomSourceDensityGradient(distribution, w_in_val)
                  .dot(-A_inv * dA_dAij[i][j] * A_inv * (w_out_val - b_val)) /
              A_det_abs -
          CalcRandomSourceDensity(distribution, w_in_val) *
              std::pow(A_det_abs, -2) * dA_abs_det_dAij[i][j];
      if (density.derivatives().rows() == 0) {
        EXPECT_EQ(dp_dAij, 0);
      } else {
        EXPECT_NEAR(density.derivatives()(i + 2 * j), dp_dAij, 10 * kEps);
      }
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
