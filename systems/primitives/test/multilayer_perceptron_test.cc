#include "drake/systems/primitives/multilayer_perceptron.h"

#include <limits>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace systems {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;

template <typename T>
void BasicTest() {
  MultilayerPerceptron<T> mlp({1, 2, 3, 4}, PerceptronActivationType::kReLU);

  EXPECT_EQ(mlp.layers(), std::vector<int>({1, 2, 3, 4}));
  for (int i = 0; i < 2; ++i) {
    EXPECT_EQ(mlp.activation_type(i), PerceptronActivationType::kReLU);
  }
  EXPECT_EQ(mlp.activation_type(2), PerceptronActivationType::kIdentity);

  auto context = mlp.CreateDefaultContext();

  EXPECT_EQ(mlp.GetWeights(*context, 0).rows(), 2);
  EXPECT_EQ(mlp.GetWeights(*context, 0).cols(), 1);
  EXPECT_EQ(mlp.GetBiases(*context, 0).rows(), 2);

  EXPECT_EQ(mlp.GetWeights(*context, 1).rows(), 3);
  EXPECT_EQ(mlp.GetWeights(*context, 1).cols(), 2);
  EXPECT_EQ(mlp.GetBiases(*context, 1).rows(), 3);

  EXPECT_EQ(mlp.GetWeights(*context, 2).rows(), 4);
  EXPECT_EQ(mlp.GetWeights(*context, 2).cols(), 3);
  EXPECT_EQ(mlp.GetBiases(*context, 2).rows(), 4);

  // Default weights are all zero, so the output will be zero.
  mlp.get_input_port().FixValue(context.get(), Vector1<T>{2.0});
  EXPECT_TRUE(CompareMatrices(mlp.get_output_port().Eval(*context),
                              Eigen::Vector4d::Zero(), 1e-14));

  Eigen::Matrix<T, 3, 2> W;
  W << 3, 4, 5, 6, 7, 8;
  const Vector3<T> b{0.4, 0.72, 0.13};

  EXPECT_FALSE(CompareMatrices(mlp.GetWeights(*context, 1), W));
  EXPECT_FALSE(CompareMatrices(mlp.GetBiases(*context, 1), b));
  mlp.SetWeights(context.get(), 1, W);
  mlp.SetBiases(context.get(), 1, b);
  EXPECT_TRUE(CompareMatrices(mlp.GetWeights(*context, 1), W));
  EXPECT_TRUE(CompareMatrices(mlp.GetBiases(*context, 1), b));

  VectorX<T> params = VectorX<T>::Zero(mlp.num_parameters());
  EXPECT_FALSE(CompareMatrices(mlp.GetWeights(params, 1), W));
  EXPECT_FALSE(CompareMatrices(mlp.GetBiases(params, 1), b));
  mlp.SetWeights(&params, 1, W);
  mlp.SetBiases(&params, 1, b);
  EXPECT_TRUE(CompareMatrices(mlp.GetWeights(params, 1), W));
  EXPECT_TRUE(CompareMatrices(mlp.GetBiases(params, 1), b));

  VectorX<T> expected = VectorX<T>::Constant(mlp.num_parameters(), .3);
  mlp.GetMutableParameters(context.get()) = expected;
  EXPECT_TRUE(CompareMatrices(mlp.GetParameters(*context), expected));
}

GTEST_TEST(MultilayerPerceptronTest, Basic) {
  BasicTest<double>();
  BasicTest<AutoDiffXd>();
  BasicTest<symbolic::Expression>();
}

GTEST_TEST(MultilayerPerceptronTest, RandomParameters) {
  MultilayerPerceptron<double> mlp({1, 100, 1},
                                   PerceptronActivationType::kIdentity);
  auto context = mlp.CreateDefaultContext();
  RandomGenerator generator(243);

  // Generate N sets of random parameters.
  int N = 100;
  VectorXd mean = VectorXd::Zero(mlp.num_parameters());
  VectorXd var = VectorXd::Zero(mlp.num_parameters());
  for (int i = 0; i < N; ++i) {
    mlp.SetRandomContext(context.get(), &generator);
    mean += mlp.GetParameters(*context);
    // Compute the variance with the true mean (zero) instead of the empirical
    // mean, for tighter statistics. This reduces to E[x^2].
    var += mlp.GetParameters(*context).array().square().matrix();
  }
  mean /= N;
  var /= N;
  VectorXd std_dev = var.array().sqrt();

  // Note: The large tolerances below are due to the small N and the fact that
  // it is a worse-case bound over 100 parameters (each). The order of magnitude
  // differences between the first and second layers confirms that the logic is
  // correct.

  // First layer should have mean≈0, std_dev≈1.
  EXPECT_TRUE(CompareMatrices(mlp.GetWeights(mean, 0), VectorXd::Zero(N), 0.3));
  EXPECT_TRUE(CompareMatrices(mlp.GetBiases(mean, 0), VectorXd::Zero(N), 0.3));
  EXPECT_TRUE(CompareMatrices(mlp.GetWeights(std_dev, 0),
                              VectorXd::Constant(N, 1.0), 0.2));
  EXPECT_TRUE(CompareMatrices(mlp.GetBiases(std_dev, 0),
                              VectorXd::Constant(N, 1.0), 0.2));
  // Second layer should have mean≈0, std_dev≈0.1.
  EXPECT_TRUE(CompareMatrices(mlp.GetWeights(mean, 1),
                              Eigen::RowVectorXd::Zero(N), 0.03));
  EXPECT_TRUE(CompareMatrices(mlp.GetBiases(mean, 1), VectorXd::Zero(1), 0.03));
  EXPECT_TRUE(CompareMatrices(mlp.GetWeights(std_dev, 1),
                              Eigen::RowVectorXd::Constant(N, 0.1), 0.02));
  EXPECT_TRUE(CompareMatrices(mlp.GetBiases(std_dev, 1),
                              VectorXd::Constant(1, 0.1), 0.02));
}

template <typename T>
void CalcOutputTest(
    const std::vector<PerceptronActivationType>& activation_types) {
  MultilayerPerceptron<T> mlp({2, 3, 3, 2}, activation_types);
  auto context = mlp.CreateDefaultContext();

  const Vector2<T> x{0.1, 0.2};
  mlp.get_input_port().FixValue(context.get(), x);

  Eigen::Matrix<T, 3, 2> W0;
  W0 << 0.3, 0.4, 0.5, 0.6, 0.7, 0.8;
  const Vector3<T> b0{0.4, 0.72, -2};  // -2 term ensures that ReLU is active.
  mlp.SetWeights(context.get(), 0, W0);
  mlp.SetBiases(context.get(), 0, b0);

  Vector3<T> y0 = (W0 * x + b0);
  if (mlp.activation_type(0) == kReLU) {
    y0 = y0.array().max(T{0.0}).matrix();
  } else if (mlp.activation_type(0) == kTanh) {
    y0 = y0.array().tanh().matrix();
  }

  Eigen::Matrix<T, 3, 3> W1;
  W1 << 0.63, 0.226, 0.47, 0.73, 0.324, 0.363, 0.62, 0.765, 0.73;
  const Vector3<T> b1{0.43, 0.63, 0.32};
  mlp.SetWeights(context.get(), 1, W1);
  mlp.SetBiases(context.get(), 1, b1);

  Vector3<T> y1 = (W1 * y0 + b1);
  if (mlp.activation_type(1) == kReLU) {
    y1 = y1.array().max(T{0.0}).matrix();
  } else if (mlp.activation_type(1) == kTanh) {
    y1 = y1.array().tanh().matrix();
  }

  Eigen::Matrix<T, 2, 3> W2;
  W2 << 0.23, 0.62, -0.2, -.73, 0.14, 0.6;
  const Vector2<T> b2{0.24, 0.17};
  mlp.SetWeights(context.get(), 2, W2);
  mlp.SetBiases(context.get(), 2, b2);

  Vector2<T> y = (W2 * y1 + b2);
  if (mlp.activation_type(2) == kReLU) {
    y = y.array().max(T{0.0}).matrix();
  } else if (mlp.activation_type(2) == kTanh) {
    y = y.array().tanh().matrix();
  }

  if constexpr (std::is_same_v<T, double>) {
    // CalcOutput<double> should not have any dynamic allocations.
    drake::test::LimitMalloc guard({.max_num_allocations = 0});
    mlp.get_output_port().Eval(*context);
  }

  EXPECT_TRUE(CompareMatrices(mlp.get_output_port().Eval(*context), y, 1e-14));
}

GTEST_TEST(MultilayerPerceptronTest, CalcOutput) {
  for (const std::vector<PerceptronActivationType>& activation_types :
       std::vector<std::vector<PerceptronActivationType>>(
           {{kReLU, kReLU, kIdentity},
            {kIdentity, kReLU, kTanh},
            {kTanh, kTanh, kIdentity}})) {
    CalcOutputTest<double>(activation_types);
    CalcOutputTest<AutoDiffXd>(activation_types);
    CalcOutputTest<symbolic::Expression>(activation_types);
  }
}

// Check that backprop gives the same gradients as AutoDiffXd.
void BackpropTest(PerceptronActivationType type, bool use_sin_cos = false) {
  std::unique_ptr<MultilayerPerceptron<double>> owned_mlp;
  std::unique_ptr<MultilayerPerceptron<AutoDiffXd>> owned_mlp_ad;
  if (use_sin_cos) {
    std::vector<bool> use_sin_cos_for_input({false, true});
    std::vector<int> remaining_layers({3, 3, 2});
    std::vector<PerceptronActivationType> activation_types({type, type, type});
    owned_mlp = std::make_unique<MultilayerPerceptron<double>>(
        use_sin_cos_for_input, remaining_layers, activation_types);
    owned_mlp_ad = std::make_unique<MultilayerPerceptron<AutoDiffXd>>(
        use_sin_cos_for_input, remaining_layers, activation_types);
  } else {
    std::vector<int> layers({2, 3, 3, 2});
    owned_mlp = std::make_unique<MultilayerPerceptron<double>>(layers, type);
    owned_mlp_ad =
        std::make_unique<MultilayerPerceptron<AutoDiffXd>>(layers, type);
  }
  MultilayerPerceptron<double>& mlp = *owned_mlp;
  MultilayerPerceptron<AutoDiffXd>& mlp_ad = *owned_mlp_ad;

  Eigen::Matrix<double, 2, 3> X, Y_desired;
  X << 0.23, 0.62, -0.2, -.73, 0.14, 0.6;
  Y_desired << 0.3, 0.4, 0.5, 0.6, 0.7, 0.8;

  auto context = mlp.CreateDefaultContext();
  auto context_ad = mlp_ad.CreateDefaultContext();

  RandomGenerator generator(243);
  mlp.SetRandomContext(context.get(), &generator);

  mlp_ad.SetParameters(context_ad.get(),
                       math::InitializeAutoDiff(mlp.GetParameters(*context)));

  // Compute MSE error gradient using autodiff.
  AutoDiffXd loss_ad{0.0};
  for (int i = 0; i < X.cols(); ++i) {
    mlp_ad.get_input_port().FixValue(context_ad.get(),
                                     VectorX<AutoDiffXd>(X.col(i)));
    VectorX<AutoDiffXd> y_ad = mlp_ad.get_output_port().Eval(*context_ad);
    loss_ad += (Y_desired.col(i) - y_ad).squaredNorm();
  }
  loss_ad /= 3.0;

  // Compute it again using Backpropagation.
  Eigen::VectorXd dloss_dparams(mlp.num_parameters());
  double loss = mlp.BackpropagationMeanSquaredError(*context, X, Y_desired,
                                                    &dloss_dparams);

  // Check that they give the same values.
  EXPECT_NEAR(loss, loss_ad.value(), 1e-14);
  EXPECT_TRUE(CompareMatrices(dloss_dparams,
                              loss_ad.derivatives().size()
                                  ? loss_ad.derivatives()
                                  : VectorXd::Zero(mlp.num_parameters()).eval(),
                              1e-14));

  {  // A second call with the same size input should not allocate.
    drake::test::LimitMalloc guard({.max_num_allocations = 0});
    loss = mlp.BackpropagationMeanSquaredError(*context, X, Y_desired,
                                               &dloss_dparams);
  }
}

GTEST_TEST(MultilayerPerceptionTest, Backprop) {
  for (const auto& type : {kIdentity, kReLU, kTanh}) {
    BackpropTest(type, false);
    BackpropTest(type, true);
  }
}

GTEST_TEST(MultilayerPereceptronTest, BatchOutput) {
  for (const auto& type : {kIdentity, kReLU, kTanh}) {
    std::vector<int> layers({2, 3, 3, 2});
    MultilayerPerceptron<double> mlp(layers, type);
    auto context = mlp.CreateDefaultContext();

    RandomGenerator generator(243);
    mlp.SetRandomContext(context.get(), &generator);

    Eigen::Matrix<double, 2, 3> X, Y, Y_desired;
    X << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
    for (int i = 0; i < 3; ++i) {
      mlp.get_input_port().FixValue(context.get(), X.col(i));
      Y_desired.col(i) = mlp.get_output_port().Eval(*context);
    }
    mlp.BatchOutput(*context, X, &Y);

    EXPECT_TRUE(CompareMatrices(Y, Y_desired, 1e-14));

    {  // A second call with the same size input should not allocate.
      drake::test::LimitMalloc guard({.max_num_allocations = 0});
      mlp.BatchOutput(*context, X, &Y);
    }
  }
}

GTEST_TEST(MultilayerPereceptronTest, BatchOutputWithGradients) {
  for (const auto& type : {kIdentity, kReLU, kTanh}) {
    std::vector<int> layers({2, 3, 3, 1});

    MultilayerPerceptron<double> mlp(layers, type);
    auto context = mlp.CreateDefaultContext();
    RandomGenerator generator(243);
    mlp.SetRandomContext(context.get(), &generator);

    MultilayerPerceptron<AutoDiffXd> mlp_ad(layers, type);
    auto context_ad = mlp_ad.CreateDefaultContext();
    mlp_ad.SetParameters(context_ad.get(),
                         mlp.GetParameters(*context).cast<AutoDiffXd>());

    Eigen::Matrix<double, 2, 3> X, dYdX, dYdX_desired;
    X << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
    Eigen::Matrix<double, 1, 3> Y, Y_desired;
    for (int i = 0; i < 3; ++i) {
      mlp_ad.get_input_port().FixValue(context_ad.get(),
                                       math::InitializeAutoDiff(X.col(i)));
      AutoDiffXd y = mlp_ad.get_output_port().Eval(*context_ad)[0];
      Y_desired(0, i) = y.value();
      // Note: Have to handle the case where the autodiff gradients are not set
      // (e.g. for ReLU the input might be truncated before reaching the
      // output).
      dYdX_desired.col(i) =
          y.derivatives().size() ? y.derivatives() : VectorXd::Zero(2).eval();
    }
    mlp.BatchOutput(*context, X, &Y, &dYdX);

    EXPECT_TRUE(CompareMatrices(Y, Y_desired, 1e-14));
    EXPECT_TRUE(CompareMatrices(dYdX, dYdX_desired, 1e-14));

    {  // A second call with the same size input should not allocate.
      drake::test::LimitMalloc guard({.max_num_allocations = 0});
      mlp.BatchOutput(*context, X, &Y, &dYdX);
    }
  }
}

GTEST_TEST(MultilayerPerceptronTest, BatchOutputWithGradientsThrows) {
  MultilayerPerceptron<double> mlp({2, 2});
  auto context = mlp.CreateDefaultContext();
  const Eigen::Matrix2d X = Eigen::Matrix2d::Zero();
  Eigen::Matrix2d Y, dYdX;

  DRAKE_EXPECT_THROWS_MESSAGE(
      mlp.BatchOutput(*context, X, &Y, &dYdX),
      ".*dYdX != nullptr, but BatchOutput only supports "
      "gradients when the output layer has size 1.");
}

GTEST_TEST(MultilayerPerceptronTest, ScalarConversion) {
  MultilayerPerceptron<double> mlp({1, 2, 3, 4}, kReLU);

  auto mlp_ad = mlp.ToAutoDiffXd();
  EXPECT_EQ(mlp_ad->get_input_port().size(), 1);
  EXPECT_EQ(mlp_ad->get_output_port().size(), 4);

  auto mlp_sym = mlp.ToSymbolic();
  EXPECT_EQ(mlp_sym->get_input_port().size(), 1);
  EXPECT_EQ(mlp_sym->get_output_port().size(), 4);
}

GTEST_TEST(MultilayerPerceptronTest, SinCosFeatures) {
  MultilayerPerceptron<double> mlp({true, false}, {4, 1},
                                   {kIdentity, kIdentity});

  EXPECT_EQ(mlp.get_input_port().size(), 2);
  EXPECT_EQ(mlp.layers(), std::vector<int>({3, 4, 1}));

  auto context = mlp.CreateDefaultContext();
  RandomGenerator generator(243);
  mlp.SetRandomContext(context.get(), &generator);

  // When the output is based on random parameters, it should be periodic in 2π
  // for the first input (and not for the second).
  Eigen::Matrix<double, 2, 3> X;
  // clang-format off
  X << 0.1, 0.1 + 2*M_PI, 0.1,
       0.4, 0.4,          0.4 + 2*M_PI;
  // clang-format on
  Eigen::Matrix<double, 1, 3> Y;
  mlp.BatchOutput(*context, X, &Y);
  EXPECT_NEAR(Y[0], Y[1], 1e-14);
  EXPECT_GE(std::abs(Y[0] - Y[2]), 1e-3);

  // Check the gradients.
  auto owned_mlp_ad = mlp.ToAutoDiffXd();
  auto& mlp_ad = dynamic_cast<MultilayerPerceptron<AutoDiffXd>&>(*owned_mlp_ad);
  EXPECT_EQ(mlp_ad.get_input_port().size(), 2);
  EXPECT_EQ(mlp_ad.layers(), std::vector<int>({3, 4, 1}));

  auto context_ad = mlp_ad.CreateDefaultContext();
  mlp_ad.SetParameters(context_ad.get(),
                       mlp.GetParameters(*context).cast<AutoDiffXd>());

  Eigen::Matrix<double, 2, 3> dYdX, dYdX_desired;
  X << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  Eigen::Matrix<double, 1, 3> Y_desired;
  for (int i = 0; i < 3; ++i) {
    mlp_ad.get_input_port().FixValue(context_ad.get(),
                                     math::InitializeAutoDiff(X.col(i)));
    AutoDiffXd y = mlp_ad.get_output_port().Eval(*context_ad)[0];
    Y_desired(0, i) = y.value();
    dYdX_desired.col(i) =
        y.derivatives().size() ? y.derivatives() : VectorXd::Zero(2).eval();
  }
  mlp.BatchOutput(*context, X, &Y, &dYdX);

  EXPECT_TRUE(CompareMatrices(Y, Y_desired, 1e-14));
  EXPECT_TRUE(CompareMatrices(dYdX, dYdX_desired, 1e-14));

  {  // A second call with the same size input should not allocate.
    drake::test::LimitMalloc guard({.max_num_allocations = 0});
    mlp.BatchOutput(*context, X, &Y, &dYdX);
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
