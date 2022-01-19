#include "drake/systems/primitives/multilayer_perceptron.h"

#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace systems {
namespace {

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
}

GTEST_TEST(MultilayerPerceptronTest, Basic) {
  BasicTest<double>();
  BasicTest<AutoDiffXd>();
  BasicTest<symbolic::Expression>();
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
    y0 = y0.array().max(0.0).matrix();
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
    y1 = y1.array().max(0.0).matrix();
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
    y = y.array().max(0.0).matrix();
  } else if (mlp.activation_type(2) == kTanh) {
    y = y.array().tanh().matrix();
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
void BackpropTest(PerceptronActivationType type) {
  std::vector<int> layers({2, 3, 3, 2});
  MultilayerPerceptron<double> mlp(layers, type);
  MultilayerPerceptron<AutoDiffXd> mlp_ad(layers, type);

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
  EXPECT_TRUE(CompareMatrices(dloss_dparams, loss_ad.derivatives(), 1e-14));
}

GTEST_TEST(MultilayerPerceptionTest, Backprop) {
  for (const auto& type : {kIdentity, kReLU, kTanh}) {
    BackpropTest(type);
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
  }
}

GTEST_TEST(MultilayerPerceptronTest, ScalarConversion) {
  MultilayerPerceptron<double> mlp({1, 2, 3, 4},
                                   kReLU);

  auto mlp_ad = mlp.ToAutoDiffXd();
  EXPECT_EQ(mlp_ad->get_input_port().size(), 1);
  EXPECT_EQ(mlp_ad->get_output_port().size(), 4);

  auto mlp_sym = mlp.ToSymbolic();
  EXPECT_EQ(mlp_sym->get_input_port().size(), 1);
  EXPECT_EQ(mlp_sym->get_output_port().size(), 4);
}

}  // namespace
}  // namespace systems
}  // namespace drake
