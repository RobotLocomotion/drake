/* @file
Measures the performance of the MultilayerPerceptron implementation.
Refer to the README.md for more information. */

#include <memory>
#include <vector>

#include <gflags/gflags.h>

#include "drake/systems/primitives/multilayer_perceptron.h"
#include "drake/tools/performance/fixture_common.h"

namespace drake {
namespace systems {
namespace {

using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::VectorXd;

// TODO(jwnimmer-tri) Add benchmarking cases for input sin/cos features.

class Mlp : public benchmark::Fixture {
 public:
  Mlp() {
    tools::performance::AddMinMaxStatistics(this);
    this->Unit(benchmark::kMicrosecond);
  }

  void SetUp(benchmark::State& state) {  // NOLINT(runtime/references)
    // Number of inputs.
    const int num_inputs = state.range(0);
    DRAKE_DEMAND(num_inputs >= 1);
    // Number of layers.
    const int num_layers = state.range(1);
    DRAKE_DEMAND(num_layers >= 2);
    // Number of units in each hidden layer.
    const int width = state.range(2);
    DRAKE_DEMAND(width >= 1);
    // Use 1 output so that we can call BatchOutput with gradients.
    const int num_outputs = 1;
    // Number of batch evaluations.
    const int batch_size = state.range(3);
    DRAKE_DEMAND(batch_size >= 1);

    // Create the MLP.
    std::vector<int> layers;
    layers.push_back(num_inputs);
    for (int i = 0; i < (num_layers - 2); ++i) {
      layers.push_back(width);
    }
    layers.push_back(num_outputs);
    mlp_ = std::make_unique<MultilayerPerceptron<double>>(layers);

    // Prepare the input/output matrix storage.
    X_ = MatrixXd::Ones(num_inputs, batch_size);
    Y_.resize(batch_size);
    dloss_dparams_.resize(mlp_->num_parameters());
    Yd_ = RowVectorXd::Ones(batch_size);
    dYdX_.resize(num_inputs, batch_size);

    // Prepare a random context.
    context_ = mlp_->CreateDefaultContext();
    RandomGenerator generator(243);
    mlp_->SetRandomContext(context_.get(), &generator);
  }

 protected:
  std::unique_ptr<MultilayerPerceptron<double>> mlp_;
  std::unique_ptr<Context<double>> context_;

  MatrixXd X_;
  RowVectorXd Y_;
  RowVectorXd dloss_dparams_;
  RowVectorXd Yd_;
  MatrixXd dYdX_;
};

BENCHMARK_DEFINE_F(Mlp, Backprop)(benchmark::State& state) {  // NOLINT
  for (auto _ : state) {
    mlp_->BackpropagationMeanSquaredError(*context_, X_, Yd_, &dloss_dparams_);
  }
}

// The Args are { num_inputs, num_layers, width, batch_size }. A few notes
// about common parameter values:
// - The default architecture in stablebaselines3 has 4 layers, width=64.
// - It's relatively rare to have MLPs with more than 8 layers.
// - Batch sizes tend to be powers of 2 (16, 32, ..., 256).
// - Width also tends to be powers of 2 (64, 128, ...).
BENCHMARK_REGISTER_F(Mlp, Backprop)
    ->Args({10, 4, 64, 32})
    ->Args({10, 4, 64, 64})
    ->Args({10, 4, 64, 128})
    ->Args({10, 4, 64, 256})
    ->Args({10, 4, 128, 128})
    ->Args({10, 4, 256, 256})
    ->Args({128, 4, 64, 256})
    ->Args({128, 8, 64, 256});

BENCHMARK_DEFINE_F(Mlp, Output)(benchmark::State& state) {  // NOLINT
  for (auto _ : state) {
    mlp_->BatchOutput(*context_, X_, &Y_);
  }
}
// The Args are { num_inputs, num_layers, width, batch_size }.
BENCHMARK_REGISTER_F(Mlp, Output)
    ->Args({10, 4, 64, 32})
    ->Args({10, 4, 64, 64})
    ->Args({10, 4, 64, 128})
    ->Args({10, 4, 64, 256})
    ->Args({10, 4, 128, 128})
    ->Args({10, 4, 256, 256})
    ->Args({128, 4, 64, 256})
    ->Args({128, 8, 64, 256});

BENCHMARK_DEFINE_F(Mlp, OutputGradient)(benchmark::State& state) {  // NOLINT
  for (auto _ : state) {
    mlp_->BatchOutput(*context_, X_, &Y_, &dYdX_);
  }
}
// The Args are { num_inputs, num_layers, width, batch_size }.
BENCHMARK_REGISTER_F(Mlp, OutputGradient)
    ->Args({10, 4, 64, 32})
    ->Args({10, 4, 64, 64})
    ->Args({10, 4, 64, 128})
    ->Args({10, 4, 64, 256})
    ->Args({10, 4, 128, 128})
    ->Args({10, 4, 256, 256})
    ->Args({128, 4, 64, 256})
    ->Args({128, 8, 64, 256});

}  // namespace
}  // namespace systems
}  // namespace drake
