/* @file
 A simple binary to benchmark the performance of the MultilayerPerceptron.
 Run this at the command line with, e.g.

 time bazel-bin/systems/primitives/multilayer_perceptron_performance \
  --batch_size=1000 --iterations=10
*/

#include "systems/primitives/multilayer_perceptron.h"
#include <gflags/gflags.h>

#include "drake/tools/performance/fixture_common.h"

namespace drake {
namespace systems {
namespace {

using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::VectorXd;

// TODO(jwnimmer-tri) Rewrite these to use googlebench Args().
DEFINE_int32(batch_size, 2, "Number of batch evaluations.");
DEFINE_int32(width, 256, "Number of units in each hidden layer.");

class MultilayerPerceptronBenchmark : public benchmark::Fixture {
 public:
  MultilayerPerceptronBenchmark() {
    tools::performance::AddMinMaxStatistics(this);
    this->Unit(benchmark::kMicrosecond);

    // Use 1 output so that we can call BatchOutput with gradients.
    mlp_ = std::make_unique<MultilayerPerceptron<double>>(
         std::vector<int>({num_inputs_, FLAGS_width, FLAGS_width, 1}));
    X_ = MatrixXd::Ones(num_inputs_, FLAGS_batch_size);
    Y_.resize(FLAGS_batch_size);
    dloss_dparams_.resize(mlp_->num_parameters());
    Yd_ = RowVectorXd::Ones(FLAGS_batch_size);
    dYdX_.resize(num_inputs_, FLAGS_batch_size);

    context_ = mlp_->CreateDefaultContext();
    RandomGenerator generator(243);
    mlp_->SetRandomContext(context_.get(), &generator);
  }

 protected:
  const int num_inputs_{10};

  std::unique_ptr<MultilayerPerceptron<double>> mlp_;
  std::unique_ptr<Context<double>> context_;

  MatrixXd X_;
  RowVectorXd Y_;
  RowVectorXd dloss_dparams_;
  RowVectorXd Yd_;
  MatrixXd dYdX_;
};

BENCHMARK_F(MultilayerPerceptronBenchmark, Backprop)(benchmark::State& state) {  // NOLINT
  for (auto _ : state) {
    mlp_->BackpropagationMeanSquaredError(*context_, X_, Yd_, &dloss_dparams_);
  }
}

BENCHMARK_F(MultilayerPerceptronBenchmark, Output)(benchmark::State& state) {  // NOLINT
  for (auto _ : state) {
    mlp_->BatchOutput(*context_, X_, &Y_);
  }
}

BENCHMARK_F(MultilayerPerceptronBenchmark, OutputGradient)(benchmark::State& state) {  // NOLINT
  for (auto _ : state) {
    mlp_->BatchOutput(*context_, X_, &Y_, &dYdX_);
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
