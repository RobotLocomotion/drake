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
//    this->Unit(benchmark::kSecond);

    // Use 1 output so that we can call BatchOutput with gradients.
    mlp = std::make_unique<MultilayerPerceptron<double>>(
        std::vector<int>({num_inputs, FLAGS_width, FLAGS_width, 1}));
    X = MatrixXd::Ones(num_inputs, FLAGS_batch_size);
    Y.resize(FLAGS_batch_size);
    dloss_dparams.resize(mlp->num_parameters());
    Yd = RowVectorXd::Ones(FLAGS_batch_size);
    dYdX.resize(num_inputs, FLAGS_batch_size);

    context = mlp->CreateDefaultContext();
    RandomGenerator generator(243);
    mlp->SetRandomContext(context.get(), &generator);
  }

  const int num_inputs{10};

  std::unique_ptr<MultilayerPerceptron<double>> mlp;
  std::unique_ptr<Context<double>> context;

  MatrixXd X;
  RowVectorXd Y;
  RowVectorXd dloss_dparams;
  RowVectorXd Yd;
  MatrixXd dYdX;
};

BENCHMARK_F(MultilayerPerceptronBenchmark, Backprop)(benchmark::State& state) {  // NOLINT
  for (auto _ : state) {
    mlp->BackpropagationMeanSquaredError(*context, X, Yd, &dloss_dparams);
  }
}

BENCHMARK_F(MultilayerPerceptronBenchmark, Output)(benchmark::State& state) {  // NOLINT
  for (auto _ : state) {
    mlp->BatchOutput(*context, X, &Y);
  }
}

BENCHMARK_F(MultilayerPerceptronBenchmark, OutputGradient)(benchmark::State& state) {  // NOLINT
  for (auto _ : state) {
    mlp->BatchOutput(*context, X, &Y, &dYdX);
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
