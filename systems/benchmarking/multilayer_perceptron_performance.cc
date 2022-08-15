/* @file
 A simple binary to test the performance of the MultilayerPerceptron.
 Run this at the command line with, e.g.

 time bazel-bin/systems/primitives/multilayer_perceptron_performance \
  --batch_size=1000 --iterations=10
*/

#include "systems/primitives/multilayer_perceptron.h"
#include <gflags/gflags.h>

namespace drake {
namespace systems {
namespace {

using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::VectorXd;

// TODO(jwnimmer-tri) Rewrite these to use googlebench Args().
DEFINE_int32(batch_size, 2, "Number of batch evaluations.");
DEFINE_int32(width, 256, "Number of units in each hidden layer.");
DEFINE_int32(iterations, 2, "Number of times to call the method.");
DEFINE_string(method, "all",
              "Restrict the run to one API method.\n"
              "[--method={all,backprop,output,output_gradient}]\n"
              "By default, all of them will be run.\n");

int do_main() {
  const std::list<std::string> allowed_methods(
      {"all", "backprop", "output", "output_gradient"});
  DRAKE_DEMAND(std::find(allowed_methods.begin(), allowed_methods.end(),
                         FLAGS_method) != allowed_methods.end());

  const int num_inputs{10};
  // Use 1 output so that we can call BatchOutput with gradients.
  MultilayerPerceptron<double> mlp({num_inputs, FLAGS_width, FLAGS_width, 1});

  auto context = mlp.CreateDefaultContext();
  RandomGenerator generator(243);
  mlp.SetRandomContext(context.get(), &generator);

  MatrixXd X = MatrixXd::Ones(num_inputs, FLAGS_batch_size);
  RowVectorXd Y(FLAGS_batch_size);
  RowVectorXd dloss_dparams(mlp.num_parameters());
  RowVectorXd Yd = RowVectorXd::Ones(FLAGS_batch_size);
  MatrixXd dYdX(num_inputs, FLAGS_batch_size);

  if (FLAGS_method == "backprop" || FLAGS_method == "all") {
    for (int i = 0; i < FLAGS_iterations; ++i) {
      mlp.BackpropagationMeanSquaredError(*context, X, Yd, &dloss_dparams);
    }
  }
  if (FLAGS_method == "output" || FLAGS_method == "all") {
    for (int i = 0; i < FLAGS_iterations; ++i) {
      mlp.BatchOutput(*context, X, &Y);
    }
  }
  if (FLAGS_method == "output_gradient" || FLAGS_method == "all") {
    for (int i = 0; i < FLAGS_iterations; ++i) {
      mlp.BatchOutput(*context, X, &Y, &dYdX);
    }
  }
  return 0;
}

}  // namespace
}  // namespace systems
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::systems::do_main();
}
