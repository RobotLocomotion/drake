#include <benchmark/benchmark.h>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/pass_through.h"

/* A collection of scenarios to benchmark, scoped to cover all code within the
drake/systems/framework package. */

namespace drake {
namespace systems {
namespace {

class BasicFixture : public benchmark::Fixture {
 public:
  BasicFixture() {
    ComputeStatistics("min", [](const std::vector<double>& v) -> double {
        return *(std::min_element(std::begin(v), std::end(v)));
      });
    ComputeStatistics("max", [](const std::vector<double>& v) -> double {
        return *(std::max_element(std::begin(v), std::end(v)));
      });
  }

  // This apparently futile using statement works around "overloaded virtual"
  // errors in g++. All of this is a consequence of the weird deprecation of
  // const-ref State versions of SetUp() and TearDown() in benchmark.h.
  using benchmark::Fixture::SetUp;
  void SetUp(benchmark::State&) override {
    builder_ = std::make_unique<DiagramBuilder<double>>();
  }

  void Build() {
    diagram_ = builder_->Build();
    context_ = diagram_->CreateDefaultContext();
    builder_.reset();
  }

 protected:
  std::unique_ptr<DiagramBuilder<double>> builder_;
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> context_;
};

// NOLINTNEXTLINE(runtime/references) cpplint disapproves of gbench choices.
BENCHMARK_F(BasicFixture, PassThrough3)(benchmark::State& state) {
  const int n = 7;
  auto* alpha = builder_->AddSystem<PassThrough<double>>(n);
  auto* bravo = builder_->AddSystem<PassThrough<double>>(n);
  auto* charlie = builder_->AddSystem<PassThrough<double>>(n);
  builder_->ExportInput(alpha->get_input_port());
  builder_->Cascade(*alpha, *bravo);
  builder_->Cascade(*bravo, *charlie);
  builder_->ExportOutput(charlie->get_output_port());
  Build();

  Eigen::VectorXd value = Eigen::VectorXd::Constant(n, 22.2);
  auto& input = diagram_->get_input_port().FixValue(context_.get(), value);
  auto& output = diagram_->get_output_port();

  for (auto _ : state) {
    input.GetMutableData();
    output.Eval(*context_);
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake

BENCHMARK_MAIN();
