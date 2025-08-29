#include <memory>
#include <vector>

#include <benchmark/benchmark.h>

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/tools/performance/fixture_common.h"

/* A collection of scenarios to benchmark, scoped to cover all code within the
drake/systems/framework package. */

namespace drake {
namespace systems {
namespace {

class BasicFixture : public benchmark::Fixture {
 public:
  BasicFixture() { tools::performance::AddMinMaxStatistics(this); }

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
    // Invalidate the furthest upstream CacheEntry, in order to force
    // re-computation of all downstream values.  This also mimics a common
    // scenario of interest, where the inputs change on every tick.
    input.GetMutableData();
    output.Eval(*context_);
  }
}

// Helper function for the DiagramBuild benchmark. Creates a diagram containing
// num_systems subsystems.  When depth==0, each subsystem is an Adder, otherwise
// each subsystem is a recursive self-call with the next smaller depth.
std::unique_ptr<DiagramBuilder<double>> MakeDiagramBuilder(int num_systems,
                                                           int depth) {
  DRAKE_DEMAND(num_systems > 0);
  DRAKE_DEMAND(depth >= 0);
  auto builder = std::make_unique<DiagramBuilder<double>>();
  std::vector<System<double>*> children;
  for (int i = 0; i < num_systems; ++i) {
    if (depth == 0) {
      children.push_back(builder->AddSystem<Adder>(num_systems, 1));
    } else {
      children.push_back(builder->AddSystem(
          MakeDiagramBuilder(num_systems, depth - 1)->Build()));
    }
    System<double>* child = children.back();
    // The child's input ports are connected either to the other childrens'
    // output ports (when possible), or else the diagram's input ports.
    for (int j = 0; j < num_systems; ++j) {
      const InputPort<double>& input = child->get_input_port(j);
      if (j < i) {
        builder->Connect(children.at(j)->get_output_port(), input);
      } else {
        if (i == 0) {
          builder->ExportInput(input, fmt::to_string(j));
        } else {
          builder->ConnectInput(fmt::to_string(j), input);
        }
      }
    }
  }
  builder->ExportOutput(children.back()->get_output_port());
  return builder;
}

void DiagramBuild(benchmark::State& state) {  // NOLINT
  const int num_systems = state.range(0);
  const int depth = state.range(1);
  std::unique_ptr<DiagramBuilder<double>> builder;
  std::unique_ptr<Diagram<double>> diagram;
  for (auto _ : state) {
    // Create a DiagramBuilder, sans timekeeping.
    state.PauseTiming();
    diagram.reset();
    builder = MakeDiagramBuilder(num_systems, depth);
    state.ResumeTiming();

    // Time the Build operation.
    diagram = builder->Build();
  }
}

BENCHMARK(DiagramBuild)
    ->Unit(benchmark::kMillisecond)
    ->Args({3, 0})
    ->Args({30, 0})
    ->Args({3, 1})
    ->Args({3, 2});

}  // namespace
}  // namespace systems
}  // namespace drake
