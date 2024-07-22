#include "transfer.h"
#include <gflags/gflags.h>

#include "drake/tools/performance/fixture_common.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {
namespace {

using Eigen::Vector3f;
using Eigen::Matrix3f;
constexpr int kNumThreads = 32;

class TransferBenchmark : public benchmark::Fixture {
 public:
  TransferBenchmark() {
    tools::performance::AddMinMaxStatistics(this);
    this->Unit(benchmark::kMillisecond);
  }

  void SetUp(benchmark::State& state) {  // NOLINT(runtime/references)
    // Number of inputs.
    const int num_nodes_per_dim = state.range(0);
    DRAKE_DEMAND(num_nodes_per_dim > 0);
    const int particles_per_cell = state.range(1);
    DRAKE_DEMAND(particles_per_cell > 0);
    const float dx = 0.01;

    // Create the particles.
    for (int i = 0; i < num_nodes_per_dim; ++i) {
      for (int j = 0; j < num_nodes_per_dim; ++j) {
        for (int k = 0; k < num_nodes_per_dim; ++k) {
          const Vector3f base_node(dx * i, dx * j, dx * k);
          for (int p = 0; p < particles_per_cell; ++p) {
            particles_.m.push_back(1.0);
            const Vector3f x = base_node + p * dx / (particles_per_cell + 1.0) *
                                               Vector3f::Ones();
            particles_.x.push_back(x);
            particles_.v.push_back(Vector3f(1 * i, 2 * j, 3 * k));
            particles_.F.push_back(Matrix3f::Identity());
            particles_.C.push_back(Matrix3f::Identity());
            particles_.P.push_back(Matrix3f::Identity());
            particles_.bspline.push_back(BSplineWeights<float>(x, dx));
          }
        }
      }
    }

    // Create the grid.
    grid_ = std::make_unique<SparseGrid<float>>(dx);
    const float dt = 0.01;
    transfer_ = std::make_unique<Transfer<float>>(dt, grid_.get(), &particles_);
  }

 protected:
  std::unique_ptr<SparseGrid<float>> grid_;
  ParticleData<float> particles_;
  std::unique_ptr<Transfer<float>> transfer_;
};

BENCHMARK_DEFINE_F(TransferBenchmark, SerialP2G)(benchmark::State& state) {  // NOLINT
  for (auto _ : state) {
    transfer_->SerialParticleToGrid();
  }
}
// The Args are { num_nodes_per_dim, particles_per_cell }.
BENCHMARK_REGISTER_F(TransferBenchmark, SerialP2G)->Args({16, 8})->Args({32, 8});

BENCHMARK_DEFINE_F(TransferBenchmark, SerialSimdP2G)(benchmark::State& state) {  // NOLINT
  for (auto _ : state) {
    transfer_->SerialSimdParticleToGrid();
  }
}
// The Args are { num_nodes_per_dim, particles_per_cell }.
BENCHMARK_REGISTER_F(TransferBenchmark, SerialSimdP2G)->Args({16, 8})->Args({32, 8});

BENCHMARK_DEFINE_F(TransferBenchmark, ParallelP2G)(benchmark::State& state) {  // NOLINT
  for (auto _ : state) {
    transfer_->ParallelParticleToGrid(Parallelism(kNumThreads));
  }
}
// The Args are { num_nodes_per_dim, particles_per_cell }.
BENCHMARK_REGISTER_F(TransferBenchmark, ParallelP2G)->Args({16, 8})->Args({32, 8});

BENCHMARK_DEFINE_F(TransferBenchmark, ParallelSimdP2G)(benchmark::State& state) {  // NOLINT
  for (auto _ : state) {
    transfer_->ParallelSimdParticleToGrid(Parallelism(kNumThreads));
  }
}
// The Args are { num_nodes_per_dim, particles_per_cell }.
BENCHMARK_REGISTER_F(TransferBenchmark, ParallelSimdP2G)->Args({16, 8})->Args({32, 8});


BENCHMARK_DEFINE_F(TransferBenchmark, SerialG2P)(benchmark::State& state) {  // NOLINT
  for (auto _ : state) {
    transfer_->SerialGridToParticle();
  }
}
// The Args are { num_nodes_per_dim, particles_per_cell }.
BENCHMARK_REGISTER_F(TransferBenchmark, SerialG2P)->Args({16, 8})->Args({32, 8});

BENCHMARK_DEFINE_F(TransferBenchmark, SerialSimdG2P)(benchmark::State& state) {  // NOLINT
  for (auto _ : state) {
    transfer_->SerialSimdGridToParticle();
  }
}
// The Args are { num_nodes_per_dim, particles_per_cell }.
BENCHMARK_REGISTER_F(TransferBenchmark, SerialSimdG2P)->Args({16, 8})->Args({32, 8});

BENCHMARK_DEFINE_F(TransferBenchmark, ParallelG2P)(benchmark::State& state) {  // NOLINT
  for (auto _ : state) {
    transfer_->ParallelGridToParticle(Parallelism(kNumThreads));
  }
}
// The Args are { num_nodes_per_dim, particles_per_cell }.
BENCHMARK_REGISTER_F(TransferBenchmark, ParallelG2P)->Args({16, 8})->Args({32, 8});

BENCHMARK_DEFINE_F(TransferBenchmark, ParallelSimdG2P)(benchmark::State& state) {  // NOLINT
  for (auto _ : state) {
    transfer_->ParallelSimdGridToParticle(Parallelism(kNumThreads));
  }
}
// The Args are { num_nodes_per_dim, particles_per_cell }.
BENCHMARK_REGISTER_F(TransferBenchmark, ParallelSimdG2P)->Args({16, 8})->Args({32, 8});


}  // namespace
}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake