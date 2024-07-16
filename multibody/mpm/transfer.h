#pragma once

#include "particles.h"
#include "sparse_grid.h"

#include "drake/common/parallelism.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

template <typename T>
class Transfer {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Transfer);

  Transfer() = default;
  Transfer(T dt, SparseGrid<T>* sparse_grid, ParticleData<T>* particles);

  void ParticleToGrid(Parallelism parallelize = false) {
    if (parallelize.num_threads() == 1) {
      SerialParticleToGrid();
    } else {
      ParallelParticleToGrid(parallelize);
    }
  }

  void GridToParticle(Parallelism parallelize = false) {
    if (parallelize.num_threads() == 1) {
      // SerialGridToParticle();
      SerialSimdGridToParticle();
    } else {
      ParallelParticleToGrid(parallelize);
    }
  }

 private:
  void SerialParticleToGrid();
  void ParallelParticleToGrid(Parallelism parallelize);

  void SerialGridToParticle();
  void SerialSimdGridToParticle();
  void ParallelGridToParticle(Parallelism parallelize);

  T dt_{0.0};
  SparseGrid<T>* sparse_grid_{};
  ParticleData<T>* particles_{};
  T D_inverse_{0.0};
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
