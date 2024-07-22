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

  void ParticleToGrid(Parallelism parallelize = false);

  void GridToParticle(Parallelism parallelize = false);

  void SerialParticleToGrid();
  void SerialSimdParticleToGrid();
  void ParallelParticleToGrid(Parallelism parallelize);
  void ParallelSimdParticleToGrid(Parallelism parallelize);

  void SerialGridToParticle();
  void SerialSimdGridToParticle();
  void ParallelGridToParticle(Parallelism parallelize);
  void ParallelSimdGridToParticle(Parallelism parallelize);

 private:
  T dt_{0.0};
  SparseGrid<T>* sparse_grid_{};
  ParticleData<T>* particles_{};
  T D_inverse_{0.0};
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
