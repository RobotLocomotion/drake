#pragma once

#include "particles.h"
#include "sparse_grid.h"

#include "drake/common/parallelism.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

/* Transfer class for particle-to-grid and grid-to-particle transfer in a single
 Moving Least Squares Material Point Method (MLS-MPM) [Hu et al. 2018] step
 using Affine Particle In Cell (APIC) [Jiang et al. 2016].

 [Hu et al. 2018] Hu, Y., Fang, Y., Ge, Z., Qu, Z., Zhu, Y., Pradhana, A., &
 Jiang, C. (2018). A moving least squares material point method with
 displacement discontinuity and two-way rigid body coupling. ACM Transactions on
 Graphics (TOG), 37(4), 1-14.

 [Jiang et al. 2016] Jiang, C., Schroeder, C., Teran, J., Stomakhin, A., &
 Selle, A. (2016). The material point method for simulating continuum materials.
 In ACM SIGGRAPH 2016 courses. */
template <typename T>
class Transfer {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Transfer);

  /* Constructs a Transfer object for particle-to-grid and grid-to-particle
   transfer between `sparse_grid` and `particles`.
   The constructor prepares the grid for transfer by sorting particle indices
   and allocating memory for grid data.
   @pre sparse_grid and particles are not null. */
  Transfer(T dt, SparseGrid<T>* sparse_grid, ParticleData<T>* particles);

  /* The transfer functions below each come in four flavors as the cross product
   of two options:
   1. Serial or Parallel: Serial functions are single-threaded, while Parallel
   functions use the number of threads specified by the parallelize argument.
   Thread level parallelism is achieved using OpenMP parallel for over grid
   blocks.
   2. Scalar or Simd: Scalar functions use scalar types (double or float), while
   Simd functions use SIMD types (SimdScalar<double> or SimdScalar<float>).
   The Simd functions are vectorized over particles that share the same base
   grid node. */

  /* Particle to grid transfer (P2G). After the call to P2G, the grid store the
   mass and momentum transfered from the particles using APIC. */
  void SerialParticleToGrid();
  void SerialSimdParticleToGrid();
  void ParallelParticleToGrid(Parallelism parallelize);
  void ParallelSimdParticleToGrid(Parallelism parallelize);

  /* Grid to particle transfer (G2P). After the call to G2P, the particles store
   the mass and momentum transfered from the grid using APIC.
   @pre the grid stores mass and velocity (not momentum). Hence, the velocity
   from the grid needs to be processed after P2G and before G2P. */
  void SerialGridToParticle();
  void SerialSimdGridToParticle();
  void ParallelGridToParticle(Parallelism parallelize);
  void ParallelSimdGridToParticle(Parallelism parallelize);

 private:
  T dt_{0.0};
  SparseGrid<T>* sparse_grid_{};
  ParticleData<T>* particles_{};
  /* The D inverse matrix in computing the affine matrix. See page 42 in the MPM
   course notes referenced in the class documentation. */
  T D_inverse_{0.0};
  T D_inverse_dt_{0.0};
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
