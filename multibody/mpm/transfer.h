#pragma once

#include "drake/multibody/mpm/particle_data.h"
#include "drake/multibody/mpm/sparse_grid.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

/* This class handles particle-to-grid and grid-to-particle transfer in a single
 Moving Least Squares Material Point Method (MLS-MPM) [Hu et al. 2018] step
 using Affine Particle In Cell (APIC) [Jiang et al. 2016]. Quadratic B-spline
 interpolation is used for the transfer.

 [Hu et al. 2018] Hu, Y., Fang, Y., Ge, Z., Qu, Z., Zhu, Y., Pradhana, A., &
 Jiang, C. (2018). A moving least squares material point method with
 displacement discontinuity and two-way rigid body coupling. ACM Transactions on
 Graphics (TOG), 37(4), 1-14.

 [Jiang et al. 2016] Jiang, C., Schroeder, C., Teran, J., Stomakhin, A., &
 Selle, A. (2016). The material point method for simulating continuum materials.
 In ACM SIGGRAPH 2016 courses.

 @tparam Grid  The type of the MPM grid. Can be one of the following
               - SparseGrid<double>
               - SparseGrid<float>
               - MockSparseGrid<double>     (for testing only).
               - MockSparseGrid<AutoDiffXd> (for testing only). */

template <typename Grid = SparseGrid<double>>
class Transfer {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Transfer);

  /* The scalar type for the grid and particle data. */
  using T = typename Grid::Scalar;

  /* Constructs a Transfer object for particle-to-grid and grid-to-particle
   transfers.
   @pre dt > 0 and dx > 0. */
  Transfer(T dt, double dx);

  /* The Particle to grid transfer (P2G). After the call to P2G, the grid stores
   the mass and momentum transfered from the particles using APIC. In the
   process, also mark the grid nodes that are in support of any particle that is
   participating in a constraint with the "participating" flag.
   @note The `v` attribute of the grid data at the end of the operation stores
   the momentum, not velocity, of the grid node. */
  void ParticleToGrid(const ParticleData<T>& particle, Grid* grid);

  /* Grid to particle transfer (G2P). After the call to G2P, the particles store
   the mass and momentum transfered from the grid using APIC.
   @pre the grid stores mass and velocity (not momentum). Hence, the velocity
   from the grid needs to be processed after P2G and before G2P. */
  void GridToParticle(const Grid& grid, ParticleData<T>* particle);

  T D_inverse() const { return D_inverse_; }

 private:
  T dt_{};
  double dx_{};
  /* The D inverse matrix in computing the affine matrix. See page 42 in the MPM
   course notes referenced in the class documentation. */
  T D_inverse_{};
  T D_inverse_dt_{};
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
