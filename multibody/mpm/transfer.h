#pragma once

#include <array>
#include <vector>
#include <iostream>

#include "particles.h"
#include "sparse_grid.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

template <typename T>
class Transfer {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Transfer);
  Transfer() = default;

  Transfer(T dt, SparseGrid<T>* sparse_grid, ParticleData<T>* particles)
      : dt_(dt), sparse_grid_(sparse_grid), particles_(particles) {
    sparse_grid_->Allocate(particles_->x);
    D_inverse_ = 4.0 / (sparse_grid_->dx() * sparse_grid_->dx());
  }

  void GridToParticles() {
    const std::vector<int>& sentinel_particles =
        sparse_grid_->sentinel_particles();
    const int num_blocks = sparse_grid_->num_blocks();
    const std::vector<ParticleIndex>& particle_indices =
        sparse_grid_->particle_indices();
    for (int b = 0; b < num_blocks; ++b) {
      const int particle_start = sentinel_particles[b];
      const int particle_end = sentinel_particles[b + 1];
      for (int p = particle_start; p < particle_end; ++p) {
        const ParticleIndex& particle_index = particle_indices[p];
        Particle<T> particle = particles_->particle(p);
        particle.v.setZero();
        particle.C.setZero();

        const BSplineWeights<T>& bspline = particle.bspline;
        // TODO: refresh neighbor nodes and data only when necessary.
        const NeighborArray<Vector3<T>>& grid_x =
            sparse_grid_->GetNeighborNodes(particle.x);
        const NeighborArray<GridData<T>>& grid_data =
            sparse_grid_->GetNeighborData(particle_index.base_node_offset);
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
              const Vector3<T>& vi = grid_data[i][j][k].v;
              const Vector3<T>& xi = grid_x[i][j][k];
              const T& w = bspline.weight(i, j, k);
              particle.v += w * vi;
              particle.C += (w * vi) * ((xi - particle.x).transpose());
            }
          }
        }
        particle.x += particle.v * dt_;
        particle.C *= D_inverse_;
        particle.F += particle.C * dt_;
      }
    }
  }

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
