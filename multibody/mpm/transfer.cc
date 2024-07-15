#include "transfer.h"

#include <array>
#include <iostream>
#include <vector>

#include <omp.h>

#include "drake/common/ssize.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

template <typename T>
Transfer<T>::Transfer(T dt, SparseGrid<T>* sparse_grid,
                      ParticleData<T>* particles)
    : dt_(dt), sparse_grid_(sparse_grid), particles_(particles) {
  sparse_grid_->Allocate(particles);
  D_inverse_ = 4.0 / (sparse_grid_->dx() * sparse_grid_->dx());
}

template <typename T>
void Transfer<T>::ParallelParticleToGrid(const Parallelism parallelize) {
  const std::vector<ParticleIndex>& particle_indices =
      sparse_grid_->particle_indices();
  const std::vector<int>& sentinel_particles =
      sparse_grid_->sentinel_particles();
  const std::array<std::vector<int>, 8>& colored_pages =
      sparse_grid_->colored_pages();
  for (int c = 0; c < 8; ++c) {
    const std::vector<int>& blocks = colored_pages[c];
    [[maybe_unused]] const int num_threads = parallelize.num_threads();
#if defined(_OPENMP)
#pragma omp parallel for num_threads(num_threads)
#endif
    for (int b : blocks) {
      NeighborArray<Vector3<T>> grid_x;
      NeighborArray<GridData<T>> grid_data;
      bool need_new_pad = true;
      bool end_of_block = false;
      const int particle_start = sentinel_particles[b];
      const int particle_end = sentinel_particles[b + 1];
      for (int p = particle_start; p < particle_end; ++p) {
        const ParticleIndex& particle_index = particle_indices[p];
        const Particle<T> particle = particles_->particle(particle_index.index);
        const T& m = particle.m;
        const Vector3<T>& x = particle.x;
        const Vector3<T>& v = particle.v;
        const Matrix3<T>& C = particle.C;
        const Matrix3<T>& P = particle.P;
        const BSplineWeights<T>& bspline = particle.bspline;
        if (need_new_pad) {
          grid_x = sparse_grid_->GetNeighborNodes(x);
          grid_data =
              sparse_grid_->GetNeighborData(particle_index.base_node_offset);
        }
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
              const T& w = bspline.weight(i, j, k);
              const Vector3<T>& xi = grid_x[i][j][k];
              // TODO(xuchenhan): Better document this. The formula isn't
              // exactly the same as the paper spells out.
              /* Use the grid velocity data to store momentum. */
              T mi = m * w;
              grid_data[i][j][k].v +=
                  mi * v + (m * C - D_inverse_ * dt_ * P) * (xi - x) * w;
              grid_data[i][j][k].m += mi;
            }
          }
        }
        end_of_block = p + 1 == particle_end;
        need_new_pad =
            end_of_block || (particle_index.base_node_offset !=
                             particle_indices[p + 1].base_node_offset);
        if (end_of_block || need_new_pad) {
          sparse_grid_->SetNeighborData(particle_index.base_node_offset,
                                        grid_data);
        }
      }
    }
  }
}

template <typename T>
void Transfer<T>::SerialParticleToGrid() {
  const std::vector<ParticleIndex>& particle_indices =
      sparse_grid_->particle_indices();
  const std::vector<int>& sentinel_particles =
      sparse_grid_->sentinel_particles();
  const int num_blocks = sparse_grid_->num_blocks();
  for (int b = 0; b < num_blocks; ++b) {
    NeighborArray<Vector3<T>> grid_x;
    NeighborArray<GridData<T>> grid_data;
    bool need_new_pad = true;
    bool end_of_block = false;
    const int particle_start = sentinel_particles[b];
    const int particle_end = sentinel_particles[b + 1];
    for (int p = particle_start; p < particle_end; ++p) {
      const ParticleIndex& particle_index = particle_indices[p];
      const Particle<T> particle = particles_->particle(particle_index.index);
      const T& m = particle.m;
      const Vector3<T>& x = particle.x;
      const Vector3<T>& v = particle.v;
      const Matrix3<T>& C = particle.C;
      const Matrix3<T>& P = particle.P;
      const BSplineWeights<T>& bspline = particle.bspline;
      if (need_new_pad) {
        grid_x = sparse_grid_->GetNeighborNodes(x);
        grid_data =
            sparse_grid_->GetNeighborData(particle_index.base_node_offset);
      }
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          for (int k = 0; k < 3; ++k) {
            const T& w = bspline.weight(i, j, k);
            const Vector3<T>& xi = grid_x[i][j][k];
            // TODO(xuchenhan): Better document this. The formula isn't
            // exactly the same as the paper spells out.
            /* Use the grid velocity data to store momentum. */
            T mi = m * w;
            grid_data[i][j][k].v +=
                mi * v + (m * C - D_inverse_ * dt_ * P) * (xi - x) * w;
            grid_data[i][j][k].m += mi;
          }
        }
      }
      end_of_block = p + 1 == particle_end;
      need_new_pad = end_of_block || (particle_index.base_node_offset !=
                                      particle_indices[p + 1].base_node_offset);
      if (end_of_block || need_new_pad) {
        sparse_grid_->SetNeighborData(particle_index.base_node_offset,
                                      grid_data);
      }
    }
  }
}

template <typename T>
void Transfer<T>::ParallelGridToParticle(const Parallelism parallelize) {
  const std::vector<int>& sentinel_particles =
      sparse_grid_->sentinel_particles();
  const int num_blocks = sparse_grid_->num_blocks();
  const std::vector<ParticleIndex>& particle_indices =
      sparse_grid_->particle_indices();
  [[maybe_unused]] const int num_threads = parallelize.num_threads();
#if defined(_OPENMP)
#pragma omp parallel for num_threads(num_threads)
#endif
  for (int b = 0; b < num_blocks; ++b) {
    NeighborArray<Vector3<T>> grid_x;
    NeighborArray<GridData<T>> grid_data;
    const int particle_start = sentinel_particles[b];
    const int particle_end = sentinel_particles[b + 1];
    for (int p = particle_start; p < particle_end; ++p) {
      const ParticleIndex& particle_index = particle_indices[p];
      Particle<T> particle = particles_->particle(particle_index.index);
      particle.v.setZero();
      particle.C.setZero();
      /* Write grid data to local pad. */
      if (p == particle_start || particle_index.base_node_offset !=
                                     particle_indices[p - 1].base_node_offset) {
        grid_data =
            sparse_grid_->GetNeighborData(particle_index.base_node_offset);
        grid_x = sparse_grid_->GetNeighborNodes(particle.x);
      }
      const BSplineWeights<T>& bspline = particle.bspline;
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          for (int k = 0; k < 3; ++k) {
            const Vector3<T>& vi = grid_data[i][j][k].v;
            const Vector3<T>& xi = grid_x[i][j][k];
            const T& w = bspline.weight(i, j, k);
            particle.v += w * vi;
            particle.C += (w * vi) * (xi - particle.x).transpose();
          }
        }
      }
      particle.x += particle.v * dt_;
      particle.C *= D_inverse_;
      particle.F += particle.C * dt_;
    }
  }
}

template <typename T>
void Transfer<T>::SerialGridToParticle() {
  const std::vector<int>& sentinel_particles =
      sparse_grid_->sentinel_particles();
  const int num_blocks = sparse_grid_->num_blocks();
  const std::vector<ParticleIndex>& particle_indices =
      sparse_grid_->particle_indices();
  for (int b = 0; b < num_blocks; ++b) {
    NeighborArray<Vector3<T>> grid_x;
    NeighborArray<GridData<T>> grid_data;
    const int particle_start = sentinel_particles[b];
    const int particle_end = sentinel_particles[b + 1];
    for (int p = particle_start; p < particle_end; ++p) {
      const ParticleIndex& particle_index = particle_indices[p];
      Particle<T> particle = particles_->particle(particle_index.index);
      particle.v.setZero();
      particle.C.setZero();
      /* Write grid data to local pad. */
      if (p == particle_start || particle_index.base_node_offset !=
                                     particle_indices[p - 1].base_node_offset) {
        grid_data =
            sparse_grid_->GetNeighborData(particle_index.base_node_offset);
        grid_x = sparse_grid_->GetNeighborNodes(particle.x);
      }
      const BSplineWeights<T>& bspline = particle.bspline;
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          for (int k = 0; k < 3; ++k) {
            const Vector3<T>& vi = grid_data[i][j][k].v;
            const Vector3<T>& xi = grid_x[i][j][k];
            const T& w = bspline.weight(i, j, k);
            particle.v += w * vi;
            particle.C += (w * vi) * (xi - particle.x).transpose();
          }
        }
      }
      particle.x += particle.v * dt_;
      particle.C *= D_inverse_;
      particle.F += particle.C * dt_;
    }
  }
}

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake

template class drake::multibody::mpm::internal::Transfer<double>;
template class drake::multibody::mpm::internal::Transfer<float>;
