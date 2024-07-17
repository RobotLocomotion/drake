#include "transfer.h"

#include <array>
#include <iostream>
#include <vector>

#include "simd_scalar.h"
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
void Transfer<T>::SerialSimdParticleToGrid() {
  const int lanes = SimdScalar<T>::lanes();
  const std::vector<ParticleIndex>& particle_indices =
      sparse_grid_->particle_indices();
  const std::vector<int>& sentinel_particles =
      sparse_grid_->sentinel_particles();
  const int num_blocks = sparse_grid_->num_blocks();
  std::vector<int> indices;
  indices.reserve(lanes);
  for (int b = 0; b < num_blocks; ++b) {
    NeighborArray<Vector3<T>> grid_x;
    NeighborArray<GridData<T>> grid_data;
    const int particle_start = sentinel_particles[b];
    const int particle_end = sentinel_particles[b + 1];
    int p = particle_start;
    bool need_new_pad = true;
    bool end_of_block = false;
    while (p < particle_end) {
      int next_p = p + 1;
      // TODO(xuchenhan-tri): Can we put particles with different base nodes
      // into a single SIMD register? If we do that, the weight computation
      // needs to be slightly different. The grid node reduction will also be
      // different.
      while (next_p < particle_end &&
             particle_indices[next_p].base_node_offset ==
                 particle_indices[p].base_node_offset &&
             next_p - p < lanes) {
        ++next_p;
      }
      if (need_new_pad) {
        /* Write grid data to local pad to ensure they fit in L1 cache. */
        grid_data =
            sparse_grid_->GetNeighborData(particle_indices[p].base_node_offset);
        grid_x = sparse_grid_->GetNeighborNodes(
            particles_->x[particle_indices[p].index]);
      }
      indices.clear();
      for (int i = 0; i < next_p - p; ++i) {
        indices.push_back(particle_indices[p + i].index);
      }
      SimdScalar<T> m = Load(particles_->m, indices);
      Vector3<SimdScalar<T>> x = Load(particles_->x, indices);
      Vector3<SimdScalar<T>> v = Load(particles_->v, indices);
      Matrix3<SimdScalar<T>> C = Load(particles_->C, indices);
      Matrix3<SimdScalar<T>> P = Load(particles_->P, indices);
      const BSplineWeights<SimdScalar<T>> bspline =
          BSplineWeights<SimdScalar<T>>(x, sparse_grid_->dx());
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          for (int k = 0; k < 3; ++k) {
            const SimdScalar<T>& w = bspline.weight(i, j, k);
            const Vector3<T>& xi = grid_x[i][j][k];
            // TODO(xuchenhan): Better document this. The formula isn't
            // exactly the same as the paper spells out.
            /* Use the grid velocity data to store momentum. */
            const SimdScalar<T> mi = m * w;
            const Vector3<SimdScalar<T>> mvi =
                mi * v + (m * C - D_inverse_ * dt_ * P) * (xi - x) * w;
            grid_data[i][j][k].m += ReduceSum(mi);
            grid_data[i][j][k].v += ReduceSum(mvi);
          }
        }
      }
      end_of_block = next_p == particle_end;
      need_new_pad =
          !end_of_block && (particle_indices[p].base_node_offset !=
                            particle_indices[next_p].base_node_offset);
      if (end_of_block || need_new_pad) {
        sparse_grid_->SetNeighborData(particle_indices[p].base_node_offset,
                                      grid_data);
      }
      p = next_p;
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
  const int num_blocks = sparse_grid_->num_blocks();
  [[maybe_unused]] const int num_threads = parallelize.num_threads();
#if defined(_OPENMP)
#pragma omp parallel for num_threads(num_threads)
#endif
  for (int b = 0; b < num_blocks; ++b) {
    const std::vector<int>& sentinel_particles =
        sparse_grid_->sentinel_particles();
    const std::vector<ParticleIndex>& particle_indices =
        sparse_grid_->particle_indices();
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

template <typename T>
void Transfer<T>::ParallelSimdGridToParticle(const Parallelism parallelize) {
  const int lanes = SimdScalar<T>::lanes();
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
    const int particle_start = sentinel_particles[b];
    const int particle_end = sentinel_particles[b + 1];
    std::vector<int> indices;
    indices.reserve(lanes);
    NeighborArray<Vector3<T>> grid_x;
    NeighborArray<GridData<T>> grid_data;
    int p = particle_start;
    bool need_new_pad = true;
    while (p < particle_end) {
      int next_p = p + 1;
      while (particle_indices[next_p].base_node_offset ==
                 particle_indices[p].base_node_offset &&
             next_p - p < lanes && next_p < particle_end) {
        ++next_p;
      }
      // TODO(xuchenhan): should I get rid of this branch and always load new
      // pad? Also, if we make the pad as large as the page, then we can load
      // exactly once.
      if (need_new_pad) {
        /* Write grid data to local pad to ensure they fit in L1 cache. */
        grid_data =
            sparse_grid_->GetNeighborData(particle_indices[p].base_node_offset);
        grid_x = sparse_grid_->GetNeighborNodes(
            particles_->x[particle_indices[p].index]);
      }
      indices.clear();
      for (int i = 0; i < next_p - p; ++i) {
        indices.push_back(particle_indices[p + i].index);
      }
      Vector3<SimdScalar<T>> v = Vector3<SimdScalar<T>>::Zero();
      Matrix3<SimdScalar<T>> B = Matrix3<SimdScalar<T>>::Zero();
      Vector3<SimdScalar<T>> x = Load(particles_->x, indices);
      Matrix3<SimdScalar<T>> C = Load(particles_->C, indices);
      Matrix3<SimdScalar<T>> F = Load(particles_->F, indices);
      const BSplineWeights<SimdScalar<T>>& bspline =
          BSplineWeights<SimdScalar<T>>(x, sparse_grid_->dx());
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          for (int k = 0; k < 3; ++k) {
            const Vector3<T>& vi = grid_data[i][j][k].v;
            const Vector3<T>& xi = grid_x[i][j][k];
            const SimdScalar<T>& w = bspline.weight(i, j, k);
            v += w * vi;
            B += (w * vi) * (xi - x).transpose();
          }
        }
      }
      x += v * dt_;
      C = B * D_inverse_;
      F += C * dt_;
      Store(v, &particles_->v, indices);
      Store(x, &particles_->x, indices);
      Store(C, &particles_->C, indices);
      Store(F, &particles_->F, indices);

      need_new_pad = particle_indices[next_p].base_node_offset !=
                     particle_indices[p].base_node_offset;
      p = next_p;
    }
  }
}

template <typename T>
void Transfer<T>::SerialSimdGridToParticle() {
  const int lanes = SimdScalar<T>::lanes();
  const std::vector<int>& sentinel_particles =
      sparse_grid_->sentinel_particles();
  const int num_blocks = sparse_grid_->num_blocks();
  const std::vector<ParticleIndex>& particle_indices =
      sparse_grid_->particle_indices();
  std::vector<int> indices;
  indices.reserve(lanes);
  for (int b = 0; b < num_blocks; ++b) {
    const int particle_start = sentinel_particles[b];
    const int particle_end = sentinel_particles[b + 1];
    NeighborArray<Vector3<T>> grid_x;
    NeighborArray<GridData<T>> grid_data;
    int p = particle_start;
    bool need_new_pad = true;
    while (p < particle_end) {
      int next_p = p + 1;
      while (next_p < particle_end &&
             particle_indices[next_p].base_node_offset ==
                 particle_indices[p].base_node_offset &&
             next_p - p < lanes) {
        ++next_p;
      }
      // TODO(xuchenhan): should I get rid of this branch and always load new
      // pad? Also, if we make the pad as large as the page, then we can load
      // exactly once.
      if (need_new_pad) {
        /* Write grid data to local pad to ensure they fit in L1 cache. */
        grid_data =
            sparse_grid_->GetNeighborData(particle_indices[p].base_node_offset);
        grid_x = sparse_grid_->GetNeighborNodes(
            particles_->x[particle_indices[p].index]);
      }
      indices.clear();
      for (int i = 0; i < next_p - p; ++i) {
        indices.push_back(particle_indices[p + i].index);
      }
      Vector3<SimdScalar<T>> v = Vector3<SimdScalar<T>>::Zero();
      Matrix3<SimdScalar<T>> B = Matrix3<SimdScalar<T>>::Zero();
      Vector3<SimdScalar<T>> x = Load(particles_->x, indices);
      Matrix3<SimdScalar<T>> C = Load(particles_->C, indices);
      Matrix3<SimdScalar<T>> F = Load(particles_->F, indices);
      const BSplineWeights<SimdScalar<T>> bspline =
          BSplineWeights<SimdScalar<T>>(x, sparse_grid_->dx());
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          for (int k = 0; k < 3; ++k) {
            const Vector3<T>& vi = grid_data[i][j][k].v;
            const Vector3<T>& xi = grid_x[i][j][k];
            const SimdScalar<T>& w = bspline.weight(i, j, k);
            v += w * vi;
            B += (w * vi) * (xi - x).transpose();
          }
        }
      }
      x += v * dt_;
      C = B * D_inverse_;
      F += C * dt_;
      Store(v, &particles_->v, indices);
      Store(x, &particles_->x, indices);
      Store(C, &particles_->C, indices);
      Store(F, &particles_->F, indices);

      need_new_pad = (next_p != particle_end) &&
                     (particle_indices[next_p].base_node_offset !=
                      particle_indices[p].base_node_offset);
      p = next_p;
    }
  }
}

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake

template class drake::multibody::mpm::internal::Transfer<double>;
template class drake::multibody::mpm::internal::Transfer<float>;
