#include "transfer.h"

#include <array>
#include <iostream>
#include <vector>

#include "simd_scalar.h"
#if defined(_OPENMP)
#include <omp.h>
#endif

#include "drake/common/ssize.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

template <typename T>
Transfer<T>::Transfer(T dt, SparseGrid<T>* sparse_grid,
                      ParticleData<T>* particles)
    : dt_(dt), sparse_grid_(sparse_grid), particles_(particles) {
  DRAKE_DEMAND(dt > 0);
  DRAKE_DEMAND(sparse_grid != nullptr);
  DRAKE_DEMAND(particles != nullptr);
  sparse_grid_->Allocate(particles->x);
  D_inverse_ = 4.0 / (sparse_grid_->dx() * sparse_grid_->dx());
  D_inverse_dt_ = D_inverse_ * dt_;
}

template <typename T>
void Transfer<T>::SerialParticleToGrid() {
  const std::vector<uint64_t>& base_node_offsets =
      sparse_grid_->base_node_offsets();
  const std::vector<int>& data_indices = sparse_grid_->data_indices();
  const std::vector<int>& sentinel_particles =
      sparse_grid_->sentinel_particles();
  const int num_blocks = sparse_grid_->num_blocks();

  Pad<Vector3<T>> grid_x;
  Pad<GridData<T>> grid_data;
  bool need_new_pad = true;

  for (int b = 0; b < num_blocks; ++b) {
    const int particle_start = sentinel_particles[b];
    const int particle_end = sentinel_particles[b + 1];
    for (int p = particle_start; p < particle_end; ++p) {
      const Particle<T> particle = particles_->particle(data_indices[p]);
      const T& m = particle.m;
      const Vector3<T>& x = particle.x;
      const Vector3<T>& v = particle.v;
      const Matrix3<T>& C = particle.C;
      const Matrix3<T>& tau_v0 = particle.tau_v0;
      BsplineWeights<T> bspline(x, sparse_grid_->dx());
      if (need_new_pad) {
        grid_x = sparse_grid_->GetPadNodes(x);
        grid_data = sparse_grid_->GetPadData(base_node_offsets[p]);
      }
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          for (int k = 0; k < 3; ++k) {
            const T& w = bspline.weight(i, j, k);
            const Vector3<T>& xi = grid_x[i][j][k];
            /* The mass transfer is as described equation (126) in [Jiang et al.
             2016]. The momentum transfer is as described in equation (171) in
             [Jiang et al. 2016] with the force term equivalent to equation (18)
             in [Hu et al. 2018], but simplified. We sketch the proof of the
             equivalence here:

             The new grid momentum is given by mvᵢⁿ + fᵢdt with mvᵢⁿ being the
             grid momentum from the current time step transferred from the
             particles. That is,

             mvᵢⁿ = Σₚ mₚvₚ + Cₚ(xᵢ - xₚ) wᵢₚ (equation 178 [Jiang et al. 2016])

             where wᵢₚ is the weight of the particle p to the grid node i. fᵢdt
             is the change in momentum with the force given by fᵢ = -∂E/∂xᵢ

             E = ∑ₚ VₚΨ(Fₚ) where Vₚ is the volume of the particle p in the
             reference configuration and Ψ is the strain energy density.

             Noting that
               Fₚ = (I + dtCₚ)Fₚⁿ (equation 17 [Hu et al. 2018])
               Cₚ = Bₚ * D⁻¹ (equation 173 [Jiang et al. 2016]), and
               Bₚ = ∑ᵢ wᵢₚ vᵢ(xᵢ − xₚ) (equation 176 [Jiang et al. 2016]),

             we compute -∂E/∂xᵢ and get

              fᵢ = -∑ₚ Vₚ * Pₚ * Fₚⁿᵀ * D⁻¹ * (xᵢ − xₚ) * wᵢₚ

             with Pₚ = ∂Ψ/∂Fₚ. Noting that Pₚ * Fₚⁿᵀ is the Kirchhoff stress,
             we group Vₚ * Pₚ * Fₚⁿᵀ into a single term `tau_v0`. Rearranging
             terms reveals that mvᵢⁿ + fᵢdt is given by the equation in the code
             below. */
            const T mi = m * w;
            grid_data[i][j][k].v +=
                mi * v + (m * C - D_inverse_dt_ * tau_v0) * (xi - x) * w;
            grid_data[i][j][k].m += mi;
          }
        }
      }
      need_new_pad = (p + 1 == particle_end) ||
                     (base_node_offsets[p] != base_node_offsets[p + 1]);
      if (need_new_pad) {
        sparse_grid_->SetPadData(base_node_offsets[p], grid_data);
      }
    }
  }
}

template <typename T>
void Transfer<T>::SerialSimdParticleToGrid() {
  const std::vector<uint64_t>& base_node_offsets =
      sparse_grid_->base_node_offsets();
  const std::vector<int>& data_indices = sparse_grid_->data_indices();
  const std::vector<int>& sentinel_particles =
      sparse_grid_->sentinel_particles();
  const int num_blocks = sparse_grid_->num_blocks();
  Pad<Vector3<T>> grid_x;
  Pad<GridData<T>> grid_data;
  /* Particle indices for processing in a single SIMD instruction. */
  std::vector<int> indices;
  const int lanes = SimdScalar<T>::lanes();
  indices.reserve(lanes);

  for (int b = 0; b < num_blocks; ++b) {
    const int particle_start = sentinel_particles[b];
    const int particle_end = sentinel_particles[b + 1];
    int p = particle_start;
    bool need_new_pad = true;
    while (p < particle_end) {
      if (need_new_pad) {
        grid_data = sparse_grid_->GetPadData(base_node_offsets[p]);
        grid_x = sparse_grid_->GetPadNodes(particles_->x[data_indices[p]]);
      }
      /* We pack as many particles as we can into a single SIMD computation
       until either
       1. we pack all lanes in a simd register, or
       2. we run out of particles with the same base nodes (and thus splats to
       the same grid nodes). */
      int next_p = p + 1;
      while (next_p < particle_end &&
             base_node_offsets[next_p] == base_node_offsets[p] &&
             next_p - p < lanes) {
        ++next_p;
      }
      indices.clear();
      for (int i = p; i < next_p; ++i) {
        indices.push_back(data_indices[i]);
      }
      const SimdScalar<T> m = Load(particles_->m, indices);
      const Vector3<SimdScalar<T>> x = Load(particles_->x, indices);
      const Vector3<SimdScalar<T>> v = Load(particles_->v, indices);
      const Matrix3<SimdScalar<T>> C = Load(particles_->C, indices);
      const Matrix3<SimdScalar<T>> tau_v0 = Load(particles_->tau_v0, indices);
      const BsplineWeights<SimdScalar<T>> bspline =
          BsplineWeights<SimdScalar<T>>(x, sparse_grid_->dx());
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          for (int k = 0; k < 3; ++k) {
            const SimdScalar<T>& w = bspline.weight(i, j, k);
            const Vector3<T>& xi = grid_x[i][j][k];
            const SimdScalar<T> mi = m * w;
            const Vector3<SimdScalar<T>> mvi =
                mi * v + (m * C - D_inverse_dt_ * tau_v0) * (xi - x) * w;
            grid_data[i][j][k].m += ReduceSum(mi);
            grid_data[i][j][k].v += ReduceSum(mvi);
          }
        }
      }
      need_new_pad = (next_p == particle_end) ||
                     (base_node_offsets[p] != base_node_offsets[next_p]);
      if (need_new_pad) {
        sparse_grid_->SetPadData(base_node_offsets[p], grid_data);
      }
      p = next_p;
    }
  }
}

template <typename T>
void Transfer<T>::ParallelParticleToGrid(const Parallelism parallelize) {
  const std::vector<int>& sentinel_particles =
      sparse_grid_->sentinel_particles();
  const std::vector<int>& data_indices = sparse_grid_->data_indices();
  const std::vector<uint64_t>& base_node_offsets =
      sparse_grid_->base_node_offsets();

  const std::array<std::vector<int>, 8>& colored_blocks =
      sparse_grid_->colored_blocks();
  for (int c = 0; c < 8; ++c) {
    const std::vector<int>& blocks = colored_blocks[c];
    [[maybe_unused]] const int num_threads = parallelize.num_threads();
    /* Within a single color, we can parallel for over the blocks without any
     write hazards. */
#if defined(_OPENMP)
#pragma omp parallel for num_threads(num_threads)
#endif
    for (int b : blocks) {
      Pad<Vector3<T>> grid_x;
      Pad<GridData<T>> grid_data;
      bool need_new_pad = true;
      const int particle_start = sentinel_particles[b];
      const int particle_end = sentinel_particles[b + 1];
      for (int p = particle_start; p < particle_end; ++p) {
        const Particle<T> particle = particles_->particle(data_indices[p]);
        const T& m = particle.m;
        const Vector3<T>& x = particle.x;
        const Vector3<T>& v = particle.v;
        const Matrix3<T>& C = particle.C;
        const Matrix3<T>& tau_v0 = particle.tau_v0;
        const BsplineWeights<T> bspline(x, sparse_grid_->dx());
        if (need_new_pad) {
          grid_x = sparse_grid_->GetPadNodes(x);
          grid_data = sparse_grid_->GetPadData(base_node_offsets[p]);
        }
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
              const T& w = bspline.weight(i, j, k);
              const Vector3<T>& xi = grid_x[i][j][k];
              const T mi = m * w;
              grid_data[i][j][k].v +=
                  mi * v + (m * C - D_inverse_dt_ * tau_v0) * (xi - x) * w;
              grid_data[i][j][k].m += mi;
            }
          }
        }
        need_new_pad = (p + 1 == particle_end) ||
                       (base_node_offsets[p] != base_node_offsets[p + 1]);
        if (need_new_pad) {
          sparse_grid_->SetPadData(base_node_offsets[p], grid_data);
        }
      }
    }
  }
}

template <typename T>
void Transfer<T>::ParallelSimdParticleToGrid(const Parallelism parallelize) {
  const std::vector<int>& sentinel_particles =
      sparse_grid_->sentinel_particles();
  const std::array<std::vector<int>, 8>& colored_blocks =
      sparse_grid_->colored_blocks();
  const std::vector<int>& data_indices = sparse_grid_->data_indices();
  const std::vector<uint64_t>& base_node_offsets =
      sparse_grid_->base_node_offsets();

  for (int c = 0; c < 8; ++c) {
    const std::vector<int>& blocks = colored_blocks[c];
    [[maybe_unused]] const int num_threads = parallelize.num_threads();
#if defined(_OPENMP)
#pragma omp parallel for num_threads(num_threads)
#endif
    for (int b : blocks) {
      std::vector<int> indices;
      const int lanes = SimdScalar<T>::lanes();
      indices.reserve(lanes);
      Pad<Vector3<T>> grid_x;
      Pad<GridData<T>> grid_data;
      bool need_new_pad = true;
      const int particle_start = sentinel_particles[b];
      const int particle_end = sentinel_particles[b + 1];
      int p = particle_start;
      while (p < particle_end) {
        int next_p = p + 1;
        while (next_p < particle_end &&
               base_node_offsets[next_p] == base_node_offsets[p] &&
               next_p - p < lanes) {
          ++next_p;
        }
        if (need_new_pad) {
          grid_data = sparse_grid_->GetPadData(base_node_offsets[p]);
          grid_x = sparse_grid_->GetPadNodes(particles_->x[data_indices[p]]);
        }
        indices.clear();
        for (int i = p; i < next_p; ++i) {
          indices.push_back(data_indices[i]);
        }
        const SimdScalar<T> m = Load(particles_->m, indices);
        const Vector3<SimdScalar<T>> x = Load(particles_->x, indices);
        const Vector3<SimdScalar<T>> v = Load(particles_->v, indices);
        const Matrix3<SimdScalar<T>> C = Load(particles_->C, indices);
        const Matrix3<SimdScalar<T>> tau_v0 = Load(particles_->tau_v0, indices);
        const BsplineWeights<SimdScalar<T>> bspline =
            BsplineWeights<SimdScalar<T>>(x, sparse_grid_->dx());
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
                  mi * v + (m * C - D_inverse_dt_ * tau_v0) * (xi - x) * w;
              grid_data[i][j][k].m += ReduceSum(mi);
              grid_data[i][j][k].v += ReduceSum(mvi);
            }
          }
        }
        need_new_pad = (next_p == particle_end) ||
                       (base_node_offsets[next_p] != base_node_offsets[p]);
        if (need_new_pad) {
          sparse_grid_->SetPadData(base_node_offsets[p], grid_data);
        }
        p = next_p;
      }
    }
  }
}

template <typename T>
void Transfer<T>::SerialGridToParticle() {
  const std::vector<int>& sentinel_particles =
      sparse_grid_->sentinel_particles();
  const int num_blocks = sparse_grid_->num_blocks();
  const std::vector<int>& data_indices = sparse_grid_->data_indices();
  const std::vector<uint64_t>& base_node_offsets =
      sparse_grid_->base_node_offsets();

  bool need_new_pad = true;
  Pad<Vector3<T>> grid_x;
  Pad<GridData<T>> grid_data;
  for (int b = 0; b < num_blocks; ++b) {
    const int particle_start = sentinel_particles[b];
    const int particle_end = sentinel_particles[b + 1];
    for (int p = particle_start; p < particle_end; ++p) {
      Particle<T> particle = particles_->particle(data_indices[p]);
      particle.v.setZero();
      particle.C.setZero();
      /* Write grid data to local pad. */
      if (need_new_pad) {
        grid_data = sparse_grid_->GetPadData(base_node_offsets[p]);
        grid_x = sparse_grid_->GetPadNodes(particle.x);
      }
      const BsplineWeights<T> bspline(particle.x, sparse_grid_->dx());
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

      need_new_pad = (p + 1 == particle_end) ||
                     (base_node_offsets[p] != base_node_offsets[p + 1]);
    }
  }
}

template <typename T>
void Transfer<T>::SerialSimdGridToParticle() {
  const int lanes = SimdScalar<T>::lanes();
  const std::vector<int>& sentinel_particles =
      sparse_grid_->sentinel_particles();
  const std::vector<int>& data_indices = sparse_grid_->data_indices();
  const std::vector<uint64_t>& base_node_offsets =
      sparse_grid_->base_node_offsets();
  const int num_blocks = sparse_grid_->num_blocks();

  std::vector<int> indices;
  indices.reserve(lanes);
  for (int b = 0; b < num_blocks; ++b) {
    bool need_new_pad = true;
    Pad<Vector3<T>> grid_x;
    Pad<GridData<T>> grid_data;
    const int particle_start = sentinel_particles[b];
    const int particle_end = sentinel_particles[b + 1];
    int p = particle_start;
    while (p < particle_end) {
      int next_p = p + 1;
      while (next_p < particle_end &&
             base_node_offsets[next_p] == base_node_offsets[p] &&
             next_p - p < lanes) {
        ++next_p;
      }
      if (need_new_pad) {
        grid_data = sparse_grid_->GetPadData(base_node_offsets[p]);
        grid_x = sparse_grid_->GetPadNodes(particles_->x[data_indices[p]]);
      }
      indices.clear();
      for (int i = p; i < next_p; ++i) {
        indices.push_back(data_indices[i]);
      }
      Vector3<SimdScalar<T>> v = Vector3<SimdScalar<T>>::Zero();
      Matrix3<SimdScalar<T>> B = Matrix3<SimdScalar<T>>::Zero();
      Vector3<SimdScalar<T>> x = Load(particles_->x, indices);
      const BsplineWeights<SimdScalar<T>> bspline =
          BsplineWeights<SimdScalar<T>>(x, sparse_grid_->dx());
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
      const Matrix3<SimdScalar<T>> C = B * D_inverse_;
      Matrix3<SimdScalar<T>> F = Load(particles_->F, indices);
      x += v * dt_;
      F += C * dt_;
      Store(v, &particles_->v, indices);
      Store(x, &particles_->x, indices);
      Store(C, &particles_->C, indices);
      Store(F, &particles_->F, indices);

      need_new_pad = (next_p == particle_end) ||
                     (base_node_offsets[next_p] != base_node_offsets[p]);
      p = next_p;
    }
  }
}

template <typename T>
void Transfer<T>::ParallelGridToParticle(const Parallelism parallelize) {
  const std::vector<int>& sentinel_particles =
      sparse_grid_->sentinel_particles();
  const std::vector<int>& data_indices = sparse_grid_->data_indices();
  const std::vector<uint64_t>& base_node_offsets =
      sparse_grid_->base_node_offsets();
  const int num_blocks = sparse_grid_->num_blocks();
  [[maybe_unused]] const int num_threads = parallelize.num_threads();
#if defined(_OPENMP)
#pragma omp parallel for num_threads(num_threads)
#endif
  for (int b = 0; b < num_blocks; ++b) {
    bool need_new_pad = true;
    Pad<Vector3<T>> grid_x;
    Pad<GridData<T>> grid_data;
    const int particle_start = sentinel_particles[b];
    const int particle_end = sentinel_particles[b + 1];
    for (int p = particle_start; p < particle_end; ++p) {
      Particle<T> particle = particles_->particle(data_indices[p]);
      particle.v.setZero();
      particle.C.setZero();
      /* Write grid data to local pad. */
      if (need_new_pad) {
        grid_data = sparse_grid_->GetPadData(base_node_offsets[p]);
        grid_x = sparse_grid_->GetPadNodes(particle.x);
      }
      const BsplineWeights<T> bspline(particle.x, sparse_grid_->dx());
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
      need_new_pad = (p + 1 == particle_end) ||
                     (base_node_offsets[p] != base_node_offsets[p + 1]);
    }
  }
}

template <typename T>
void Transfer<T>::ParallelSimdGridToParticle(const Parallelism parallelize) {
  const int lanes = SimdScalar<T>::lanes();
  const std::vector<int>& sentinel_particles =
      sparse_grid_->sentinel_particles();
  const std::vector<int>& data_indices = sparse_grid_->data_indices();
  const std::vector<uint64_t>& base_node_offsets =
      sparse_grid_->base_node_offsets();
  const int num_blocks = sparse_grid_->num_blocks();
  [[maybe_unused]] const int num_threads = parallelize.num_threads();
#if defined(_OPENMP)
#pragma omp parallel for num_threads(num_threads)
#endif
  for (int b = 0; b < num_blocks; ++b) {
    bool need_new_pad = true;
    Pad<Vector3<T>> grid_x;
    Pad<GridData<T>> grid_data;
    const int particle_start = sentinel_particles[b];
    const int particle_end = sentinel_particles[b + 1];
    std::vector<int> indices;
    indices.reserve(lanes);
    int p = particle_start;
    while (p < particle_end) {
      int next_p = p + 1;
      while (base_node_offsets[next_p] == base_node_offsets[p] &&
             next_p - p < lanes && next_p < particle_end) {
        ++next_p;
      }
      if (need_new_pad) {
        grid_data = sparse_grid_->GetPadData(base_node_offsets[p]);
        grid_x = sparse_grid_->GetPadNodes(particles_->x[data_indices[p]]);
      }
      indices.clear();
      for (int i = p; i < next_p; ++i) {
        indices.push_back(data_indices[i]);
      }
      Vector3<SimdScalar<T>> v = Vector3<SimdScalar<T>>::Zero();
      Matrix3<SimdScalar<T>> B = Matrix3<SimdScalar<T>>::Zero();
      Vector3<SimdScalar<T>> x = Load(particles_->x, indices);
      const BsplineWeights<SimdScalar<T>> bspline =
          BsplineWeights<SimdScalar<T>>(x, sparse_grid_->dx());
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          for (int k = 0; k < 3; ++k) {
            const Vector3<T>& vi = grid_data[i][j][k].v;
            const Vector3<T>& xi = grid_x[i][j][k];
            const SimdScalar<T> w = bspline.weight(i, j, k);
            v += w * vi;
            B += (w * vi) * (xi - x).transpose();
          }
        }
      }
      const Matrix3<SimdScalar<T>> C = B * D_inverse_;
      x += v * dt_;
      Matrix3<SimdScalar<T>> F = Load(particles_->F, indices);
      F += C * dt_;
      Store(v, &particles_->v, indices);
      Store(x, &particles_->x, indices);
      Store(C, &particles_->C, indices);
      Store(F, &particles_->F, indices);

      need_new_pad = (next_p == particle_end) ||
                     base_node_offsets[next_p] != base_node_offsets[p];
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
