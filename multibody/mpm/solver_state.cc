#include "drake/multibody/mpm/solver_state.h"

#include "drake/multibody/mpm/mock_sparse_grid.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

template <typename T, typename Grid>
SolverState<T, Grid>::SolverState(const MpmModel<T, Grid>& model)
    : transfer_(model.dt(), model.dx()),
      grid_(std::make_unique<Grid>(model.dx())) {
  Reset(model);
}

template <typename T, typename Grid>
void SolverState<T, Grid>::Reset(const MpmModel<T, Grid>& model) {
  DRAKE_DEMAND(model.dx() == grid_->dx());
  const ParticleData<T>& particle_data = model.particle_data();
  grid_->Allocate(particle_data.x());
  transfer_.ParticleToGrid(particle_data, grid_.get_mutable());
  int num_dofs = 0;
  /* Convert from grid momentum to grid velocity and count the number of dofs
   in the system. */
  grid_->IterateGrid([&](GridData<T>* node) {
    if (node->m > 0.0) {
      node->v /= node->m;
      num_dofs += 3;
    }
  });
  F_ = particle_data.F();
  scratch_ = particle_data.deformation_gradient_data();
  tau_volume_ = particle_data.tau_volume();
  volume_scaled_stress_derivatives_.resize(F_.size());
  particle_data.ComputePK1StressDerivatives(F_, &scratch_,
                                            &volume_scaled_stress_derivatives_);

  index_permutation_ = grid_->SetNodeIndices();
  dv_.resize(num_dofs);
  dv_.setZero();

  is_valid_ = true;
}

template <typename T, typename Grid>
void SolverState<T, Grid>::UpdateState(const VectorX<T>& ddv,
                                       const MpmModel<T, Grid>& model) {
  DRAKE_ASSERT(ddv.size() == num_dofs());
  DRAKE_ASSERT(model.dx() == grid().dx());
  DRAKE_ASSERT(is_valid_);
  constexpr int kDim = 3;
  const T dt = model.dt();
  dv_ += ddv;
  using U = typename Grid::NodeScalarType;
  using PadNodeType = typename Grid::PadNodeType;
  using PadDataType = typename Grid::PadDataType;
  auto update_F_kernel = [this, dt](int p_index, const PadNodeType& grid_x,
                                    const PadDataType& grid_data,
                                    ParticleData<T>* particle_data) {
    const ParticleData<T>& const_particle_data = *particle_data;
    const Vector3<T>& x = const_particle_data.x()[p_index];
    Matrix3<T> C = Matrix3<T>::Zero();
    const BsplineWeights<U> bspline =
        MakeBsplineWeights(x, static_cast<U>(grid_->dx()));
    for (int a = 0; a < 3; ++a) {
      for (int b = 0; b < 3; ++b) {
        for (int c = 0; c < 3; ++c) {
          if (grid_data[a][b][c].m == 0.0) continue;
          DRAKE_ASSERT(grid_data[a][b][c].index_or_flag.is_index());
          const int grid_index = grid_data[a][b][c].index_or_flag.index();
          const Vector3<T>& vi = grid_data[a][b][c].v +
                                 dv_.template segment<kDim>(kDim * grid_index);
          const Vector3<U>& xi = grid_x[a][b][c];
          const U w = bspline.weight(a, b, c);
          C += (w * vi) * (xi - x).transpose();
        }
      }
    }
    C *= transfer_.D_inverse();
    const Matrix3<T>& F0 = const_particle_data.F()[p_index];
    F_[p_index] = F0 + C * dt * F0;
  };
  const auto& particle_data = model.particle_data();
  /* We const cast the particle data to satisfy the signature of G2P, but we
   don't actually modify the content of the particle data as seen in the
   kernel above; instead, we only make use of the side-effect of the transfer.
  */
  auto& mutable_particle_data = const_cast<ParticleData<T>&>(particle_data);
  grid_->ApplyGridToParticleKernel(&mutable_particle_data, update_F_kernel);

  /* Then update stress and stress derivatives. */
  particle_data.ComputeKirchhoffStress(F_, &scratch_, &tau_volume_);
  particle_data.ComputePK1StressDerivatives(F_, &scratch_,
                                            &volume_scaled_stress_derivatives_);
}

template <typename T, typename Grid>
void SolverState<T, Grid>::UpdateModel(MpmModel<T, Grid>* model) {
  DRAKE_ASSERT(model != nullptr);
  constexpr int kDim = 3;
  auto update_grid_velocity = [&](GridData<T>* node) {
    if (node->m > 0.0) {
      DRAKE_ASSERT(node->index_or_flag.is_index());
      node->v += dv_.template segment<kDim>(kDim * node->index_or_flag.index());
    }
  };
  grid_->IterateGrid(update_grid_velocity);
  ParticleData<T>& particle_data = model->mutable_particle_data();
  transfer_.GridToParticle(*grid_, &particle_data);
  particle_data.ComputeKirchhoffStress(
      particle_data.F(), &particle_data.mutable_deformation_gradient_data(),
      &particle_data.mutable_tau_volume());

  is_valid_ = false;
}

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake

template class drake::multibody::mpm::internal::SolverState<double>;
template class drake::multibody::mpm::internal::SolverState<float>;
template class drake::multibody::mpm::internal::SolverState<
    drake::AutoDiffXd,
    drake::multibody::mpm::internal::MockSparseGrid<drake::AutoDiffXd>>;
