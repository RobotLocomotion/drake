#include "drake/multibody/mpm/solver_state.h"

#include "drake/multibody/mpm/mock_sparse_grid.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

template <typename T, typename Grid>
SolverState<T, Grid>::SolverState(const MpmModel<T, Grid>& model)
    : D_inverse_(4.0 / (model.dx() * model.dx())) {
  Reset(model);
}

template <typename T, typename Grid>
void SolverState<T, Grid>::Reset(const MpmModel<T, Grid>& model) {
  const ParticleData<T>& particle_data = model.particle_data();
  F_ = particle_data.F();
  deformation_gradient_data_ = particle_data.deformation_gradient_data();
  tau_volume_ = particle_data.tau_volume();
  volume_scaled_stress_derivatives_.resize(F_.size());
  particle_data.ComputePK1StressDerivatives(F_, &deformation_gradient_data_,
                                            &volume_scaled_stress_derivatives_);

  dv_.resize(model.num_dofs());
  dv_.setZero();
}

template <typename T, typename Grid>
void SolverState<T, Grid>::UpdateState(const VectorX<T>& ddv,
                                       const MpmModel<T, Grid>& model) {
  DRAKE_ASSERT(ddv.size() == num_dofs());
  constexpr int kDim = 3;
  const T dt = model.dt();
  const double dx = model.dx();
  dv_ += ddv;
  using U = typename Grid::NodeScalarType;
  using PadNodeType = typename Grid::PadNodeType;
  using PadDataType = typename Grid::PadDataType;
  auto update_F_kernel = [this, dt, dx](int p_index, const PadNodeType& grid_x,
                                        const PadDataType& grid_data,
                                        const ParticleData<T>& particle_data) {
    const Vector3<T>& x = particle_data.x()[p_index];
    Matrix3<T> C = Matrix3<T>::Zero();
    const BsplineWeights<U> bspline = MakeBsplineWeights(x, static_cast<U>(dx));
    for (int a = 0; a < 3; ++a) {
      for (int b = 0; b < 3; ++b) {
        for (int c = 0; c < 3; ++c) {
          const GridData<T>& data_i = grid_data[a][b][c];
          if (data_i.m == 0.0) continue;
          const Vector3<U>& xi = grid_x[a][b][c];
          DRAKE_ASSERT(data_i.index_or_flag.is_index());
          const int grid_index = data_i.index_or_flag.index();
          const Vector3<T>& vi =
              data_i.v + dv_.template segment<kDim>(kDim * grid_index);
          const U w = bspline.weight(a, b, c);
          C += (w * vi) * (xi - x).transpose();
        }
      }
    }
    C *= D_inverse_;
    const Matrix3<T>& F0 = particle_data.F()[p_index];
    F_[p_index] = F0 + C * dt * F0;
  };
  const auto& particle_data = model.particle_data();
  model.grid().IterateParticleAndGrid(particle_data, update_F_kernel);

  // TODO(xuchenhan-tri): This can be grouped into a single function to reduce
  // redundant computation.
  /* Then update energy, stress, and stress derivatives. */
  elastic_energy_ =
      particle_data.ComputeTotalEnergy(F_, &deformation_gradient_data_);
  particle_data.ComputeKirchhoffStress(F_, &deformation_gradient_data_,
                                       &tau_volume_);
  particle_data.ComputePK1StressDerivatives(F_, &deformation_gradient_data_,
                                            &volume_scaled_stress_derivatives_);
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
