#include "drake/multibody/mpm/transfer.h"

#include "drake/common/autodiff.h"
#include "drake/multibody/mpm/mock_sparse_grid.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

template <typename Grid>
Transfer<Grid>::Transfer(T dt, Grid* grid, ParticleData<T>* particle_data)
    : dt_(dt), grid_(grid), particle_data_(particle_data) {
  DRAKE_DEMAND(dt > 0);
  DRAKE_DEMAND(grid != nullptr);
  DRAKE_DEMAND(particle_data != nullptr);
  grid_->Allocate(particle_data_->x());
  D_inverse_ = 4.0 / (grid_->dx() * grid_->dx());
  D_inverse_dt_ = D_inverse_ * dt_;
}

template <typename Grid>
void Transfer<Grid>::ParticleToGrid() {
  /* U == T when Grid == SparseGrid<T>.
     U == double when Grid == MockSparseGrid<T>. */
  using U = typename Grid::NodeScalarType;
  using PadNodeType = typename Grid::PadNodeType;
  using PadDataType = typename Grid::PadDataType;
  auto p2g_kernel = [this](int data_index, const PadNodeType& grid_x,
                           const ParticleData<T>& particle_data,
                           PadDataType* grid_data) {
    const T& m = particle_data.m()[data_index];
    const Vector3<T>& x = particle_data.x()[data_index];
    const Vector3<T>& v = particle_data.v()[data_index];
    const Matrix3<T>& C = particle_data.C()[data_index];
    const Matrix3<T>& tau_volume = particle_data.tau_volume()[data_index];
    const bool participating = particle_data.in_constraint()[data_index];
    const BsplineWeights<U> bspline =
        MakeBsplineWeights(x, static_cast<U>(grid_->dx()));
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) {
          const U& w = bspline.weight(i, j, k);
          const Vector3<U>& xi = grid_x[i][j][k];
          /* The mass transfer is as described equation (126) in [Jiang et al.
           2016]. The momentum transfer is as described in equation (171) in
           [Jiang et al. 2016] with the force term equivalent to equation (18)
           in [Hu et al. 2018], but simplified. We sketch the proof of the
           equivalence here:

           The new grid momentum is given by mvᵢⁿ + fᵢdt with mvᵢⁿ being the
           grid momentum from the current time step transferred from the
           particles. That is,

           mvᵢⁿ = Σₚ mₚvₚ + Cₚ(xᵢ - xₚ) wᵢₚ (equation 178 [Jiang et al. 2016])

           where wᵢₚ is the weight of the particle p to the grid node i. fᵢdt is
           the change in momentum with the force given by fᵢ = -∂E/∂xᵢ

           E = ∑ₚ VₚΨ(Fₚ) where Vₚ is the volume of the particle p in the
           reference configuration and Ψ is the strain energy density.

           Noting that
             Fₚ = (I + dtCₚ)Fₚⁿ (equation 17 [Hu et al. 2018])
             Cₚ = Bₚ * D⁻¹ (equation 173 [Jiang et al. 2016]), and
             Bₚ = ∑ᵢ wᵢₚ vᵢ(xᵢ − xₚ) (equation 176 [Jiang et al. 2016]),

           we compute -∂E/∂xᵢ and get

            fᵢ = -∑ₚ Vₚ * Pₚ * Fₚⁿᵀ * D⁻¹ * (xᵢ − xₚ) * wᵢₚ

           with Pₚ = ∂Ψ/∂Fₚ. Noting that Pₚ * Fₚⁿᵀ is the Kirchhoff stress, we
           group Vₚ * Pₚ * Fₚⁿᵀ into a single term `tau_v0`. Rearranging terms
           reveals that mvᵢⁿ + fᵢdt is given by the equation in the code below.
          */
          const T mi = m * w;
          (*grid_data)[i][j][k].v +=
              mi * v + (m * C - D_inverse_dt_ * tau_volume) * (xi - x) * w;
          (*grid_data)[i][j][k].m += mi;
          if (participating) (*grid_data)[i][j][k].index_or_flag.set_flag();
        }
      }
    }
  };
  grid_->ApplyParticleToGridKernel(*particle_data_, std::move(p2g_kernel));
}

template <typename Grid>
void Transfer<Grid>::GridToParticle() {
  /* U == T when Grid == SparseGrid<T>.
     U == double when Grid == MockSparseGrid<T>. */
  using U = typename Grid::NodeScalarType;
  using PadNodeType = typename Grid::PadNodeType;
  using PadDataType = typename Grid::PadDataType;
  auto g2p_kernel = [this](int data_index, const PadNodeType& grid_x,
                           const PadDataType& grid_data,
                           ParticleData<T>* particle_data) {
    Vector3<T>& x = particle_data->mutable_x()[data_index];
    const BsplineWeights<U> bspline =
        MakeBsplineWeights(x, static_cast<U>(grid_->dx()));
    Vector3<T>& v = particle_data->mutable_v()[data_index];
    Matrix3<T>& C = particle_data->mutable_C()[data_index];
    /* Clear old particle data to prepare for accumulation. */
    v.setZero();
    C.setZero();
    Matrix3<T>& F = particle_data->mutable_F()[data_index];
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) {
          const Vector3<T>& vi = grid_data[i][j][k].v;
          const Vector3<U>& xi = grid_x[i][j][k];
          const U& w = bspline.weight(i, j, k);
          v += w * vi;
          C += (w * vi) * (xi - x).transpose();
        }
      }
    }
    x += v * dt_;
    C *= D_inverse_;
    F += C * dt_ * F;
  };
  grid_->ApplyGridToParticleKernel(particle_data_, std::move(g2p_kernel));
}

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake

template class drake::multibody::mpm::internal::Transfer<
    drake::multibody::mpm::internal::SparseGrid<double>>;
template class drake::multibody::mpm::internal::Transfer<
    drake::multibody::mpm::internal::SparseGrid<float>>;
template class drake::multibody::mpm::internal::Transfer<
    drake::multibody::mpm::internal::MockSparseGrid<double>>;
template class drake::multibody::mpm::internal::Transfer<
    drake::multibody::mpm::internal::MockSparseGrid<drake::AutoDiffXd>>;
