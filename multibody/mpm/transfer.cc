#include "drake/multibody/mpm/transfer.h"

#include "drake/common/autodiff.h"
#include "drake/multibody/mpm/mock_sparse_grid.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

template <typename Grid>
Transfer<Grid>::Transfer(T dt, double dx) : dt_(dt), dx_(dx) {
  DRAKE_DEMAND(dt > 0);
  DRAKE_DEMAND(dx > 0);
  D_inverse_ = 4.0 / (dx_ * dx_);
  D_inverse_dt_ = D_inverse_ * dt_;
}

template <typename Grid>
void Transfer<Grid>::ParticleToGrid(const ParticleData<T>& particle,
                                    Grid* grid) {
  /* U == T when Grid == SparseGrid<T>.
     U == double when Grid == MockSparseGrid<T>. */
  using U = typename Grid::NodeScalarType;
  using PadNodeType = typename Grid::PadNodeType;
  using PadDataType = typename Grid::PadDataType;
  auto p2g_kernel = [this](int p_index, const PadNodeType& grid_x,
                           const ParticleData<T>& particle_data,
                           PadDataType* grid_data) {
    const T& m = particle_data.m()[p_index];
    const Vector3<T>& x = particle_data.x()[p_index];
    const Vector3<T>& v = particle_data.v()[p_index];
    const Matrix3<T>& C = particle_data.C()[p_index];
    const Matrix3<T>& tau_volume = particle_data.tau_volume()[p_index];
    const bool participating = particle_data.in_constraint()[p_index];
    const BsplineWeights<U> bspline =
        MakeBsplineWeights(x, static_cast<U>(dx_));
    for (int a = 0; a < 3; ++a) {
      for (int b = 0; b < 3; ++b) {
        for (int c = 0; c < 3; ++c) {
          const U& w_ip = bspline.weight(a, b, c);
          const Vector3<U>& xi = grid_x[a][b][c];
          /* The mass transfer is as described equation (126) in [Jiang et al.
           2016]. The momentum transfer is as described in equation (171) in
           [Jiang et al. 2016] with the force term equivalent to equation (18)
           in [Hu et al. 2018], but simplified. We sketch the proof of the
           equivalence here:

           The new grid momentum is given by mvᵢⁿ + fᵢⁿdt with mvᵢⁿ being the
           grid momentum from the current time step transferred from the
           particles. That is,

           mvᵢⁿ = Σₚ mₚ(vₚ + Cₚ(xᵢ - xₚ))wᵢₚ (equation 178 [Jiang et al. 2016])

           where wᵢₚ is the weight of the particle p to the grid node i. fᵢⁿdt
           is the change in momentum with the force given by fᵢⁿ = -∂E/∂xᵢ

           E = ∑ₚ VₚΨ(Fₚ) where Vₚ is the volume of the particle p in the
           reference configuration and Ψ is the strain energy density.

           Noting that
             Fₚ = (I + dtCₚ)Fₚⁿ (equation 17 [Hu et al. 2018])
             Cₚ = Bₚ * D⁻¹ (equation 173 and 178 [Jiang et al. 2016]), and
             Bₚ = ∑ᵢ wᵢₚ vᵢ(xᵢ − xₚ) (equation 176 [Jiang et al. 2016]),

           we compute -∂E/∂xᵢ and get

            fᵢⁿ = -∑ₚ Vₚ * Pₚ * Fₚⁿᵀ * D⁻¹ * (xᵢ − xₚ) * wᵢₚ

           with Pₚ = ∂Ψ/∂Fₚ. Noting that Pₚ * Fₚⁿᵀ is the Kirchhoff stress, we
           group Vₚ * Pₚ * Fₚⁿᵀ into a single term `tau_v0`. Rearranging terms
           reveals that mvᵢⁿ + fᵢⁿdt is given by the equation in the code below.
          */
          const T m_ip = m * w_ip;
          (*grid_data)[a][b][c].v +=
              m_ip * v + (m * C - D_inverse_dt_ * tau_volume) * (xi - x) * w_ip;
          (*grid_data)[a][b][c].m += m_ip;
          if (participating) (*grid_data)[a][b][c].index_or_flag.set_flag();
        }
      }
    }
  };
  grid->ApplyParticleToGridKernel(particle, p2g_kernel);
}

template <typename Grid>
void Transfer<Grid>::GridToParticle(const Grid& grid,
                                    ParticleData<T>* particle) {
  /* U == T when Grid == SparseGrid<T>.
     U == double when Grid == MockSparseGrid<T>. */
  using U = typename Grid::NodeScalarType;
  using PadNodeType = typename Grid::PadNodeType;
  using PadDataType = typename Grid::PadDataType;
  auto g2p_kernel = [this](int p_index, const PadNodeType& grid_x,
                           const PadDataType& grid_data,
                           ParticleData<T>* particle_data) {
    Vector3<T>& x = particle_data->mutable_x()[p_index];
    const BsplineWeights<U> bspline =
        MakeBsplineWeights(x, static_cast<U>(dx_));
    Vector3<T>& v = particle_data->mutable_v()[p_index];
    Matrix3<T>& C = particle_data->mutable_C()[p_index];
    /* Clear old particle data to prepare for accumulation. */
    v.setZero();
    C.setZero();
    Matrix3<T>& F = particle_data->mutable_F()[p_index];
    for (int a = 0; a < 3; ++a) {
      for (int b = 0; b < 3; ++b) {
        for (int c = 0; c < 3; ++c) {
          const Vector3<T>& vi = grid_data[a][b][c].v;
          const Vector3<U>& xi = grid_x[a][b][c];
          const U& w_ip = bspline.weight(a, b, c);
          v += w_ip * vi;
          /* Use C to store B (equation 176 [Jiang et al. 2016]). We multiply by
           D_inverse later on. */
          C += (w_ip * vi) * (xi - x).transpose();
        }
      }
    }
    x += v * dt_;
    C *= D_inverse_;
    F += C * dt_ * F;
  };
  grid.ApplyGridToParticleKernel(particle, g2p_kernel);
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
