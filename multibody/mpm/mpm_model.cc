#include "drake/multibody/mpm/mpm_model.h"

#include "drake/multibody/mpm/mock_sparse_grid.h"
#include "drake/multibody/mpm/solver_state.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

template <typename T, typename Grid>
MpmModel<T, Grid>::MpmModel(T dt, double dx, ParticleData<T> particle_data)
    : dt_(dt),
      dx_(dx),
      particle_data_(std::move(particle_data)),
      D_inverse_(4.0 / (dx_ * dx_)) {
  DRAKE_DEMAND(dt > 0);
  DRAKE_DEMAND(dx > 0);
}

template <typename T, typename Grid>
T MpmModel<T, Grid>::CalcCost(const SolverState<T, Grid>& solver_state) const {
  const int kDim = 3;
  const int num_dofs = solver_state.num_dofs();
  const VectorX<T>& dv = solver_state.dv();
  /* Potential energy from the particles. */
  T total_energy = solver_state.elastic_energy();
  /* The 1/2*dv*M*dv term. */
  solver_state.grid().IterateGrid([&](const GridData<T>& node) {
    if (node.m > 0.0) {
      DRAKE_ASSERT(node.index_or_flag.is_index());
      const int index = node.index_or_flag.index();
      DRAKE_ASSERT(index >= 0 && index * kDim < num_dofs);
      total_energy +=
          0.5 * node.m * dv.template segment<kDim>(index * kDim).squaredNorm();
    }
  });
  return total_energy;
}

template <typename T, typename Grid>
void MpmModel<T, Grid>::CalcResidual(const SolverState<T, Grid>& solver_state,
                                     VectorX<T>* result) const {
  DRAKE_ASSERT(result != nullptr);
  result->resize(solver_state.num_dofs());
  result->setZero();
  constexpr int kDim = 3;
  const Grid& grid = solver_state.grid();

  using U = typename Grid::NodeScalarType;
  using PadNodeType = typename Grid::PadNodeType;
  using PadDataType = typename Grid::PadDataType;
  /* Splat temporary impulses to the grid data scratch and collect them into the
   result. */
  auto splat_force_kernel = [&](int p_index, const PadNodeType& grid_x,
                                const PadDataType& grid_data,
                                const ParticleData<T>& particle_data) {
    const Vector3<T>& xp = particle_data.x()[p_index];
    const BsplineWeights<U> bspline =
        MakeBsplineWeights(xp, static_cast<U>(dx_));
    for (int a = 0; a < 3; ++a) {
      for (int b = 0; b < 3; ++b) {
        for (int c = 0; c < 3; ++c) {
          const U& w_ip = bspline.weight(a, b, c);
          const Vector3<U>& xi = grid_x[a][b][c];
          /* Note that we take the stress from the scratch data here instead
           directly from the particle because we are in the Newton loop. */
          const Matrix3<T>& tau_volume = solver_state.tau_volume()[p_index];
          /* For the elastic force from particles, we compute -∂E/∂xᵢ and
           get

             fᵢ = -∑ₚ Vₚ * Pₚ * Fₚⁿᵀ * D⁻¹ * (xᵢ − xₚ) * wᵢₚ

           with Pₚ = ∂Ψ/∂Fₚ. Noting that Pₚ * Fₚⁿᵀ is the Kirchhoff stress,
           we group Vₚ * Pₚ * Fₚⁿᵀ into a single term `tau_v0`. Rearranging
           terms reveals that - fᵢdt is given by the equation in the code
           below. */
          const int grid_index = grid_data[a][b][c].index_or_flag.index();
          result->template segment<kDim>(kDim * grid_index) +=
              tau_volume * (xi - xp) * D_inverse_ * dt_ * w_ip;
        }
      }
    }
  };
  grid.IterateParticleAndGrid(particle_data_, splat_force_kernel);
  /* Collect from the scratch data, add in the M * dv term, and clear the
   scratch data. */
  const VectorX<T>& dv = solver_state.dv();
  grid.IterateGrid([&](const GridData<T>& node) {
    if (node.m > 0.0) {
      DRAKE_ASSERT(node.index_or_flag.is_index());
      const int index = node.index_or_flag.index();
      DRAKE_ASSERT(index >= 0 && index < result->size() / 3);
      result->template segment<kDim>(index * kDim) +=
          node.m * dv.template segment<kDim>(index * kDim);
    }
  });
}

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake

template class drake::multibody::mpm::internal::MpmModel<double>;
template class drake::multibody::mpm::internal::MpmModel<float>;
template class drake::multibody::mpm::internal::MpmModel<
    drake::AutoDiffXd,
    drake::multibody::mpm::internal::MockSparseGrid<drake::AutoDiffXd>>;
