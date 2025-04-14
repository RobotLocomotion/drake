#pragma once

#include <vector>

#include "drake/multibody/mpm/particle_data.h"
#include "drake/multibody/mpm/sparse_grid.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

template <typename T, typename Grid>
class SolverState;

/* MpmModel provides information about an implicit (backward Euler) MPM system.
 Given particle states, it models the objective function as a function of the
 grid velocity change, `dv`. It works in conjunction with a Solver that
 minimizes the objective function by solving non-linear system of equations and
 a SolverState that holds the state that changes during the optimization
 process.

 Specifically, the cost function is given by

  E(v) = ½ (v − vⁿ)ᵀ M (v − vⁿ) + ∑ₚ Ψ(Fₚ(v; Fₚⁿ)) ⋅ volₚ

 Here, v is the grid velocity, vⁿ is the grid velocity at the previous time
 step, M is the lumped mass matrix, Fₚ is the deformation gradient of the p-th
 particle, volₚ is the volume of the p-th particle in the reference
 configuration, Ψ is the strain energy density function. Note in particular that
 the deformation gradient Fₚ(v; Fₚⁿ) is a function of the grid velocity v and
 the previous deformation gradient Fₚⁿ, but Fₚⁿ is fixed in the optimization
 process.

 Differentiating E(v) with respect to v, we get the gradient(or "residual")
 vector

  M(v - vⁿ) - f(v; Fₚⁿ)dt,

 where f(v; Fₚⁿ) is the grid force that depends implicitly on v. In practice, we
 work with dv = v - vⁿ because it's slightly more convenient.

 MpmModel stores the particle data evaluated at time step tⁿ (e.g Fₚⁿ) and those
 that are constant during the optimization process (e.g. volₚ). The state that
 changes during the optimization process, such as dv, Fₚ(v; Fₚⁿ), and stress
 [dΨ/dFₚ(Fₚ(v; Fₚⁿ))] are stored in SolverState. MpmModel provides functions to
 compute the cost, gradient(residual), and Hessian of the objective function by
 evaluating them at a given solver state. Once the solver state that minimizes
 the objective has been found, the state can be used to update the MpmModel (see
 SolverState::UpdateModel). */
template <typename T, typename Grid = SparseGrid<T>>
class MpmModel {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MpmModel);

  /* Creates a MpmModel given the current state of particles.
   @param[in] dt         The time step used in this MpmModel (in seconds).
   @param[in] dx         The grid spacing (in meters).
   @param[in] particles  The particle data.
   @pre dt > 0 and dx > 0. */
  MpmModel(T dt, double dx, ParticleData<T> particles);

  T dt() const { return dt_; }

  double dx() const { return dx_; }

  int num_particles() const { return particle_data_.num_particles(); }

  /* Computes the energy at the given solver state using the formula
   E(v) = ½ (v − vⁿ)ᵀ M (v − vⁿ) + ∑ₚ Ψ(Fₚ(v)) ⋅ volₚ. */
  T CalcCost(const SolverState<T, Grid>& solver_state) const;

  /* Computes the residual vector b = Mdv - f(v)dt. */
  void CalcResidual(const SolverState<T, Grid>& solver_state,
                    VectorX<T>* result) const;

  const ParticleData<T>& particle_data() const { return particle_data_; }
  ParticleData<T>& mutable_particle_data() { return particle_data_; }

 private:
  T dt_{};
  double dx_{};
  ParticleData<T> particle_data_{};
  T D_inverse_{};
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
