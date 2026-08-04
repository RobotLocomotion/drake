#pragma once

#include <vector>

#include "drake/math/fourth_order_tensor.h"
#include "drake/multibody/mpm/mpm_model.h"
#include "drake/multibody/mpm/particle_data.h"
#include "drake/multibody/mpm/sparse_grid.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

/* The SolverState class works closely with MpmModel and stores the state of
 the MPM system during the optimization process (see MpmModel). It maintains
 the grid velocity update, dv—which serves as the independent variable during
 optimization—along with other quantities that are functions of dv. */
template <typename T, typename Grid = SparseGrid<T>>
class SolverState {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SolverState)

  /* Constructs a SolverState object that's compatible with the given MpmModel.
   The grid mass and velocity are transferred from the particles in MpmModel.
   The difference between the grid velocity and the previous grid velocity,
   `dv`, is initialized to zero. The deformation gradient, stress, and stress
   derivatives are also initialized to be their values at the previous time
   step. */
  explicit SolverState(const MpmModel<T, Grid>& model);

  /* Resets the SolverState to be compatible with the given MpmModel.
   @post `this` SolverState is as if it was just constructed with the given
   MpmModel. */
  void Reset(const MpmModel<T, Grid>& model);

  /* Returns the number of degrees of freedom (DoFs) in the optimization
   problem, which is the 3 times the number of supported grid nodes. */
  int num_dofs() const { return dv_.size(); }

  /* Returns the change in the grid velocity dv = v - vⁿ, ordered using grid
   node indices. This is the independent variable in the optimization process.
  */
  const VectorX<T>& dv() const { return dv_; }

  const std::vector<Matrix3<T>>& F() const { return F_; }

  T elastic_energy() const { return elastic_energy_; }

  const std::vector<Matrix3<T>>& tau_volume() const { return tau_volume_; }

  /* Given the change in `dv`, `ddv`, and the MpmModel associated with `this`
   SolverState, updates the internal state of `this` SolverState.
   @pre ddv.size() == num_dofs().
   @pre `this` SolverState is constructed with the given `model` or is reset
   from the given `model`. */
  void UpdateState(const VectorX<T>& ddv, const MpmModel<T, Grid>& model);

 private:
  T D_inverse_{};
  VectorX<T> dv_;
  std::vector<Matrix3<T>> F_;
  T elastic_energy_{};
  std::vector<DeformationGradientDataVariant<T>> deformation_gradient_data_;
  std::vector<Matrix3<T>> tau_volume_;
  std::vector<math::internal::FourthOrderTensor<T>>
      volume_scaled_stress_derivatives_;
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
