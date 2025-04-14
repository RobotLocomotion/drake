#pragma once

#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/math/fourth_order_tensor.h"
#include "drake/multibody/contact_solvers/sap/partial_permutation.h"
#include "drake/multibody/mpm/mpm_model.h"
#include "drake/multibody/mpm/particle_data.h"
#include "drake/multibody/mpm/sparse_grid.h"
#include "drake/multibody/mpm/transfer.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

/* The SolverState class works closely with MpmModel and stores the state of
 the MPM system during the optimization process (see MpmModel). It holds the
 grid velocity change, `dv`, and other quantities that depend on `dv`
 optimization process. */
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

  /* Resets the SolverState to be compatible with the given MpmModel. Makes this
   @pre  model.dx() == grid().dx().
   @post `this` SolverState is as if it was just constructed with the given
   MpmModel. */
  void Reset(const MpmModel<T, Grid>& model);

  /* Returns the number of degrees of freedom (DoFs) in the optimization
   problem, which is the 3 times the number of supported grid nodes. */
  int num_dofs() const {
    DRAKE_ASSERT(is_valid_);
    return dv_.size();
  }

  /* Returns the MPM grid used in the optimization process. */
  const Grid& grid() const {
    DRAKE_ASSERT(is_valid_);
    return *grid_;
  }

  /* Returns the change in the grid velocity dv = v - vn, ordered using grid
   node indices. */
  const VectorX<T>& dv() const {
    DRAKE_ASSERT(is_valid_);
    return dv_;
  }

  /* Returns the partial permutation that maps participating vertices/dofs to
   their indices in the SAP contact problem. */
  const contact_solvers::internal::VertexPartialPermutation& index_permutation()
      const {
    DRAKE_ASSERT(is_valid_);
    return index_permutation_;
  }

  const std::vector<Matrix3<T>>& F() const {
    DRAKE_ASSERT(is_valid_);
    return F_;
  }

  T elastic_energy() const {
    DRAKE_ASSERT(is_valid_);
    return elastic_energy_;
  }

  const std::vector<Matrix3<T>>& tau_volume() const {
    DRAKE_ASSERT(is_valid_);
    return tau_volume_;
  }

  /* Given the change in `dv`, `ddv`, and the MpmModel associated with `this`
   SolverState, updates the internal state of `this` SolverState.
   @pre ddv.size() == num_dofs().
   @pre `this` SolverState is constructed with the given `model` or is reset
   from the given `model`. */
  void UpdateState(const VectorX<T>& ddv, const MpmModel<T, Grid>& model);

  /* Uses `this` SolverState to update the given MpmModel to the next time
   step's state. Calls to this function invalidates the SolverState; the
   SolverState needs to be reset before being used again (see Reset()). */
  void UpdateModel(MpmModel<T, Grid>* model);

 private:
  Transfer<Grid> transfer_;
  copyable_unique_ptr<Grid> grid_;
  VectorX<T> dv_;
  contact_solvers::internal::VertexPartialPermutation index_permutation_;
  std::vector<Matrix3<T>> F_;
  std::vector<DeformationGradientDataVariant<T>> scratch_;
  T elastic_energy_{};
  std::vector<Matrix3<T>> tau_volume_;
  std::vector<math::internal::FourthOrderTensor<T>>
      volume_scaled_stress_derivatives_;
  bool is_valid_{false};
};

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
