#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/contact_solver.h"
#include "drake/multibody/fixed_fem/dev/deformable_model.h"
#include "drake/multibody/fixed_fem/dev/fem_solver.h"
#include "drake/multibody/plant/discrete_update_manager.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** %DeformableRigidManager implements the interface in DiscreteUpdateManager
 and performs discrete update for deformable bodies and rigid bodies in a
 two-way coupled fashion.
 @tparam_nonsymbolic_scalar. */
template <typename T>
class DeformableRigidManager final
    : public multibody::internal::DiscreteUpdateManager<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DeformableRigidManager)

  /** Constructs a %DeformableRigidManager that solves contact with the given
   contact solver. */
  DeformableRigidManager(
      std::unique_ptr<multibody::contact_solvers::internal::ContactSolver<T>>
          contact_solver)
      : contact_solver_(std::move(contact_solver)) {}

 private:
  /* Implements DiscreteUpdateManager::ExtractModelInfo(). Verifies that
   exactly one DeformableModel is registered in the owning plant and sets up FEM
   solvers for deformable bodies. */
  void ExtractModelInfo() final;

  /* Extracts the necessary information from the `deformable_model` and sets up
   the FEM solvers that solves the model. */
  void SetFemSolvers(const DeformableModel<T>& deformable_model);

  // TODO(xuchenhan-tri): Implement this.
  void DoCalcContactSolverResults(
      const systems::Context<T>&,
      contact_solvers::internal::ContactSolverResults<T>*) const final {
    throw std::logic_error(
        "CalcContactSolverResults() hasn't be implemented for "
        "DeformableRigidManager yet.");
  }

  // TODO(xuchenhan-tri): Implement this once AccelerationKinematicsCache also
  //  caches acceleration for deformable dofs.
  void DoCalcAccelerationKinematicsCache(
      const systems::Context<T>&,
      multibody::internal::AccelerationKinematicsCache<T>*) const final {
    throw std::logic_error(
        "DoCalcAcclerationKinematicsCache() hasn't be implemented for "
        "DeformableRigidManager yet.");
  }

  void DoCalcDiscreteValues(const systems::Context<T>& context,
                            systems::DiscreteValues<T>* updates) const final;

  /* The discrete state indexes for all deformable bodies. */
  std::vector<systems::DiscreteStateIndex> discrete_state_indexes_{};
  /* Scratch space for the previous timestep and contact-free FEM states to
   avoid repeated allocation. */
  mutable std::vector<std::unique_ptr<FemStateBase<T>>> state0s_{};
  mutable std::vector<std::unique_ptr<FemStateBase<T>>> state_stars_{};
  /* Solvers for all deformable bodies. */
  std::vector<std::unique_ptr<FemSolver<T>>> fem_solvers_{};
  std::unique_ptr<multibody::contact_solvers::internal::ContactSolver<T>>
      contact_solver_{nullptr};
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fixed_fem::DeformableRigidManager);
