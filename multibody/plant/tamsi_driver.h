#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/plant/discrete_update_manager.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {


// Naming: TamsiBridge? other?
template <typename T>
class TamsiDriver {
 public:
  TamsiDriver(CompliantContactManager<T>* manager);

  const CompliantContactManager<T>& manager() const { return *manager_; }

  void CalcContactSolverResults(
      const systems::Context<T>&,
      contact_solvers::internal::ContactSolverResults<T>*) const;

 private:
  // The driver only has mutable access at construction time, when it can
  // declare additional state, cache entries, ports, etc. After construction,
  // the driver only has const access to the manager.
  const CompliantContactManager<T>* manager_{nullptr};
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake