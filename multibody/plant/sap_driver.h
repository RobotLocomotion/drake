#pragma once

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/context.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/contact_solvers/contact_solver_results.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"

namespace drake {
namespace multibody {

// Forward declaration.  
template <typename>
class MultibodyPlant;

namespace internal {    

// Forward declaration.
template <typename>
class CompliantContactManager;

template <typename T>
struct ContactProblemCache {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ContactProblemCache);
  explicit ContactProblemCache(double time_step) {
    sap_problem =
        std::make_unique<contact_solvers::internal::SapContactProblem<T>>(
            time_step);
  }
  copyable_unique_ptr<contact_solvers::internal::SapContactProblem<T>>
      sap_problem;
  std::vector<math::RotationMatrix<T>> R_WC;
};

// Naming: SapBridge?  other?
template <typename T>
class SapDriver {
 public:
  SapDriver(CompliantContactManager<T>* manager);

  const CompliantContactManager<T>& manager() const { return *manager_; }

  const MultibodyPlant<T>& plant() const { return manager().plant(); }

  void DeclareCacheEntries(CompliantContactManager<T>* manager);

  void CalcContactSolverResults(
      const systems::Context<T>&,
      contact_solvers::internal::ContactSolverResults<T>*) const;

  // TODO: make private? only needed by the driver?
  const ContactProblemCache<T>& EvalContactProblemCache(
      const systems::Context<T>& context) const;

 private:  
  // Computes the necessary data to describe the SAP contact problem. Additional
  // information such as the orientation of each contact frame in the world is
  // also computed here so that it can be used at a later stage to compute
  // contact results.
  // All contact constraints are added before any other constraint types. This
  // manager assumes this ordering of the constraints in order to extract
  // contact impulses for reporting contact results.
  void CalcContactProblemCache(const systems::Context<T>& context,
                               ContactProblemCache<T>* cache) const;

  // The driver only has mutable access at construction time, when it can
  // declare additional state, cache entries, ports, etc. After construction,
  // the driver only has const access to the manager.
  const CompliantContactManager<T>* manager_{nullptr};

  systems::CacheIndex contact_problem_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake