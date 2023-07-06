#pragma once

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* Use automatic differentiation to validate the implementation of analytical
 gradients implemented by constraint `c`, evaluated at constraint velocity `vc`.
 N.B. This method uses arbitrary values of time step and Delassus estimation to
 make an initial constraint data with c->MakeData(). If the validation requires
 specific values of time step and Delassus estimation, consider using the
 alternative signature of this method passing an externally created data. */
void ValidateConstraintGradients(const SapConstraint<AutoDiffXd>& c,
                                 const Eigen::VectorXd& vc);

/* Use automatic differentiation to validate the implementation of analytical
 gradients implemented by constraint `c`, evaluated at `data`. */
void ValidateConstraintGradients(const SapConstraint<AutoDiffXd>& c,
                                 const AbstractValue& data);

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
