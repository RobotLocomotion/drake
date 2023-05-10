#pragma once

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

void ValidateConstraintGradients(const SapConstraint<AutoDiffXd>& c,
                                 const AbstractValue& data);

void ValidateConstraintGradients(const SapConstraint<AutoDiffXd>& c,
                                 const Eigen::VectorXd& vc);

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
