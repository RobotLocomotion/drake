#pragma once

#include <Eigen/Dense>

#include "drake/common/autodiff.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace solvers {
namespace test {
/** Compare the result between Eval<double> and Eval<AutoDiffXd>. Also compare
the gradient in Eval<AutoDiffXd> with a numerical approximation. */
void CheckConstraintEvalNonsymbolic(const Constraint& constraint,
                         const Eigen::Ref<const AutoDiffVecXd>& x_autodiff,
                         double tol);
}  // namespace test
}  // namespace solvers
}  // namespace drake
