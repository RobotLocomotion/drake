#pragma once

#include <Eigen/Dense>

#include "drake/common/autodiff.h"
#include "drake/solvers/constraint.h"

namespace drake {
namespace solvers {
namespace test {
/** Compare the result between Eval<double>() and Eval<AutoDiffXd>(). Also
compare the gradient in Eval<AutoDiffXd>() with a finite difference
approximation.
@param constraint The constraint object to test.
@param x_autodiff The point at which the Eval() methods are tested.
@param tol Tolerance on the comparison of the results from Eval<double>() and
Eval<AutoDiffXd>(). The tolerance on the comparison between the autodiff
gradient and the finite difference approximation is sqrt(tolerance) to account
for approximation error.
*/
void CheckConstraintEvalNonsymbolic(
    const Constraint& constraint,
    const Eigen::Ref<const AutoDiffVecXd>& x_autodiff, double tol);
}  // namespace test
}  // namespace solvers
}  // namespace drake
