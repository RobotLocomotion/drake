#pragma once

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"

// TODO(soonho-tri): Migrate the functions in drake/solvers:symbolic_extract to
// this file.

namespace drake {
namespace symbolic {

/// Decomposes @p expressions into @p M1 * @p vars1 + @p M2 * @p vars2.
///
/// @throws runtime_error if @p expressions is not linear in @p vars1 and @p
/// vars2.
void DecomposeLinearExpressions(
    const Eigen::Ref<const VectorX<Expression>>& expressions,
    const Eigen::Ref<const VectorX<Variable>> vars1,
    const Eigen::Ref<const VectorX<Variable>> vars2, Eigen::MatrixXd* const M1,
    Eigen::MatrixXd* const M2);

/// Decomposes @p expressions into @p M1 * @p vars1 + @p M2 * @p vars2 + v.
///
/// @throws runtime_error if @p expressions is not affine in @p vars1` and @p
/// vars2.
void DecomposeAffineExpressions(
    const Eigen::Ref<const VectorX<Expression>>& expressions,
    const Eigen::Ref<const VectorX<Variable>> vars1,
    const Eigen::Ref<const VectorX<Variable>> vars2, Eigen::MatrixXd* const M1,
    Eigen::MatrixXd* const M2, Eigen::VectorXd* const v);

}  // namespace symbolic
}  // namespace drake
