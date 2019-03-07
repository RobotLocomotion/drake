#pragma once

#include "drake/common/eigen_types.h"
#include "drake/common/symbolic.h"

// TODO(soonho-tri): Migrate the functions in drake/solvers:symbolic_extract to
// this file.

namespace drake {
namespace symbolic {

/// Decomposes @p expressions into @p M * @p vars.
///
/// @throws std::runtime_error if @p expressions is not linear in @p vars.
/// @pre M.rows() == expressions.rows() && M.cols() == vars.rows().
void DecomposeLinearExpressions(
    const Eigen::Ref<const VectorX<Expression>>& expressions,
    const Eigen::Ref<const VectorX<Variable>>& vars,
    EigenPtr<Eigen::MatrixXd> M);

/// Decomposes @p expressions into @p M * @p vars + @p v.
///
/// @throws std::runtime_error if @p expressions is not affine in @p vars.
/// @pre M.rows() == expressions.rows() && M.cols() == vars.rows().
/// @pre v.rows() == expressions.rows().
void DecomposeAffineExpressions(
    const Eigen::Ref<const VectorX<Expression>>& expressions,
    const Eigen::Ref<const VectorX<Variable>>& vars,
    EigenPtr<Eigen::MatrixXd> M, EigenPtr<Eigen::VectorXd> v);

}  // namespace symbolic
}  // namespace drake
