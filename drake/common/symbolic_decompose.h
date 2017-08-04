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
///
/// @pre @p state_vars and @p input_vars should not overlap.
/// @pre M1.rows() == expressions.rows() && M1.cols() == vars1.rows().
/// @pre M2.rows() == expressions.rows() && M2.cols() == vars2.rows().
void DecomposeLinearExpressions(
    const Eigen::Ref<const VectorX<Expression>>& expressions,
    const Eigen::Ref<const VectorX<Variable>>& vars1,
    const Eigen::Ref<const VectorX<Variable>>& vars2,
    Eigen::Ref<Eigen::MatrixXd> M1, Eigen::Ref<Eigen::MatrixXd> M2);

/// Decomposes @p expressions into @p M1 * @p vars1 + @p M2 * @p vars2 + v.
///
/// @throws runtime_error if @p expressions is not affine in @p vars1` and @p
/// vars2.
///
/// @pre @p state_vars and @p input_vars should not overlap.
/// @pre M1.rows() == expressions.rows() && M1.cols() == vars1.rows().
/// @pre M2.rows() == expressions.rows() && M2.cols() == vars2.rows().
/// @pre v.rows() == expressions.rows().
void DecomposeAffineExpressions(
    const Eigen::Ref<const VectorX<Expression>>& expressions,
    const Eigen::Ref<const VectorX<Variable>>& vars1,
    const Eigen::Ref<const VectorX<Variable>>& vars2,
    Eigen::Ref<Eigen::MatrixXd> M1, Eigen::Ref<Eigen::MatrixXd> M2,
    Eigen::Ref<Eigen::VectorXd> v);

}  // namespace symbolic
}  // namespace drake
