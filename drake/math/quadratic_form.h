#pragma once

#include <Eigen/Core>

namespace drake {
namespace math {
/**
 * Rewrite a quadratic form xᵀQx + bᵀx + c to
 * (Rx+d)ᵀ(Rx+d)
 * where
 * RᵀR = Q
 * Rᵀd = b / 2
 * Notice that this decomposition is not unique. For example, with any
 * permutation matrix P, we can define
 * R₁ = P*R
 * d₁ = P*d
 * Then (R₁*x+d₁)ᵀ(R₁*x+d₁) gives the same quadratic form.
 */
std::pair<Eigen::MatrixXd, Eigen::MatrixXd>
DecomposePositiveQuadraticForm(const Eigen::Ref<const Eigen::MatrixXd>& Q,
                               const Eigen::Ref<const Eigen::VectorXd>& b,
                               double c);
}  // namespace math
}  // namespace drake
