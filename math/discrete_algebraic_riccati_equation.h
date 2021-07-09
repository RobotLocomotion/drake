#pragma once

#include <cmath>
#include <cstdlib>

#include <Eigen/Dense>

namespace drake {
namespace math {

/// Computes the unique stabilizing solution X to the discrete-time algebraic
/// Riccati equation:
///
/// @f[
/// A'XA - X - A'XB(B'XB+R)^{-1}B'XA + Q = 0
/// @f]
///
/// @throws std::exception if Q is not positive semi-definite.
/// @throws std::exception if R is not positive definite.
///
/// Based on the Schur Vector approach outlined in this paper:
/// "On the Numerical Solution of the Discrete-Time Algebraic Riccati Equation"
/// by Thrasyvoulos Pappas, Alan J. Laub, and Nils R. Sandell
///
Eigen::MatrixXd DiscreteAlgebraicRiccatiEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R);

/// DiscreteAlgebraicRiccatiEquation function
/// computes the unique stabilizing solution X to the discrete-time algebraic
/// Riccati equation:
/// \f[
/// A'XA - X - (A'XB + N)(B'XB + R)^{-1}(B'XA + N') + Q = 0
/// \f]
///
/// This is equivalent to:
/// \f[
/// A_{scr}'XA_{scr} - X - A_{scr}'XB(B'XB + R)^{-1}B'XA_{scr} + Q_{scr} = 0
/// \f]
///
/// where:
/// \f[
/// A_{scr} = A - BR^{-1}N' and Q_{scr} = Q - NR^{-1}N'
/// \f]
///
/// @throws std::runtime_error if Q is not positive semi-definite.
/// @throws std::runtime_error if R is not positive definite.
///
Eigen::MatrixXd DiscreteAlgebraicRiccatiEquation(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& B,
    const Eigen::Ref<const Eigen::MatrixXd>& Q,
    const Eigen::Ref<const Eigen::MatrixXd>& R,
    const Eigen::Ref<const Eigen::MatrixXd>& N);
}  // namespace math
}  // namespace drake

