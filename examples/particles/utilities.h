#pragma once

#include <memory>

#include "drake/common/eigen_types.h"
#include "drake/systems/primitives/matrix_gain.h"

namespace drake {
namespace examples {
namespace particles {

/// Makes an NDOF to 6DOF poses mapping system.
///
/// The given matrix @f$ \mathbf{M}_{\mathrm{6,N}} @f$ is
/// augmented to deal with position and velocity
/// state input @f$ [\mathbf{q} ; \mathbf{v}] @f$ as follows:
///
/// @f[ M^*_{\mathrm{12,2N}} =
///     \begin{bmatrix}
///       \mathbf{M}_{\mathrm{6,N}} & \mathbf{0}_{\mathrm{6,N}}
///    \\ \mathbf{0}_{\mathrm{6,N}} & \mathbf{M}_{\mathrm{6,N}}
///     \end{bmatrix}
/// @f]
///
/// @param[in] translator 6xN matrix to translate inputs to outputs, where
/// 0 < N < 6.
/// @return MatrixGain representing the joint.
/// @throws std::runtime_error whenever @p translator matrix implies
/// M DOF output where M is not 6, or an N DOF input where N is not a
/// positive non-zero integer that is less than 6.
///
/// @tparam T must be a valid Eigen ScalarType.
///
/// @note
/// Instantiated templates for the following scalar types
/// @p T are provided:
/// - double
///
template<typename T>
std::unique_ptr<typename systems::MatrixGain<T>>
MakeDegenerateEulerJoint(const MatrixX<T>& translator);

}  // namespace particles
}  // namespace examples
}  // namespace drake
