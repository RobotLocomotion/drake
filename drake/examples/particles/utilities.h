#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
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
///       \mathbf{M}_{\mathrm{6,N}} & \mathbf{0}_{\mathrm{6,N}} \\\
///       \mathbf{0}_{\mathrm{6,N}} & \mathbf{M}_{\mathrm{6,N}}
///     \end{bmatrix}
/// @f]
///
/// @param[in] translator 6xN matrix to translate inputs to outputs.
/// @return MatrixGain system representing the joint.
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
