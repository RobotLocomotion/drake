#pragma once

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace particles {

/// An NDOF to 6DOF poses mapping system.
///
/// Stateless, this system applies a translating
/// matrix to its input to determine its outputs.
/// Such outputs are euler-representation poses,
/// thus fixing the port size to 6. However, its
/// input port size is inferred from the matrix
/// column count.
/// The system can be described in terms of its:
///
/// - Inputs:
///    - NDOF position (input indexes 0 to N-1).
///    - NDOF velocity (input indexes N to 2N-1).
/// - Outputs:
///    - 6DOF position (output indexes 0 to 5).
///    - 6DOF velocity (output indexes 6 to 11).
///
/// @tparam T must be a valid Eigen ScalarType.
///
/// @note
/// Instantiated templates for the following scalar types
/// @p T are provided:
/// - double
///
template <typename T>
class DegenerateEulerJoint : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DegenerateEulerJoint)

  /// A constructor that takes the input-output translating matrix
  /// for the joint.
  ///
  /// @param translator 6xN matrix to translate inputs to outputs.
  ///
  explicit DegenerateEulerJoint(const MatrixX<T>& translator);

 protected:
  void DoCalcOutput(const systems::Context<T>& context,
    systems::SystemOutput<T>* output) const override;

 private:
  const MatrixX<T> translator_;  ///< Matrix for input-output translation.
};

}  // namespace particles
}  // namespace examples
}  // namespace drake
