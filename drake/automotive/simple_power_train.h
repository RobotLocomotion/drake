#pragma once

#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace automotive {

/// SimplePowerTrain -- A simple power-train model modeled as a linear
/// first-order lag, with input taken as the throttle setting and the output
/// taken as the force applied to the wheels of the vehicle.
///
/// Input:
///  - A unitless scalar value representing the throttle input to the power
///    system.
///
/// Output:
///  - The force transmitted from the wheels to the rigid body [N].
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_systems
template <typename T>
class SimplePowerTrain : public systems::LinearSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimplePowerTrain);

  /// Construct a simple power train model, specified via a fixed time-constant
  /// and scalar gain.  The inputs are as follows:
  /// @p time_constant is the powertrain time constant [1/s].
  /// @p gain is the gain from throttle input to force output [N].
  SimplePowerTrain(const double& time_constant, const double& gain)
      : systems::LinearSystem<T>(make_singleton_matrix(-1. / time_constant),
                                 make_singleton_matrix(gain),
                                 make_singleton_matrix(1. / time_constant),
                                 make_singleton_matrix(0.)) {}
  ~SimplePowerTrain() override = default;

  const systems::InputPortDescriptor<T>& get_throttle_input_port() const {
    return systems::System<T>::get_input_port(0);
  }

  const systems::OutputPortDescriptor<T>& get_force_output_port() const {
    return systems::System<T>::get_output_port(0);
  }

 private:
  static Eigen::MatrixXd make_singleton_matrix(const double& value) {
    Eigen::MatrixXd matrix(1, 1);
    matrix << value;
    return matrix;
  }
};

}  // namespace automotive
}  // namespace drake
