#pragma once

#include <memory>

#include <Eigen/Geometry>

#include "drake/automotive/gen/bicycle_car_parameters.h"
#include "drake/automotive/gen/bicycle_car_state.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// BicycleCar implements a nonlinear rigid body bicycle model from Althoff &
/// Dolan (2014) [1].  The three-DOF model captures the rigid-body dynamics in
/// the lateral, longitudinal, and yaw directions but not in the roll and pitch
/// directions.  The model assumes a vehicle that has two wheels: one at the
/// front and one at the rear.  It has been demonstrated (e.g. [2]) that the
/// representation reasonably approximates the dynamics of a four-wheeled
/// vehicle; hence the model is useful as a simplified abstraction of car
/// dynamics.
///
/// The states of the model are:
///  - yaw angle Ψ [rad]
///  - yaw rate Ψ_dot [rad/s]
///  - slip angle at the center of mass β [rad]
///  - velocity magnitude (vector magnitude at the slip angle) vel [m/s]
///  - x-position of the center of mass sx [m]
///  - y-position of the center of mass sy [m]
///
/// N.B. "slip angle" (β) is the angle made between the body and the velocity
/// vector.  Thus, the velocity vector can be resolved into the body-relative
/// componenets `vx_body = cos(β)` and `vy_body = sin(β)`.  `β = 0` means the
/// velocity vector is pointing along the bicycle's longitudinal axis.
///
/// Inputs:
///  - Angle of the front wheel of the bicycle δ [rad]
///    (InputPortDescriptor getter: get_steering_input_port())
///  - Force acting on the rigid body F_in [N]
///    (InputPortDescriptor getter: get_force_input_port())
///
/// Output:
///  - A BicycleCarState containing the 6-dimensional state vector of the
///    bicycle.
///    (OutputPortDescriptor getter: get_state_output_port())
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::AutoDiffXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in libdrakeAutomotive.
///
/// [1] M. Althoff and J.M. Dolan, Online verification of automated road
///     vehicles using reachability analysis, IEEE Transactions on Robotics,
///     30(4), 2014, pp. 903-908.  DOI: 10.1109/TRO.2014.2312453.
///
/// [2] M. Althoff and J. M. Dolan, Reachability computation of low-order
///     models for the safety verification of high-order road vehicle models,
///     in Proc. of the American Control Conference, 2012, pp. 3559–3566.
///
/// @ingroup automotive_systems
template <typename T>
class BicycleCar : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BicycleCar)

  BicycleCar();
  ~BicycleCar() override;

  /// Returns a descriptor of the input port that contains the steering angle.
  const systems::InputPortDescriptor<T>& get_steering_input_port() const;

  /// Returns a descriptor of the input port that contains the applied
  /// powertrain force.
  const systems::InputPortDescriptor<T>& get_force_input_port() const;

  /// Returns a descriptor of the output port that contains the bicycle states.
  const systems::OutputPortDescriptor<T>& get_state_output_port() const;

  /// Sets the parameters in the @p context to default values.
  void SetDefaultParameters(const systems::LeafContext<T>& context,
                            systems::Parameters<T>* params) const override;

  // LeafSystem<T> overrides
  std::unique_ptr<systems::Parameters<T>> AllocateParameters() const override;

  // System<T> overrides.
  // Declare that the outputs are all algebraically isolated from the input.
  bool has_any_direct_feedthrough() const override { return false; }

 private:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  void ImplCalcTimeDerivatives(const BicycleCarParameters<T>& params,
                               const BicycleCarState<T>& state,
                               const systems::BasicVector<T>& steering,
                               const systems::BasicVector<T>& force,
                               BicycleCarState<T>* derivatives) const;

  int steering_input_port_{};
  int force_input_port_{};
  int state_output_port_{};
};

}  // namespace automotive
}  // namespace drake
