#pragma once

#include <memory>

#include <Eigen/Geometry>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// Bicycle -- Implements a nonlinear bicycle model from Althff & Dolan, 2014
/// [1, 2].  The three-DOF model considers only a single wheel in the front and
/// rear to capture lateral and longitudinal dynamics of the bicycle.  The model
/// ignores roll and pitch dynamics.
///
/// The states of the model are: heading angle Ψ, yaw rate Ψ˙, slip angle at the
/// center of mass β, velocity magnitude (vector magnitude at the slip angle) v,
/// x-position at CG sx, and y-position at CG sy.
///
/// Inputs:
///  - Angle of the front wheels of the bicycle δ [rad].
///  - Force acting on the rigid body F_in [N].
///
/// Output:
///  - A 7-dimensional vector collecting the states of the bicycle.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::AutoDiffXd
/// - drake::symbolic::Expression
///
/// They are already available to link against in libdrakeAutomotive.
///
/// @ingroup automotive_systems
///
/// [1] M. Althoff, J.M. Dolan, Online Verification of Automated Road Vehicles
///     Using Reachability Analysis, IEEE Transactions on Robotics, 30(4), 2014,
///     pp. 903-908.  DOI: 10.1109/TRO.2014.2312453.
/// [2] M. Althoff and J. M. Dolan, Reachability computation of low-order
///     models for the safety verification of high-order road vehicle models,
///     in Proc. of the American Control Conference, 2012, pp. 3559–3566.
template <typename T>
class Bicycle : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Bicycle)

  Bicycle();
  ~Bicycle() override;

  /// Returns the steering input port in units of radians.
  const systems::InputPortDescriptor<T>& get_steering_input_port() const;

  /// Returns the applied powertrain force input port in units of N.
  const systems::InputPortDescriptor<T>& get_force_input_port() const;

  /// Returns the states.
  const systems::OutputPortDescriptor<T>& get_state_output_port() const;

  /// Sets the paramters in the @p context to default values.
  void SetDefaultParameters(const systems::LeafContext<T>& context,
                            systems::Parameters<T>* params) const override;

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

  // Specify the dimension of the state vector and of each input port.
  const int state_dimension_{6};
  const int steering_input_dimension_{1};
  const int force_input_dimension_{1};
};

}  // namespace automotive
}  // namespace drake
