#pragma once

#include <memory>

#include <Eigen/Geometry>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace automotive {

/// Bicycle -- Implements a nonlinear bicycle model from Althff & Dolan, 2014
/// [1].  The 6-DOF model considers a lumped representation of the front and
/// rear wheels, ignoring roll and pitch dynamics.  Although not included here,
/// it is shown that addition of bounded uncertainties can allow this model to
/// capture possible behaviors of higher-order car models [2].
///
/// The states of the model are: heading angle Ψ, yaw rate Ψ˙, slip angle at the
/// center of mass β, velocity magnitude (vector magnitude at the slip angle) v,
/// x-position at CG sx, and y-position at CG sy.
///
/// TODO(jadecastro) ********** Fill in the details of the model here **********
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
/// [2]  M. Althoff and J. M. Dolan, Reachability computation of low-order
///      models for the safety verification of high-order road vehicle models,
///      in Proc. of the American Control Conference, 2012, pp. 3559–3566.
///
/// Inputs:
///   0: @p delta angle of the front wheel [rad].
///   1: @p F_in force input to the front wheel [N].
///
/// Outputs:
///   0: State vector containing linear/angular positions in the global
///      reference frame and linear/angular velocities in the local reference
///      frame.
///      (@p X [m], @p Y [m], @p theta [rad],
///       @p u [m/s], @p v [m/s], @p r [rad/s])

template <typename T>
class Bicycle : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Bicycle)

  Bicycle();
  ~Bicycle() override;

  /// Returns the steering input port in units of radians.
  const systems::InputPortDescriptor<T>& get_steering_input_port() const;

  /// Returns the applied powertrain force input port in units of N.
  const systems::InputPortDescriptor<T>& get_pt_input_port() const;

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
  const int kStateDimension{6};
  const int kSteeringInputDimension{1};
  const int kPTInputDimension{1};
};

}  // namespace automotive
}  // namespace drake
