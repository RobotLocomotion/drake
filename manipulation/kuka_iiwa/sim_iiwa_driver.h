#pragma once

#include <optional>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"  // IiwaControlMode
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {
namespace internal {

/* SimIiwaDriver simulates the IIWA control and status interface using a
MultibodyPlant.

@system
name: SimIiwaDriver
input_ports:
- <b style="color:orange">state</b>
- <b style="color:orange">generalized_contact_forces</b>
- position (in kPositionOnly or kPositionAndTorque mode)
- torque (in kTorqueOnly or kPositionAndTorque mode)
output_ports:
- <b style="color:orange">actuation</b>
- position_commanded
- position_measured
- velocity_estimated
- state_estimated
- torque_commanded
- torque_measured
- torque_external
@endsystem

Ports shown in <b style="color:orange">orange</b> are intended to connect to the
MultibodyPlant's per-model-instance ports of the same name. All other ports are
intended to mimic the LCM command and status message fields.

@tparam_default_scalar */
template <typename T>
class SimIiwaDriver : public systems::Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimIiwaDriver);

  /* Constructs a diagram with the given parameters. A reference to the
  `controller_plant` is retained by this system, so the `controller_plant`
  must outlive `this`. */
  SimIiwaDriver(IiwaControlMode control_mode,
                const multibody::MultibodyPlant<T>* controller_plant,
                double ext_joint_filter_tau,
                const std::optional<Eigen::VectorXd>& kp_gains);

  /* Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit SimIiwaDriver(const SimIiwaDriver<U>&);
};

}  // namespace internal
}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
