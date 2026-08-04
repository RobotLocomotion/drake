#pragma once

#include <optional>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/manipulation/kuka_iiwa/iiwa_driver.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

/** SimIiwaDriver simulates the IIWA control and status interface using a
MultibodyPlant.

@experimental

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
- velocity_commanded (in kPositionOnly or kPositionAndTorque mode)
@endsystem

Ports shown in <b style="color:orange">orange</b> are intended to connect to the
MultibodyPlant's per-model-instance ports of the same name. All other ports are
intended to mimic the LCM command and status message fields.

@tparam_default_scalar */
template <typename T>
class SimIiwaDriver : public systems::Diagram<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimIiwaDriver);

  /** Constructs a diagram with the given `driver_config`. A reference to the
  `controller_plant` is retained by this system, so the `controller_plant`
  must outlive `this`. */
  SimIiwaDriver(const IiwaDriver& driver_config,
                const multibody::MultibodyPlant<T>* controller_plant);

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit SimIiwaDriver(const SimIiwaDriver<U>&);

  /** Given a `plant` (and associated `iiwa_instance`) and a `builder`,
  installs in that builder the `SimIiwaDriver` system to control and
  monitor an iiwa described by `controller_plant`.

  The added `SimIiwaDriver` system is connected to the actuation input port,
  state and generalized contact forces output ports in `plant` corresponding to
  the iiwa model.

  Returns the newly-added `SimIiwaDriver` System.

  Note: The Diagram will maintain an internal reference to
  `controller_plant`, so you must ensure that `controller_plant` has a
  longer lifetime than the Diagram. */
  static const systems::System<double>& AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      const multibody::MultibodyPlant<double>& plant,
      const multibody::ModelInstanceIndex iiwa_instance,
      const IiwaDriver& driver_config,
      const multibody::MultibodyPlant<double>& controller_plant);
};

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
