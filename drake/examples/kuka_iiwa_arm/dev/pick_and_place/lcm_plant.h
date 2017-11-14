#pragma once

#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_wsg_diagram_factory.h"
#include "drake/examples/kuka_iiwa_arm/pick_and_place/pick_and_place_configuration.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace pick_and_place {
/** A custom systems::Diagram comprised of an IiwaWsgPlantWithStateEstimator, a
simulated Optitrack system, and SchunkWsgControllers for each gripper in the
plant. The purpose of this system is to provide a drop-in replacement for the
hardware + drivers. Note that LCM publishers and subscribers are intentionally
ommitted from this system so that it can be directly connected to other systems.
The block diagram for a system with a single arm is shown below:

                       ┌─────────┐    ┌───────────┐    ┌───────────┐
                       │ Frame   │    │ Optitrack │    │ Optitrack │   optitrack
                  ┌───▶│ Pose    ├───▶│ Encoder   ├───▶│ LCM       ├──▶frame
                  │    │ Tracker │    │           │    │ Frame     │
                  │    │         │    │           │    │ Sender    │
                  │    └─────────┘    └───────────┘    └───────────┘
                  └────────────────────────────────────┐
                                                       │
                                                       │
                                   ┌──────────────┐    │
                                   │              │    │
                                   │              │    │
                                   │              ├────┘
                         iiwa      │              │
           ┌──────────┐  desired   │              │ iiwa  ┌────────┐
     iiwa  │ Iiwa     │  state     │              │ state │ Iiwa   │   iiwa
     cmd──▶│ Command  ├───────────▶│ IiwaWsgPlant ├──────▶│ Status ├──▶status
           │ Receiver │            │              │       │ Sender │
           └──────────┘            │              │       └────────┘
           ┌───────────┐ wsg       │              │ wsg   ┌────────┐
     wsg   │ Wsg       │ input     │              │ state │ Schunk │   wsg
     cmd──▶│ Controller├──────────▶│              ├───┬──▶│ Wsg    ├──▶status
           │           │           │              │   │   │ Status │
      ┌───▶│           │           │              │   │   │ Sender │
      │    └───────────┘           └──────────────┘   │   └────────┘
      │                                               │
      │                                               │
      └───────────────────────────────────────────────┘
**/

class LcmPlant : public systems::Diagram<double> {
 public:
  LcmPlant(
      const pick_and_place::SimulatedPlantConfiguration& plant_configuration,
      const pick_and_place::OptitrackConfiguration& optitrack_configuration);

  const systems::InputPortDescriptor<double>& get_input_port_iiwa_command(
      int index) const {
    return this->get_input_port(input_port_iiwa_command_.at(index));
  }

  const systems::InputPortDescriptor<double>& get_input_port_wsg_command(
      int index) const {
    return this->get_input_port(input_port_wsg_command_.at(index));
  }

  const systems::OutputPort<double>& get_output_port_iiwa_status(
      int index) const {
    return this->get_output_port(output_port_iiwa_status_.at(index));
  }

  const systems::OutputPort<double>& get_output_port_iiwa_robot_state(
      int index) const {
    return this->get_output_port(output_port_iiwa_robot_state_.at(index));
  }

  const systems::OutputPort<double>& get_output_port_wsg_status(
      int index) const {
    return this->get_output_port(output_port_wsg_status_.at(index));
  }

  const systems::OutputPort<double>& get_output_port_optitrack_frame() const {
    return this->get_output_port(output_port_optitrack_frame_);
  }

  const systems::OutputPort<double>& get_output_port_contact_results() const {
    return this->get_output_port(output_port_contact_results_);
  }

  const systems::OutputPort<double>& get_output_port_plant_state() const {
    return this->get_output_port(output_port_plant_state_);
  }

  const RigidBodyTree<double>& get_tree() const {
    return this->iiwa_and_wsg_plant_->get_tree();
  }

  int num_iiwa() const { return input_port_iiwa_command_.size(); }

  int num_wsg() const { return input_port_wsg_command_.size(); }

 private:
  std::vector<int> input_port_iiwa_command_;
  std::vector<int> input_port_wsg_command_;
  std::vector<int> output_port_iiwa_status_;
  std::vector<int> output_port_wsg_status_;
  std::vector<int> output_port_iiwa_robot_state_;
  int output_port_optitrack_frame_{-1};
  int output_port_contact_results_{-1};
  int output_port_plant_state_{-1};
  IiwaAndWsgPlantWithStateEstimator<double>* iiwa_and_wsg_plant_;
};
}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
