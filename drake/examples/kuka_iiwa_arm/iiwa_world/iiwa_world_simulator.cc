#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_world_simulator.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"

#include "drake/common/drake_export.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/parser_sdf.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/rigid_body_plant/drake_visualizer.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
IiwaWorldSimulator<T>::IiwaWorldSimulator()
    : IiwaWorldSimulator(std::make_unique<lcm::DrakeLcm>()) {}

template <typename T>
IiwaWorldSimulator<T>::IiwaWorldSimulator(
    std::unique_ptr<lcm::DrakeLcmInterface> lcm) :
    lcm_(std::move(lcm)) {}

template <typename T>
IiwaWorldSimulator<T>::~IiwaWorldSimulator() {
  lcm_.reset();
}

template <typename T>
int IiwaWorldSimulator<T>::AddIiwaArm(bool with_gripper) {

  DRAKE_DEMAND(!started_);

  parsers::ModelInstanceIdTable vehicle_instance_id_table =
      drake::parsers::urdf::AddModelInstanceFromUrdfFile(
          drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
          kFixed, nullptr /* weld to frame */, rigid_body_tree_.get());

  AddGround(tree.get());

  const int object_number = allocate_vehicle_number();

//  const parsers::ModelInstanceIdTable table =
//      parsers::sdf::AddModelInstancesFromSdfFileInWorldFrame(
//          sdf_filename, kRollPitchYaw, rigid_body_tree_.get());

  const int model_instance_id = table.begin()->second;
  const std::vector<const RigidBody*> bodies =
      rigid_body_tree_->FindModelInstanceBodies(model_instance_id);
  rigid_body_tree_publisher_inputs_.push_back(
      std::make_pair(model_instance_id, coord_transform));
  return model_instance_id;
}

template <typename T>
void IiwaWorldSimulator<T>::Build() {

}

template <typename T>
const systems::System<T>& IiwaWorldSimulator<T>::GetDiagramSystemByName(
    std::string name) const {
  DRAKE_DEMAND(started_);
  // Ask the diagram.
  const systems::System<T>* result{nullptr};
  for (const systems::System<T>* system : diagram_->GetSystems()) {
    if (system->get_name() == name) {
      DRAKE_THROW_UNLESS(!result);
      result = system;
    }
  }
  DRAKE_THROW_UNLESS(result);
  return *result;
}


template <typename T>
void IiwaWorldSimulator<T>::StepBy(const T& time_step) {
  const T time = simulator_->get_context().get_time();
  SPDLOG_TRACE(drake::log(), "Time is now {}", time);
  simulator_->StepTo(time + time_step);
}

template class DRAKE_EXPORT IiwaWorldSimulator<double>;
}
}
}