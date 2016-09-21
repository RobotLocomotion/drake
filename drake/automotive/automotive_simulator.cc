#include "drake/automotive/automotive_simulator.h"

#include <lcm/lcm-cpp.hpp>

#include "drake/automotive/gen/driving_command_translator.h"
#include "drake/automotive/gen/euler_floating_joint_state_translator.h"
#include "drake/automotive/gen/simple_car_state_translator.h"
#include "drake/automotive/simple_car.h"
#include "drake/automotive/simple_car_to_euler_floating_joint.h"
#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"
#include "drake/drakeAutomotive_export.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_receive_thread.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
namespace automotive {

template <typename T>
AutomotiveSimulator<T>::AutomotiveSimulator() {}

template <typename T>
AutomotiveSimulator<T>::~AutomotiveSimulator() {}

template <typename T>
lcm::LCM* AutomotiveSimulator<T>::get_lcm() {
  return lcm_.get();
}

template <typename T>
systems::DiagramBuilder<T>* AutomotiveSimulator<T>::get_builder() {
  DRAKE_DEMAND(!started_);
  return builder_.get();
}

template <typename T>
void AutomotiveSimulator<T>::AddSimpleCar() {
  DRAKE_DEMAND(!started_);
  const int vehicle_number = allocate_vehicle_number();

  static const DrivingCommandTranslator driving_command_translator;
  auto command_subscriber =
      builder_->template AddSystem<systems::lcm::LcmSubscriberSystem>(
          "DRIVING_COMMAND", driving_command_translator, lcm_.get());
  auto simple_car = builder_->template AddSystem<SimpleCar<T>>();
  auto coord_transform =
      builder_->template AddSystem<SimpleCarToEulerFloatingJoint<T>>();

  builder_->Connect(*command_subscriber, *simple_car);
  builder_->Connect(*simple_car, *coord_transform);
  AddPublisher(*simple_car, vehicle_number);
  AddPublisher(*coord_transform, vehicle_number);
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(const SimpleCar<T>& system,
                                          int vehicle_number) {
  DRAKE_DEMAND(!started_);
  static const SimpleCarStateTranslator translator;
  auto publisher =
      builder_->template AddSystem<systems::lcm::LcmPublisherSystem>(
          std::to_string(vehicle_number) + "_SIMPLE_CAR_STATE",
          translator, lcm_.get());
  builder_->Connect(system, *publisher);
}

template <typename T>
void AutomotiveSimulator<T>::AddPublisher(
    const SimpleCarToEulerFloatingJoint<T>& system,
    int vehicle_number) {
  DRAKE_DEMAND(!started_);
  static const EulerFloatingJointStateTranslator translator;
  auto publisher =
      builder_->template AddSystem<systems::lcm::LcmPublisherSystem>(
          std::to_string(vehicle_number) + "_FLOATING_JOINT_STATE",
          translator, lcm_.get());
  builder_->Connect(system, *publisher);
}

template <typename T>
void AutomotiveSimulator<T>::AddSystem(
    std::unique_ptr<systems::System<T>> system) {
  DRAKE_DEMAND(!started_);
  builder_->AddSystem(std::move(system));
}

template <typename T>
systems::System<T>& AutomotiveSimulator<T>::GetBuilderSystemByName(
    std::string name) {
  DRAKE_DEMAND(!started_);
  systems::System<T>* result{nullptr};
  for (systems::System<T>* system : builder_->GetMutableSystems()) {
    if (system->get_name() == name) {
      DRAKE_THROW_UNLESS(!result);
      result = system;
    }
  }
  DRAKE_THROW_UNLESS(result);
  return *result;
}

template <typename T>
const systems::System<T>& AutomotiveSimulator<T>::GetDiagramSystemByName(
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
void AutomotiveSimulator<T>::Start() {
  DRAKE_DEMAND(!started_);

  diagram_ = builder_->Build();
  simulator_ = std::make_unique<systems::Simulator<T>>(*diagram_);
  lcm_receive_thread_ =
      std::make_unique<systems::lcm::LcmReceiveThread>(lcm_.get());

  simulator_->Initialize();

  started_ = true;
}

template <typename T>
void AutomotiveSimulator<T>::StepBy(const T& time_step) {
  const T time = simulator_->get_context().get_time();
  SPDLOG_TRACE(drake::log(), "Time is now {}", time);
  simulator_->StepTo(time + time_step);
}

template <typename T>
int AutomotiveSimulator<T>::allocate_vehicle_number() {
  DRAKE_DEMAND(!started_);
  return next_vehicle_number_++;
}

template class DRAKEAUTOMOTIVE_EXPORT AutomotiveSimulator<double>;

}  // namespace automotive
}  // namespace drake
