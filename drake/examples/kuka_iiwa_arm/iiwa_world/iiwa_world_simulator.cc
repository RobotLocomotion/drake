#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_world_simulator.h"

#include "drake/common/drake_export.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

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
void IiwaWorldSimulator<T>::AddIiwaArm(bool with_gripper) {
  // blah blah
}

template <typename T>
void IiwaWorldSimulator<T>::Build() {

}


template <typename T>
void IiwaWorldSimulator<T>::StepBy(const T& time_step) {
  const T time = simulator_->get_context().get_time();
  SPDLOG_TRACE(drake::log(), "Time is now {}", time);
  simulator_->StepTo(time + time_step);
}


}
}
}