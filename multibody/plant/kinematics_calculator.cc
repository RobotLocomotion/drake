#include "drake/multibody/plant/kinematics_calculator.h"

#include <utility>

namespace drake {
namespace multibody {

KinematicsCaculator::KinematicsCaculator(const MultibodyPlant<double>* plant)
    : plant_(*plant) {
  this->DeclareVectorInputPort(
      "state", systems::BasicVector<double>(plant_.num_multibody_states()));
  this->set_name("kinematics_calculator");

  auto alloc = [this]() {
    auto context = plant_.CreateDefaultContext();
    Value<systems::Context<double>> val{std::move(context)};
    return val.Clone();
  };

  auto calc = [this](const systems::Context<double>& context,
                     AbstractValue* val) {
    auto& output = val->get_mutable_value<systems::Context<double>>();
    const systems::BasicVector<double>* x = this->EvalVectorInput(context, 0);
    plant_.SetPositionsAndVelocities(&output, x->get_value());
  };

  this->DeclareAbstractOutputPort("kinematics", alloc, calc,
                                  {this->all_input_ports_ticket()});
}

}  // namespace multibody
}  // namespace drake
