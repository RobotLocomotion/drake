#include "drake/multibody/plant/kinematics_calculator.h"

#include <utility>

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {

template <typename T>
KinematicsCalculator<T>::KinematicsCalculator(const MultibodyPlant<T>* plant)
    : plant_(*plant) {
  this->DeclareVectorInputPort(
      "state", systems::BasicVector<T>(plant_.num_multibody_states()));
  this->set_name("kinematics_calculator");

  auto alloc = [this]() {
    auto context = plant_.CreateDefaultContext();
    Value<systems::Context<T>> val{std::move(context)};
    return val.Clone();
  };

  auto calc = [this](const systems::Context<T>& context, AbstractValue* val) {
    auto& output = val->get_mutable_value<systems::Context<T>>();
    const systems::BasicVector<T>* x = this->EvalVectorInput(context, 0);
    plant_.SetPositionsAndVelocities(&output, x->get_value());
  };

  this->DeclareAbstractOutputPort("kinematics", alloc, calc,
                                  {this->all_input_ports_ticket()});
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::KinematicsCalculator)
