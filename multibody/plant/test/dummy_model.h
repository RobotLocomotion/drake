#pragma once
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/physical_model.h"

namespace drake {
namespace multibody {
namespace internal {
namespace test {
using Eigen::VectorXd;
using systems::BasicVector;
using systems::Context;
using systems::DiscreteStateIndex;
using systems::OutputPortIndex;
/* A dummy manager class derived from PhysicalModel for testing
 purpose. This dummy manager declares a single group of discrete state that
 concatenates the state added through `AppendDiscreteState()`. It also declares
 a vector output port that reports this additional state and an abstract output
 port that reports the the same state. */
class DummyModel : public PhysicalModel<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummyModel);

  DummyModel() = default;

  ~DummyModel() = default;

  /* Appends additional entries to the single group of discrete state with the
   given `model_value`. */
  void AppendDiscreteState(const VectorXd& model_value) {
    ThrowIfFinalized(__func__);
    num_dofs_ += model_value.size();
    discrete_states_.emplace_back(model_value);
  }

  const systems::OutputPort<double>& get_abstract_output_port() const {
    this->ThrowIfNotFinalized(__func__);
    return this->plant().get_output_port(abstract_output_port_index_);
  }

  const systems::OutputPort<double>& get_vector_output_port() const {
    this->ThrowIfNotFinalized(__func__);
    return this->plant().get_output_port(vector_output_port_index_);
  }

  systems::DiscreteStateIndex discrete_state_index() const {
    return discrete_state_index_;
  }

 private:
  /* Declares a single group of discrete state by concatenating all the state
   added so far. It also declares two output ports that reports the value of the
   dummy discrete state: one abstract output port with underlying value type
   VectorXd and one plain-old vector port. We can verify the two ports report
   the same results as a sanity check. */
  void DoDeclareSystemResources(MultibodyPlant<double>* plant) final {
    /* Declares the single group of discrete state. */
    VectorXd model_state(num_dofs_);
    int dof_offset = 0;
    for (size_t i = 0; i < discrete_states_.size(); ++i) {
      const VectorXd& s = discrete_states_[i];
      model_state.segment(dof_offset, s.size()) = s;
      dof_offset += s.size();
    }
    discrete_state_index_ = this->DeclareDiscreteState(plant, model_state);

    /* Declare output ports. */
    abstract_output_port_index_ =
        this->DeclareAbstractOutputPort(
                plant, "dummy_abstract_output_port",
                [=]() { return AbstractValue::Make(model_state); },
                [this](const Context<double>& context, AbstractValue* output) {
                  VectorXd& data = output->get_mutable_value<VectorXd>();
                  data = context.get_discrete_state(discrete_state_index_)
                             .get_value();
                },
                {systems::System<double>::xd_ticket()})
            .get_index();
    vector_output_port_index_ =
        this->DeclareVectorOutputPort(
                plant, "dummy_vector_output_port",
                BasicVector<double>(num_dofs_),
                [this](const Context<double>& context,
                       BasicVector<double>* output) {
                  auto data = output->get_mutable_value();
                  data = context.get_discrete_state(discrete_state_index_)
                             .get_value();
                },
                {systems::System<double>::xd_ticket()})
            .get_index();
  }

  std::vector<VectorXd> discrete_states_{};
  int num_dofs_{0};
  OutputPortIndex abstract_output_port_index_;
  OutputPortIndex vector_output_port_index_;
  DiscreteStateIndex discrete_state_index_;
};
}  // namespace test
}  // namespace internal
}  // namespace multibody
}  // namespace drake
