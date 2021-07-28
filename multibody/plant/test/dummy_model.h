#pragma once
#include <memory>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/physical_model.h"
#include "drake/systems/framework/scalar_conversion_traits.h"

namespace drake {
namespace multibody {
namespace internal {
namespace test {
using systems::BasicVector;
using systems::Context;
using systems::DiscreteStateIndex;
using systems::OutputPortIndex;
// TODO(xuchenhan-tri): Rename this class to DummyPhysicalModel.
/* A dummy manager class derived from PhysicalModel for testing
 purpose. This dummy manager declares a single group of discrete state that
 concatenates the state added through `AppendDiscreteState()`. It also declares
 a vector output port that reports this additional state and an abstract output
 port that reports the same state.
 @tparam_nonsymbolic_scalar */
template <typename T>
class DummyModel : public PhysicalModel<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummyModel);

  DummyModel() = default;

  ~DummyModel() final = default;

  /* Appends additional entries to the single group of discrete state with the
   given `model_value`. */
  void AppendDiscreteState(const VectorX<T>& model_value) {
    this->ThrowIfSystemResourcesDeclared(__func__);
    num_dofs_ += model_value.size();
    discrete_states_.emplace_back(model_value);
  }

  const systems::OutputPort<T>& get_abstract_output_port() const {
    this->ThrowIfSystemResourcesNotDeclared(__func__);
    return *abstract_output_port_;
  }

  const systems::OutputPort<T>& get_vector_output_port() const {
    this->ThrowIfSystemResourcesNotDeclared(__func__);
    return *vector_output_port_;
  }

  systems::DiscreteStateIndex discrete_state_index() const {
    this->ThrowIfSystemResourcesNotDeclared(__func__);
    return discrete_state_index_;
  }

 private:
  /* Allow different specializations to access each other's private data for
   cloning to a different scalar type. */
  template <typename U>
  friend class DummyModel;

  std::unique_ptr<PhysicalModel<double>> CloneToDouble() const final {
    return CloneToScalar<double>();
  }

  std::unique_ptr<PhysicalModel<AutoDiffXd>> CloneToAutoDiffXd() const final {
    return CloneToScalar<AutoDiffXd>();
  }

  bool is_cloneable_to_double() const final { return true; }

  bool is_cloneable_to_autodiff() const final { return true; }

  template <typename ScalarType>
  std::unique_ptr<PhysicalModel<ScalarType>> CloneToScalar() const {
    auto clone = std::make_unique<DummyModel<ScalarType>>();
    clone->num_dofs_ = this->num_dofs_;
    clone->discrete_states_.resize(this->discrete_states_.size());
    for (size_t i = 0; i < discrete_states_.size(); ++i) {
      clone->discrete_states_[i] = this->discrete_states_[i].unaryExpr(
          systems::scalar_conversion::ValueConverter<ScalarType, T>{});
    }
    return clone;
  }

  /* Declares a single group of discrete state by concatenating all the state
   added so far. It also declares two output ports that reports the value of the
   dummy discrete state: one abstract output port with underlying value type
   VectorX<T> and one plain-old vector port. We can verify the two ports report
   the same results as a sanity check. */
  void DoDeclareSystemResources(MultibodyPlant<T>* plant) final {
    /* Declares the single group of discrete state. */
    VectorX<T> model_state(num_dofs_);
    int dof_offset = 0;
    for (size_t i = 0; i < discrete_states_.size(); ++i) {
      const VectorX<T>& s = discrete_states_[i];
      model_state.segment(dof_offset, s.size()) = s;
      dof_offset += s.size();
    }
    discrete_state_index_ = this->DeclareDiscreteState(plant, model_state);

    /* Declare output ports. */
    abstract_output_port_ = &this->DeclareAbstractOutputPort(
        plant, "dummy_abstract_output_port",
        [=]() { return AbstractValue::Make(model_state); },
        [this](const Context<T>& context, AbstractValue* output) {
          VectorX<T>& data = output->get_mutable_value<VectorX<T>>();
          data = context.get_discrete_state(discrete_state_index_).get_value();
        },
        {systems::System<T>::xd_ticket()});
    vector_output_port_ = &this->DeclareVectorOutputPort(
        plant, "dummy_vector_output_port", BasicVector<T>(num_dofs_),
        [this](const Context<T>& context, BasicVector<T>* output) {
          auto data = output->get_mutable_value();
          data = context.get_discrete_state(discrete_state_index_).get_value();
        },
        {systems::System<T>::xd_ticket()});
  }

  std::vector<VectorX<T>> discrete_states_{};
  int num_dofs_{0};
  const systems::OutputPort<T>* abstract_output_port_{nullptr};
  const systems::OutputPort<T>* vector_output_port_{nullptr};
  DiscreteStateIndex discrete_state_index_;
};
}  // namespace test
}  // namespace internal
}  // namespace multibody
}  // namespace drake
