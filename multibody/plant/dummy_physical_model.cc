#include "drake/multibody/plant/dummy_physical_model.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
DummyPhysicalModel<T>::~DummyPhysicalModel() = default;

template <typename T>
void DummyPhysicalModel<T>::DoDeclareSystemResources() {
  /* Declares the single group of discrete state. */
  VectorX<T> model_state(num_dofs_);
  int dof_offset = 0;
  for (size_t i = 0; i < discrete_states_.size(); ++i) {
    const VectorX<T>& s = discrete_states_[i];
    model_state.segment(dof_offset, s.size()) = s;
    dof_offset += s.size();
  }
  discrete_state_index_ = this->DeclareDiscreteState(model_state);

  /* Declare output ports. */
  abstract_output_port_ = &this->DeclareAbstractOutputPort(
      "dummy_abstract_output_port",
      [=]() {
        return AbstractValue::Make(model_state);
      },
      [this](const systems::Context<T>& context, AbstractValue* output) {
        VectorX<T>& data = output->get_mutable_value<VectorX<T>>();
        data = context.get_discrete_state(discrete_state_index_).get_value();
      },
      {systems::System<T>::xd_ticket()});
  vector_output_port_ = &this->DeclareVectorOutputPort(
      "dummy_vector_output_port", systems::BasicVector<T>(num_dofs_),
      [this](const systems::Context<T>& context,
             systems::BasicVector<T>* output) {
        Eigen::VectorBlock<VectorX<T>> data = output->get_mutable_value();
        data = context.get_discrete_state(discrete_state_index_).get_value();
      },
      {systems::System<T>::xd_ticket()});
}

template <typename T>
void DummyPhysicalModel<T>::DoDeclareSceneGraphPorts() {
  scene_graph_output_port_ = &this->DeclareVectorOutputPort(
      "dummy_scene_graph_port", systems::BasicVector<T>(1),
      [](const systems::Context<T>&, systems::BasicVector<T>* output) {
        output->set_value(VectorX<T>::Constant(1, 42.0));
      });
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::DummyPhysicalModel);
