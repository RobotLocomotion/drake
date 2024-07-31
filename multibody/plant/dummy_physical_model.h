#pragma once

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/multibody/plant/physical_model.h"
#include "drake/systems/framework/scalar_conversion_traits.h"

namespace drake {
namespace multibody {

template <typename T>
class MultibodyPlant;

namespace internal {

/* A dummy manager class derived from PhysicalModel for testing
 purpose. This dummy manager declares a single group of discrete state that
 concatenates the state added through `AppendDiscreteState()`. It also declares
 a vector output port that reports this additional state and an abstract output
 port that reports the same state.
 @tparam_default_scalar */
template <typename T>
class DummyPhysicalModel final : public PhysicalModel<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummyPhysicalModel);

  explicit DummyPhysicalModel(MultibodyPlant<T>* plant)
      : PhysicalModel<T>(plant) {}

  ~DummyPhysicalModel() final;

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

  /* Returns the output port for SceneGraph communication if one is declared.
   Throws a std::exception otherwise. */
  const systems::OutputPort<T>& GetSceneGraphPortOrThrow() const {
    if (scene_graph_output_port_ == nullptr) {
      throw std::runtime_error(
          "The SceneGraph output port has not been declared.");
    }
    return *scene_graph_output_port_;
  }

  systems::DiscreteStateIndex discrete_state_index() const {
    this->ThrowIfSystemResourcesNotDeclared(__func__);
    return discrete_state_index_;
  }

  bool is_cloneable_to_double() const final { return true; }

  bool is_cloneable_to_autodiff() const final { return true; }

  bool is_cloneable_to_symbolic() const final { return true; }

 private:
  /* Allow different specializations to access each other's private data for
   cloning to a different scalar type. */
  template <typename U>
  friend class DummyPhysicalModel;

  /* A dummy stub for PhysicalModelPointerVariant. Here we return a
   std::monostate() as a proxy for an empty model so that a manager's
   ExtractConcreteModel() results in a no-op. */
  PhysicalModelPointerVariant<T> DoToPhysicalModelPointerVariant() const final {
    return std::monostate();
  }

  std::unique_ptr<PhysicalModel<double>> CloneToDouble(
      MultibodyPlant<double>* plant) const final {
    return CloneImpl<double>(plant);
  }

  std::unique_ptr<PhysicalModel<AutoDiffXd>> CloneToAutoDiffXd(
      MultibodyPlant<AutoDiffXd>* plant) const final {
    return CloneImpl<AutoDiffXd>(plant);
  }

  std::unique_ptr<PhysicalModel<symbolic::Expression>> CloneToSymbolic(
      MultibodyPlant<symbolic::Expression>* plant) const final {
    return CloneImpl<symbolic::Expression>(plant);
  }

  template <typename ScalarType>
  std::unique_ptr<PhysicalModel<ScalarType>> CloneImpl(
      MultibodyPlant<ScalarType>* plant) const {
    auto clone = std::make_unique<DummyPhysicalModel<ScalarType>>(plant);
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
  void DoDeclareSystemResources() final;

  /* Declares a dummy output port that always returns a constant double, 42.0.
   */
  void DoDeclareSceneGraphPorts() final;

  std::vector<VectorX<T>> discrete_states_{};
  int num_dofs_{0};
  const systems::OutputPort<T>* abstract_output_port_{nullptr};
  const systems::OutputPort<T>* vector_output_port_{nullptr};
  const systems::OutputPort<T>* scene_graph_output_port_{nullptr};
  systems::DiscreteStateIndex discrete_state_index_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::internal::DummyPhysicalModel);
