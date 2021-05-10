#include "drake/systems/framework/leaf_context.h"

#include <utility>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/state.h"

namespace drake {
namespace systems {

template <typename T>
LeafContext<T>::LeafContext()
    : state_(std::make_unique<State<T>>()) {}

template <typename T>
LeafContext<T>::~LeafContext() {}

template <typename T>
LeafContext<T>::LeafContext(const LeafContext& source) : Context<T>(source) {
  // Make a deep copy of the state.
  state_ = source.CloneState();

  // Everything else was handled by the Context<T> copy constructor.
}

template <typename T>
std::unique_ptr<ContextBase> LeafContext<T>::DoCloneWithoutPointers() const {
  return std::unique_ptr<ContextBase>(new LeafContext<T>(*this));
}

template <typename T>
std::unique_ptr<State<T>> LeafContext<T>::DoCloneState() const {
  auto clone = std::make_unique<State<T>>();

  // Make a deep copy of the continuous state using BasicVector::Clone().
  const ContinuousState<T>& xc = this->get_continuous_state();
  const int num_q = xc.get_generalized_position().size();
  const int num_v = xc.get_generalized_velocity().size();
  const int num_z = xc.get_misc_continuous_state().size();
  const BasicVector<T>& xc_vector =
      dynamic_cast<const BasicVector<T>&>(xc.get_vector());
  clone->set_continuous_state(std::make_unique<ContinuousState<T>>(
      xc_vector.Clone(), num_q, num_v, num_z));

  // Make deep copies of the discrete and abstract states.
  clone->set_discrete_state(state_->get_discrete_state().Clone());
  clone->set_abstract_state(state_->get_abstract_state().Clone());

  return clone;
}

template <typename T>
std::string LeafContext<T>::do_to_string() const {
  std::ostringstream os;

  os << this->GetSystemPathname() << " Context\n";
  os << std::string(this->GetSystemPathname().size() + 9, '-') << "\n";
  os << "Time: " << this->get_time() << "\n";

  if (this->num_continuous_states() ||
      this->num_discrete_state_groups() ||
      this->num_abstract_states()) {
    os << "States:\n";
    if (this->num_continuous_states()) {
      os << "  " << this->num_continuous_states()
         << " continuous states\n";
      os << "    " << this->get_continuous_state_vector() << "\n";
    }
    if (this->num_discrete_state_groups()) {
      os << "  " << this->num_discrete_state_groups()
         << " discrete state groups with\n";
      for (int i = 0; i < this->num_discrete_state_groups(); i++) {
        os << "     " << this->get_discrete_state(i).size() << " states\n";
        os << "       " << this->get_discrete_state(i) << "\n";
      }
    }
    if (this->num_abstract_states()) {
      os << "  " << this->num_abstract_states() << " abstract states\n";
    }
    os << "\n";
  }

  if (this->num_numeric_parameter_groups() ||
      this->num_abstract_parameters()) {
    os << "Parameters:\n";
    if (this->num_numeric_parameter_groups()) {
      os << "  " << this->num_numeric_parameter_groups()
         << " numeric parameter groups";
      os << " with\n";
      for (int i = 0; i < this->num_numeric_parameter_groups(); i++) {
        os << "     " << this->get_numeric_parameter(i).size()
           << " parameters\n";
        os << "       " << this->get_numeric_parameter(i) << "\n";
      }
    }
    if (this->num_abstract_parameters()) {
      os << "  " << this->num_abstract_parameters()
         << " abstract parameters\n";
    }
  }
  return os.str();
}

template <typename T>
void LeafContext<T>::notify_set_system_id(internal::SystemId id) {
  state_->set_system_id(id);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::LeafContext)
