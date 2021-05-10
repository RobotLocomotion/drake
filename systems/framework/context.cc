#include "drake/systems/framework/context.h"

#include "drake/common/pointer_cast.h"

namespace drake {
namespace systems {

template <typename T>
void Context<T>::SetTime(const T& time_sec) {
  ThrowIfNotRootContext(__func__, "Time");
  const int64_t change_event = this->start_new_change_event();
  PropagateTimeChange(this, time_sec, {}, change_event);
}

template <typename T>
void Context<T>::SetAccuracy(const std::optional<double>& accuracy) {
  ThrowIfNotRootContext(__func__, "Accuracy");
  const int64_t change_event = this->start_new_change_event();
  PropagateAccuracyChange(this, accuracy, change_event);
}

template <typename T>
State<T>& Context<T>::get_mutable_state() {
  const int64_t change_event = this->start_new_change_event();
  PropagateBulkChange(change_event, &Context<T>::NoteAllStateChanged);
  return do_access_mutable_state();
}

template <typename T>
ContinuousState<T>& Context<T>::get_mutable_continuous_state() {
  const int64_t change_event = this->start_new_change_event();
  PropagateBulkChange(change_event,
                      &Context<T>::NoteAllContinuousStateChanged);
  return do_access_mutable_state().get_mutable_continuous_state();
}

template <typename T>
DiscreteValues<T>& Context<T>::get_mutable_discrete_state() {
  const int64_t change_event = this->start_new_change_event();
  PropagateBulkChange(change_event,
                      &Context<T>::NoteAllDiscreteStateChanged);
  return do_access_mutable_state().get_mutable_discrete_state();
}

template <typename T>
AbstractValues& Context<T>::get_mutable_abstract_state() {
  const int64_t change_event = this->start_new_change_event();
  PropagateBulkChange(change_event,
                      &Context<T>::NoteAllAbstractStateChanged);
  return do_access_mutable_state().get_mutable_abstract_state();
}

template <typename T>
Parameters<T>& Context<T>::get_mutable_parameters() {
  const int64_t change_event = this->start_new_change_event();
  PropagateBulkChange(change_event, &Context<T>::NoteAllParametersChanged);
  return *parameters_;
}

template <typename T>
BasicVector<T>& Context<T>::get_mutable_numeric_parameter(int index) {
  const int64_t change_event = this->start_new_change_event();
  PropagateBulkChange(change_event,
                      &Context<T>::NoteAllNumericParametersChanged);
  return parameters_->get_mutable_numeric_parameter(index);
}

template <typename T>
AbstractValue& Context<T>::get_mutable_abstract_parameter(int index) {
  const int64_t change_event = this->start_new_change_event();
  PropagateBulkChange(change_event,
                      &Context<T>::NoteAllAbstractParametersChanged);
  return parameters_->get_mutable_abstract_parameter(index);
}

template <typename T>
VectorBase<T>& Context<T>::SetTimeAndGetMutableQVector(const T& time_sec) {
  ThrowIfNotRootContext(__func__, "Time");
  const int64_t change_event = this->start_new_change_event();
  PropagateTimeChange(this, time_sec, {}, change_event);
  PropagateBulkChange(change_event, &Context<T>::NoteAllQChanged);
  return do_access_mutable_state()  // No invalidation here.
      .get_mutable_continuous_state()
      .get_mutable_generalized_position();
}

template <typename T>
std::pair<VectorBase<T>*, VectorBase<T>*> Context<T>::GetMutableVZVectors() {
  const int64_t change_event = this->start_new_change_event();
  PropagateBulkChange(change_event, &Context<T>::NoteAllVZChanged);
  ContinuousState<T>& xc =  // No invalidation here.
      do_access_mutable_state().get_mutable_continuous_state();
  return {&xc.get_mutable_generalized_velocity(),
          &xc.get_mutable_misc_continuous_state()};
}

template <typename T>
std::unique_ptr<Context<T>> Context<T>::Clone() const {
  return dynamic_pointer_cast_or_throw<Context<T>>(ContextBase::Clone());
}

template <typename T>
std::unique_ptr<State<T>> Context<T>::CloneState() const {
  auto result = DoCloneState();
  result->set_system_id(this->get_system_id());
  return result;
}

template <typename T>
std::string Context<T>::to_string() const {
  return do_to_string();
}

template <typename T>
void Context<T>::PerturbTime(const T& time, const T& true_time) {
  ThrowIfNotRootContext(__func__, "Time");
  const int64_t change_event = this->start_new_change_event();
  PropagateTimeChange(this, time, std::optional<T>(true_time), change_event);
}

template <typename T>
Context<T>::Context() = default;

template <typename T>
Context<T>::Context(const Context<T>&) = default;

template <typename T>
void Context<T>::PropagateTimeChange(
    Context<T>* context, const T& time, const std::optional<T>& true_time,
    int64_t change_event) {
  DRAKE_ASSERT(context != nullptr);
  context->NoteTimeChanged(change_event);
  context->time_ = time;
  context->true_time_ = true_time;
  context->DoPropagateTimeChange(time, true_time, change_event);
}

template <typename T>
void Context<T>::PropagateAccuracyChange(
    Context<T>* context, const std::optional<double>& accuracy,
    int64_t change_event) {
  DRAKE_ASSERT(context != nullptr);
  context->NoteAccuracyChanged(change_event);
  context->accuracy_ = accuracy;
  context->DoPropagateAccuracyChange(accuracy, change_event);
}

template <typename T>
std::unique_ptr<Context<T>> Context<T>::CloneWithoutPointers(
    const Context<T>& source) {
  return dynamic_pointer_cast_or_throw<Context<T>>(
      ContextBase::CloneWithoutPointers(source));
}

template <typename T>
void Context<T>::init_continuous_state(std::unique_ptr<ContinuousState<T>> xc) {
  do_access_mutable_state().set_continuous_state(std::move(xc));
}

template <typename T>
void Context<T>::init_discrete_state(std::unique_ptr<DiscreteValues<T>> xd) {
  do_access_mutable_state().set_discrete_state(std::move(xd));
}

template <typename T>
void Context<T>::init_abstract_state(std::unique_ptr<AbstractValues> xa) {
  do_access_mutable_state().set_abstract_state(std::move(xa));
}

template <typename T>
void Context<T>::init_parameters(std::unique_ptr<Parameters<T>> params) {
  DRAKE_DEMAND(params != nullptr);
  parameters_ = std::move(params);
}

template <typename T>
void Context<T>::ThrowIfNotRootContext(
    const char* func_name, const char* quantity) const {
  if (!is_root_context()) {
    throw std::logic_error(
        fmt::format("{}(): {} change allowed only in the root Context.",
                    func_name, quantity));
  }
}

template <typename T>
void Context<T>::SetTimeAndNoteContinuousStateChangeHelper(
    const char* func_name, const T& time_sec) {
  ThrowIfNotRootContext(func_name, "Time");
  const int64_t change_event = this->start_new_change_event();
  PropagateTimeChange(this, time_sec, {}, change_event);
  PropagateBulkChange(change_event,
                      &Context<T>::NoteAllContinuousStateChanged);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Context)
