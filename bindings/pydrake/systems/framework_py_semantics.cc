#include "drake/bindings/pydrake/systems/framework_py_semantics.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/bindings/generated_docstrings/systems_framework.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/common/ref_cycle_pybind.h"
#include "drake/bindings/pydrake/common/type_safe_index_pybind.h"
#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/builder_life_support_pybind.h"
#include "drake/bindings/pydrake/systems/value_producer_pybind.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_output_port.h"
#include "drake/systems/framework/system_output.h"

using std::string;

namespace drake {
namespace pydrake {

namespace {

// NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
using namespace drake::systems;
constexpr auto& doc = pydrake_doc_systems_framework.drake.systems;

// Given a vector of (possibly null) pointers, returns a vector formed by
// calling Clone() elementwise.
template <typename T>
std::vector<std::unique_ptr<T>> CloneVectorOfPointers(
    const std::vector<const T*>& input) {
  std::vector<std::unique_ptr<T>> result;
  result.reserve(input.size());
  for (const T* item : input) {
    result.push_back(item != nullptr ? item->Clone() : nullptr);
  }
  return result;
}

// Given an InputPort or OutputPort as self, return self.Eval(context).  In
// python, always returns either a numpy.ndarray (when vector-valued) or the
// unwrapped T in a Value<T> (when abstract-valued).
template <typename SomeObject, typename T>
py::object DoEval(const SomeObject* self, const systems::Context<T>& context) {
  switch (self->get_data_type()) {
    case systems::kVectorValued: {
      const VectorX<T> eigen_copy = self->Eval(context);
      return py::cast(eigen_copy);
    }
    case systems::kAbstractValued: {
      const auto& abstract = self->template Eval<AbstractValue>(context);
      // The storage for the abstract value is owned by the context, so we need
      // to inform pybind that the abstract value reference should keep the
      // entire context alive.  Note that `abstract_value_ref` itself will be
      // immediately released (it's a local variable, and we don't return it)
      // but in certain cases the return from `get_value` is an internal
      // reference into the `abstract_value_ref`, and so will need to
      // transitively keep the entire context alive as well.
      py::object abstract_value_ref =
          py::cast(&abstract, py_rvp::reference_internal, py::cast(&context));
      return abstract_value_ref.attr("get_value")();
    }
  }
  DRAKE_UNREACHABLE();
}

void DoScalarIndependentDefinitions(py::module m) {
  {
    using Class = UseDefaultName;
    py::class_<Class>(m, "UseDefaultName", doc.UseDefaultName.doc)
        .def("__repr__", [](const Class&) { return "kUseDefaultName"; });
  }
  m.attr("kUseDefaultName") = kUseDefaultName;

  py::enum_<PortDataType>(m, "PortDataType")
      .value("kVectorValued", kVectorValued)
      .value("kAbstractValued", kAbstractValued);

  py::enum_<InputPortSelection>(m, "InputPortSelection")
      .value("kNoInput", InputPortSelection::kNoInput)
      .value("kUseFirstInputIfItExists",
          InputPortSelection::kUseFirstInputIfItExists);
  py::enum_<OutputPortSelection>(m, "OutputPortSelection")
      .value("kNoOutput", OutputPortSelection::kNoOutput)
      .value("kUseFirstOutputIfItExists",
          OutputPortSelection::kUseFirstOutputIfItExists);

  BindTypeSafeIndex<DependencyTicket>(m, "DependencyTicket");
  BindTypeSafeIndex<CacheIndex>(m, "CacheIndex");
  BindTypeSafeIndex<SubsystemIndex>(m, "SubsystemIndex");
  BindTypeSafeIndex<InputPortIndex>(m, "InputPortIndex");
  BindTypeSafeIndex<OutputPortIndex>(m, "OutputPortIndex");
  BindTypeSafeIndex<ContinuousStateIndex>(m, "ContinuousStateIndex");
  BindTypeSafeIndex<DiscreteStateIndex>(m, "DiscreteStateIndex");
  BindTypeSafeIndex<AbstractStateIndex>(m, "AbstractStateIndex");
  BindTypeSafeIndex<NumericParameterIndex>(m, "NumericParameterIndex");
  BindTypeSafeIndex<AbstractParameterIndex>(m, "AbstractParameterIndex");
  BindTypeSafeIndex<SystemConstraintIndex>(m, "SystemConstraintIndex");

  py::class_<FixedInputPortValue>(
      m, "FixedInputPortValue", doc.FixedInputPortValue.doc)
      .def("GetMutableData", &FixedInputPortValue::GetMutableData,
          py_rvp::reference_internal,
          doc.FixedInputPortValue.GetMutableData.doc);

  // N.B. `AbstractValues` provides the ability to reference non-owned values,
  // without copying them. For consistency with other model-value Python
  // bindings, only the ownership variant is exposed.
  py::class_<AbstractValues> abstract_values(
      m, "AbstractValues", doc.AbstractValues.doc);
  DefClone(&abstract_values);
  abstract_values  // BR
      .def(py::init<>(), doc.AbstractValues.ctor.doc_0args)
      .def(py::init([](const std::vector<const AbstractValue*>& data) {
        return std::make_unique<AbstractValues>(CloneVectorOfPointers(data));
      }),
          py::arg("data"), doc.AbstractValues.ctor.doc_1args)
      .def("size", &AbstractValues::size, doc.AbstractValues.size.doc)
      .def("get_value", &AbstractValues::get_value, py::arg("index"),
          py_rvp::reference_internal, doc.AbstractValues.get_value.doc)
      .def("get_mutable_value", &AbstractValues::get_mutable_value,
          py::arg("index"), py_rvp::reference_internal,
          doc.AbstractValues.get_mutable_value.doc)
      .def("SetFrom", &AbstractValues::SetFrom, doc.AbstractValues.SetFrom.doc);

  {
    using Class = TriggerType;
    constexpr auto& cls_doc = doc.TriggerType;
    py::enum_<TriggerType>(m, "TriggerType", cls_doc.doc)
        .value("kUnknown", Class::kUnknown, cls_doc.kUnknown.doc)
        .value("kInitialization", Class::kInitialization,
            cls_doc.kInitialization.doc)
        .value("kForced", Class::kForced, cls_doc.kForced.doc)
        .value("kTimed", Class::kTimed, cls_doc.kTimed.doc)
        .value("kPeriodic", Class::kPeriodic, cls_doc.kPeriodic.doc)
        .value("kPerStep", Class::kPerStep, cls_doc.kPerStep.doc)
        .value("kWitness", Class::kWitness, cls_doc.kWitness.doc);
  }

  py::enum_<WitnessFunctionDirection>(
      m, "WitnessFunctionDirection", doc.WitnessFunctionDirection.doc)
      .value("kNone", WitnessFunctionDirection::kNone,
          doc.WitnessFunctionDirection.kNone.doc)
      .value("kPositiveThenNonPositive",
          WitnessFunctionDirection::kPositiveThenNonPositive,
          doc.WitnessFunctionDirection.kPositiveThenNonPositive.doc)
      .value("kNegativeThenNonNegative",
          WitnessFunctionDirection::kNegativeThenNonNegative,
          doc.WitnessFunctionDirection.kNegativeThenNonNegative.doc)
      .value("kCrossesZero", WitnessFunctionDirection::kCrossesZero,
          doc.WitnessFunctionDirection.kCrossesZero.doc);

  {
    py::class_<PeriodicEventData> cls(
        m, "PeriodicEventData", doc.PeriodicEventData.doc);
    DefCopyAndDeepCopy(&cls);
    cls  // BR
        .def("period_sec", &PeriodicEventData::period_sec,
            doc.PeriodicEventData.period_sec.doc)
        .def("offset_sec", &PeriodicEventData::offset_sec,
            doc.PeriodicEventData.offset_sec.doc);
  }

  {
    using Class = EventStatus;
    constexpr auto& cls_doc = doc.EventStatus;
    py::class_<Class> cls(m, "EventStatus", cls_doc.doc);

    using Enum = Class::Severity;
    constexpr auto& enum_doc = cls_doc.Severity;
    py::enum_<Enum>(cls, "Severity", enum_doc.doc)
        .value("kDidNothing", Enum::kDidNothing, enum_doc.kDidNothing.doc)
        .value("kSucceeded", Enum::kSucceeded, enum_doc.kSucceeded.doc)
        .value("kReachedTermination", Enum::kReachedTermination,
            enum_doc.kReachedTermination.doc)
        .value("kFailed", Enum::kFailed, enum_doc.kFailed.doc);

    cls  // BR
        .def_static("DidNothing", &Class::DidNothing, cls_doc.DidNothing.doc)
        .def_static("Succeeded", &Class::Succeeded, cls_doc.Succeeded.doc)
        .def_static(
            "ReachedTermination",
            [](py::object system, std::string message) {
              // The `class System` is not bound yet, so we must use a dynamic
              // argument type to avoid referring to a class that doesn't exist.
              // TODO(jwnimmer-tri) With major surgery to the bindings we could
              // change the order of operations to work around this, but at the
              // moment that's too much churn to for the payoff.
              const SystemBase* system_base = system.cast<SystemBase*>();
              return EventStatus::ReachedTermination(
                  system_base, std::move(message));
            },
            py::arg("system"), py::arg("message"),
            cls_doc.ReachedTermination.doc)
        .def_static(
            "Failed",
            [](py::object system, std::string message) {
              // The `class System` is not bound yet, so we must use a dynamic
              // argument type to avoid referring to a class that doesn't exist.
              // TODO(jwnimmer-tri) With major surgery to the bindings we could
              // change the order of operations to work around this, but at the
              // moment that's too much churn to for the payoff.
              const SystemBase* system_base = system.cast<SystemBase*>();
              return EventStatus::Failed(system_base, std::move(message));
            },
            py::arg("system"), py::arg("message"), cls_doc.Failed.doc)
        .def("severity", &Class::severity, cls_doc.severity.doc)
        .def(
            "system",
            [](const Class& self) -> py::object {
              // The `class System` is not bound yet, so we must use a dynamic
              // return type to avoid referring to a class that doesn't exist.
              // TODO(jwnimmer-tri) With major surgery to the bindings we could
              // change the order of operations to work around this, but at the
              // moment that's too much churn to for the payoff.
              const SystemBase* result = self.system();
              if (result == nullptr) {
                return py::none();
              }
              py::object self_py = py::cast(self, py_rvp::reference);
              return py::cast(result, py_rvp::reference_internal, self_py);
            },
            cls_doc.system.doc)
        .def("message", &Class::message, cls_doc.message.doc)
        .def("KeepMoreSevere", &Class::KeepMoreSevere, py::arg("candidate"),
            cls_doc.KeepMoreSevere.doc);
    DefCopyAndDeepCopy(&cls);
  }

  py::class_<ContextBase>(m, "ContextBase", doc.ContextBase.doc)
      .def("num_input_ports", &ContextBase::num_input_ports,
          doc.ContextBase.num_input_ports.doc)
      .def("num_output_ports", &ContextBase::num_output_ports,
          doc.ContextBase.num_output_ports.doc)
      .def("DisableCaching", &ContextBase::DisableCaching,
          doc.ContextBase.DisableCaching.doc)
      .def("EnableCaching", &ContextBase::EnableCaching,
          doc.ContextBase.EnableCaching.doc)
      .def("SetAllCacheEntriesOutOfDate",
          &ContextBase::SetAllCacheEntriesOutOfDate,
          doc.ContextBase.SetAllCacheEntriesOutOfDate.doc)
      .def("FreezeCache", &ContextBase::FreezeCache,
          doc.ContextBase.FreezeCache.doc)
      .def("UnfreezeCache", &ContextBase::UnfreezeCache,
          doc.ContextBase.UnfreezeCache.doc)
      .def("is_cache_frozen", &ContextBase::is_cache_frozen,
          doc.ContextBase.is_cache_frozen.doc);
  // TODO(russt, eric.cousineau): Add remaining methods from ContextBase here.

  {
    using Class = ValueProducer;
    constexpr auto& cls_doc = doc.ValueProducer;
    py::class_<Class>(m, "ValueProducer", cls_doc.doc)
        .def(py::init([](py::function allocate,
                          std::function<void(py::object, py::object)> calc) {
          return Class(MakeCppCompatibleAllocateCallback(std::move(allocate)),
              MakeCppCompatibleCalcCallback(std::move(calc)));
        }),
            py::arg("allocate"), py::arg("calc"), cls_doc.ctor.doc_overload_5d)
        .def_static("NoopCalc", &Class::NoopCalc, cls_doc.NoopCalc.doc);
  }

  {
    using Class = CacheEntryValue;
    constexpr auto& cls_doc = doc.CacheEntryValue;
    py::class_<Class>(m, "CacheEntryValue", cls_doc.doc)
        .def(
            "GetValueOrThrow",
            [](const Class& self) {
              py::object value = py::cast<const AbstractValue*>(
                  &self.GetAbstractValueOrThrow());
              return value.attr("get_value")();
            },
            py_rvp::reference_internal, cls_doc.GetValueOrThrow.doc)
        .def(
            "GetMutableValueOrThrow",
            [](Class* self) {
              py::object value = py::cast<AbstractValue*>(
                  &self->GetMutableAbstractValueOrThrow());
              return value.attr("get_mutable_value")();
            },
            py_rvp::reference_internal, cls_doc.GetMutableValueOrThrow.doc);
  }

  {
    using Class = CacheEntry;
    constexpr auto& cls_doc = doc.CacheEntry;
    py::class_<Class>(m, "CacheEntry", cls_doc.doc)
        .def("prerequisites", &Class::prerequisites, cls_doc.prerequisites.doc)
        .def("EvalAbstract", &Class::EvalAbstract, py::arg("context"),
            py_rvp::reference_internal, cls_doc.EvalAbstract.doc)
        .def(
            "Eval",
            [](const Class& self, const ContextBase& context) {
              const auto& abstract = self.EvalAbstract(context);
              // See above comments in `DoEval`.
              py::object context_ref = py::cast(&context);
              py::object abstract_value_ref =
                  py::cast(&abstract, py_rvp::reference_internal, context_ref);
              return abstract_value_ref.attr("get_value")();
            },
            py::arg("context"), cls_doc.Eval.doc)
        .def("is_out_of_date", &Class::is_out_of_date, py::arg("context"),
            cls_doc.is_out_of_date.doc)
        .def("is_cache_entry_disabled", &Class::is_cache_entry_disabled,
            py::arg("context"), cls_doc.is_cache_entry_disabled.doc)
        .def("disable_caching", &Class::disable_caching, py::arg("context"),
            cls_doc.disable_caching.doc)
        .def("enable_caching", &Class::enable_caching, py::arg("context"),
            cls_doc.enable_caching.doc)
        .def("disable_caching_by_default", &Class::disable_caching_by_default,
            cls_doc.disable_caching_by_default.doc)
        .def("is_disabled_by_default", &Class::is_disabled_by_default,
            cls_doc.is_disabled_by_default.doc)
        .def("description", &Class::description, cls_doc.description.doc)
        .def("get_cache_entry_value", &Class::get_cache_entry_value,
            py::arg("context"),
            // Keep alive, ownership: `return` keeps `context` alive.
            py::keep_alive<0, 2>(), py_rvp::reference,
            cls_doc.get_cache_entry_value.doc)
        .def("get_mutable_cache_entry_value",
            &Class::get_mutable_cache_entry_value, py::arg("context"),
            // Keep alive, ownership: `return` keeps `context` alive.
            py::keep_alive<0, 2>(), py_rvp::reference,
            cls_doc.get_mutable_cache_entry_value.doc)
        .def("cache_index", &Class::cache_index, cls_doc.cache_index.doc)
        .def("ticket", &Class::ticket, cls_doc.ticket.doc)
        .def("has_default_prerequisites", &Class::has_default_prerequisites,
            cls_doc.has_default_prerequisites.doc);
  }

  {
    // Binding this full feature in pydrake is non-trivial effort. For now,
    // we'll just provide the default constructor for use in our unit tests,
    // and mark the class itself with private visibility since it's not very
    // useful without the full bindings.
    using Class = ExternalSystemConstraint;
    constexpr auto& cls_doc = doc.ExternalSystemConstraint;
    py::class_<Class>(m, "_ExternalSystemConstraint", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc_0args);
  }
}

template <typename T>
py::class_<Context<T>, ContextBase> DefineContext(py::module m) {
  auto context_cls = DefineTemplateClassWithDefault<Context<T>, ContextBase>(
      m, "Context", GetPyParam<T>(), doc.Context.doc);
  context_cls
      // Bindings for the Context methods in the Doxygen group titled
      // "Accessors for locally-stored values", placed in the same order
      // as the header file.
      .def("get_time", &Context<T>::get_time, doc.Context.get_time.doc)
      .def("get_state", &Context<T>::get_state, py_rvp::reference_internal,
          doc.Context.get_state.doc)
      .def("is_stateless", &Context<T>::is_stateless,
          doc.Context.is_stateless.doc)
      .def("has_only_continuous_state", &Context<T>::has_only_continuous_state,
          doc.Context.has_only_continuous_state.doc)
      .def("has_only_discrete_state", &Context<T>::has_only_discrete_state,
          doc.Context.has_only_discrete_state.doc)
      .def("num_total_states", &Context<T>::num_total_states,
          doc.Context.num_total_states.doc)
      .def("num_continuous_states", &Context<T>::num_continuous_states,
          doc.Context.num_continuous_states.doc)
      .def("get_continuous_state", &Context<T>::get_continuous_state,
          py_rvp::reference_internal, doc.Context.get_continuous_state.doc)
      .def("get_continuous_state_vector",
          &Context<T>::get_continuous_state_vector, py_rvp::reference_internal,
          doc.Context.get_continuous_state_vector.doc)
      .def("num_discrete_state_groups", &Context<T>::num_discrete_state_groups,
          doc.Context.num_discrete_state_groups.doc)
      .def("get_discrete_state",
          overload_cast_explicit<const DiscreteValues<T>&>(
              &Context<T>::get_discrete_state),
          py_rvp::reference_internal, doc.Context.get_discrete_state.doc_0args)
      .def("get_discrete_state_vector", &Context<T>::get_discrete_state_vector,
          py_rvp::reference_internal, doc.Context.get_discrete_state_vector.doc)
      .def("get_discrete_state",
          overload_cast_explicit<const BasicVector<T>&, int>(
              &Context<T>::get_discrete_state),
          py_rvp::reference_internal, doc.Context.get_discrete_state.doc_1args)
      .def("num_abstract_states", &Context<T>::num_abstract_states,
          doc.Context.num_abstract_states.doc)
      .def("get_abstract_state",
          static_cast<const AbstractValues& (Context<T>::*)() const>(
              &Context<T>::get_abstract_state),
          py_rvp::reference_internal, doc.Context.get_abstract_state.doc_0args)
      .def(
          "get_abstract_state",
          [](const Context<T>* self, int index) -> auto& {
            return self->get_abstract_state().get_value(index);
          },
          py::arg("index"), py_rvp::reference_internal,
          doc.Context.get_abstract_state.doc_1args)
      .def("get_accuracy", &Context<T>::get_accuracy,
          doc.Context.get_accuracy.doc)
      .def("get_parameters", &Context<T>::get_parameters,
          py_rvp::reference_internal, doc.Context.get_parameters.doc)
      .def("num_numeric_parameter_groups",
          &Context<T>::num_numeric_parameter_groups,
          doc.Context.num_numeric_parameter_groups.doc)
      .def("get_numeric_parameter", &Context<T>::get_numeric_parameter,
          py::arg("index"), py_rvp::reference_internal,
          doc.Context.get_numeric_parameter.doc)
      .def("num_abstract_parameters", &Context<T>::num_abstract_parameters,
          doc.Context.num_abstract_parameters.doc)
      .def("get_abstract_parameter", &Context<T>::get_abstract_parameter,
          py::arg("index"), py_rvp::reference_internal,
          doc.Context.get_numeric_parameter.doc)
      // Bindings for the Context methods in the Doxygen group titled
      // "Methods for changing locally-stored values", placed in the same
      // order as the header file.
      .def("SetTime", &Context<T>::SetTime, py::arg("time_sec"),
          doc.Context.SetTime.doc)
      .def("SetContinuousState", &Context<T>::SetContinuousState,
          doc.Context.SetContinuousState.doc)
      .def("SetTimeAndContinuousState", &Context<T>::SetTimeAndContinuousState,
          doc.Context.SetTimeAndContinuousState.doc)
      .def("SetDiscreteState",
          overload_cast_explicit<void, const Eigen::Ref<const VectorX<T>>&>(
              &Context<T>::SetDiscreteState),
          py::arg("xd"), doc.Context.SetDiscreteState.doc_single_group)
      .def("SetDiscreteState",
          overload_cast_explicit<void, int,
              const Eigen::Ref<const VectorX<T>>&>(
              &Context<T>::SetDiscreteState),
          py::arg("group_index"), py::arg("xd"),
          doc.Context.SetDiscreteState.doc_select_one_group)
      .def("SetDiscreteState",
          overload_cast_explicit<void, const DiscreteValues<T>&>(
              &Context<T>::SetDiscreteState),
          py::arg("xd"), doc.Context.SetDiscreteState.doc_set_everything)
      .def(
          "SetAbstractState",
          [](py::object self, int index, py::object value) {
            // Use type erasure from Python bindings of Value[T].set_value.
            py::object abstract_value =
                self.attr("get_mutable_abstract_state")(index);
            abstract_value.attr("set_value")(value);
          },
          py::arg("index"), py::arg("value"), doc.Context.SetAbstractState.doc)
      // NOTE: SetTimeStateAndParametersFrom is bound below in
      // bind_context_methods_templated_on_a_secondary_scalar
      .def("SetAccuracy", &Context<T>::SetAccuracy, py::arg("accuracy"),
          doc.Context.SetAccuracy.doc)
      // Bindings for the Context methods in the Doxygen group titled
      // "Dangerous methods for changing locally-stored values", placed in the
      // same order as the header file.
      .def("get_mutable_state", &Context<T>::get_mutable_state,
          py_rvp::reference_internal, doc.Context.get_mutable_state.doc)
      .def("get_mutable_continuous_state",
          &Context<T>::get_mutable_continuous_state, py_rvp::reference_internal,
          doc.Context.get_mutable_continuous_state.doc)
      .def("get_mutable_continuous_state_vector",
          &Context<T>::get_mutable_continuous_state_vector,
          py_rvp::reference_internal,
          doc.Context.get_mutable_continuous_state_vector.doc)
      .def("get_mutable_discrete_state",
          overload_cast_explicit<DiscreteValues<T>&>(
              &Context<T>::get_mutable_discrete_state),
          py_rvp::reference_internal,
          doc.Context.get_mutable_discrete_state.doc_0args)
      .def("get_mutable_discrete_state_vector",
          &Context<T>::get_mutable_discrete_state_vector,
          py_rvp::reference_internal,
          doc.Context.get_mutable_discrete_state_vector.doc)
      .def("get_mutable_discrete_state",
          overload_cast_explicit<BasicVector<T>&, int>(
              &Context<T>::get_mutable_discrete_state),
          py_rvp::reference_internal,
          doc.Context.get_mutable_discrete_state.doc_1args)
      .def(
          "get_mutable_abstract_state",
          [](Context<T>* self) -> AbstractValues& {
            return self->get_mutable_abstract_state();
          },
          py_rvp::reference_internal,
          doc.Context.get_mutable_abstract_state.doc_0args)
      .def(
          "get_mutable_abstract_state",
          [](Context<T>* self, int index) -> AbstractValue& {
            return self->get_mutable_abstract_state().get_mutable_value(index);
          },
          py::arg("index"), py_rvp::reference_internal,
          doc.Context.get_mutable_abstract_state.doc_1args)
      .def("get_mutable_parameters", &Context<T>::get_mutable_parameters,
          py_rvp::reference_internal, doc.Context.get_mutable_parameters.doc)
      .def("get_mutable_numeric_parameter",
          &Context<T>::get_mutable_numeric_parameter, py::arg("index"),
          py_rvp::reference_internal,
          doc.Context.get_mutable_numeric_parameter.doc)
      .def("get_mutable_abstract_parameter",
          &Context<T>::get_mutable_abstract_parameter, py::arg("index"),
          py_rvp::reference_internal,
          doc.Context.get_mutable_abstract_parameter.doc)
      // Note: No bindings yet for "Advanced methods for changing
      //   locally-stored values"
      //
      // Bindings for the Context methods in the Doxygen group titled
      // "Miscellaneous public methods", placed in the same order as the
      // header file.
      // TODO(EricCousineau-TRI): Replace these with DefClone.
      .def("Clone", &Context<T>::Clone, doc.Context.Clone.doc)
      .def("__copy__", &Context<T>::Clone)
      .def("__deepcopy__", [](const Context<T>* self,
                               py::dict /* memo */) { return self->Clone(); })
      .def("__str__", &Context<T>::to_string, doc.Context.to_string.doc);
  return context_cls;
}

template <typename T, typename PyClass>
void DefineContextMethodsTemplatedOnASecondaryScalar(PyClass* context_cls) {
  PyClass& cls = *context_cls;
  type_visit(
      [&cls](auto dummy_u) {
        using U = decltype(dummy_u);
        cls  // BR
            .def(
                "SetStateAndParametersFrom",
                [](Context<T>* self, const Context<U>& source) {
                  self->SetStateAndParametersFrom(source);
                },
                py::arg("source"), doc.Context.SetStateAndParametersFrom.doc)
            .def(
                "SetTimeStateAndParametersFrom",
                [](Context<T>* self, const Context<U>& source) {
                  self->SetTimeStateAndParametersFrom(source);
                },
                py::arg("source"),
                doc.Context.SetTimeStateAndParametersFrom.doc);
      },
      CommonScalarPack{});
}

template <typename T>
void DefineLeafContext(py::module m) {
  DefineTemplateClassWithDefault<LeafContext<T>, Context<T>>(
      m, "LeafContext", GetPyParam<T>(), doc.LeafContext.doc);
}

template <typename T>
void DefineEventAndEventSubclasses(py::module m) {
  // Event mechanisms.
  DefineTemplateClassWithDefault<Event<T>>(
      m, "Event", GetPyParam<T>(), doc.Event.doc)
      .def("get_trigger_type", &Event<T>::get_trigger_type,
          doc.Event.get_trigger_type.doc);
  {
    auto cls = DefineTemplateClassWithDefault<PublishEvent<T>, Event<T>>(
        m, "PublishEvent", GetPyParam<T>(), doc.PublishEvent.doc);
    // Because Python doesn't offer static type checking to help remind the user
    // to return an EventStatus from an event handler function, we'll bind the
    // callback as optional<> to allow the user to omit a return statement.
    //
    // We'll also keep around both callbacks (with and without a System) because
    // the C++ rationale for injecting the System doesn't apply to Python so we
    // might as well not disturb any existing Python users of events.
    using Callback = std::function<std::optional<EventStatus>(
        const Context<T>&, const PublishEvent<T>&)>;
    using SystemCallback = std::function<std::optional<EventStatus>(
        const System<T>&, const Context<T>&, const PublishEvent<T>&)>;
    cls  // BR
        .def(py::init(WrapCallbacks([](const Callback& callback) {
          return std::make_unique<PublishEvent<T>>(
              [callback](const System<T>&, const Context<T>& context,
                  const PublishEvent<T>& event) {
                return callback(context, event)
                    .value_or(EventStatus::Succeeded());
              });
        })),
            py::arg("callback"),
            "Constructs a PublishEvent with the given callback function.")
        .def(py::init(WrapCallbacks([](const SystemCallback& system_callback) {
          return std::make_unique<PublishEvent<T>>(
              [system_callback](const System<T>& system,
                  const Context<T>& context, const PublishEvent<T>& event) {
                return system_callback(system, context, event)
                    .value_or(EventStatus::Succeeded());
              });
        })),
            py::arg("system_callback"),
            "Constructs a PublishEvent with the given callback function.")
        .def(py::init(WrapCallbacks(
                 [](const TriggerType& trigger_type, const Callback& callback) {
                   return std::make_unique<PublishEvent<T>>(trigger_type,
                       [callback](const System<T>&, const Context<T>& context,
                           const PublishEvent<T>& event) {
                         return callback(context, event)
                             .value_or(EventStatus::Succeeded());
                       });
                 })),
            py::arg("trigger_type"), py::arg("callback"),
            "Users should not be calling these")
        .def(py::init(WrapCallbacks([](const TriggerType& trigger_type,
                                        const SystemCallback& system_callback) {
          return std::make_unique<PublishEvent<T>>(trigger_type,
              [system_callback](const System<T>& system,
                  const Context<T>& context, const PublishEvent<T>& event) {
                return system_callback(system, context, event)
                    .value_or(EventStatus::Succeeded());
              });
        })),
            py::arg("trigger_type"), py::arg("system_callback"),
            "Users should not be calling these");
  }
  DefineTemplateClassWithDefault<DiscreteUpdateEvent<T>, Event<T>>(
      m, "DiscreteUpdateEvent", GetPyParam<T>(), doc.DiscreteUpdateEvent.doc);
  {
    auto cls =
        DefineTemplateClassWithDefault<UnrestrictedUpdateEvent<T>, Event<T>>(m,
            "UnrestrictedUpdateEvent", GetPyParam<T>(),
            doc.UnrestrictedUpdateEvent.doc);
    // Because Python doesn't offer static type checking to help remind the user
    // to return an EventStatus from an event handler function, we'll bind the
    // callback as optional<> to allow the user to omit a return statement.
    //
    // We'll also keep around both callbacks (with and without a System) because
    // the C++ rationale for injecting the System doesn't apply to Python so we
    // might as well not disturb any existing Python users of events.
    using Callback = std::function<std::optional<EventStatus>(
        const Context<T>&, const UnrestrictedUpdateEvent<T>&, State<T>*)>;
    using SystemCallback =
        std::function<std::optional<EventStatus>(const System<T>&,
            const Context<T>&, const UnrestrictedUpdateEvent<T>&, State<T>*)>;
    cls  // BR
        .def(py::init(WrapCallbacks([](const Callback& callback) {
          return std::make_unique<UnrestrictedUpdateEvent<T>>(
              [callback](const System<T>&, const Context<T>& context,
                  const UnrestrictedUpdateEvent<T>& event, State<T>* state) {
                return callback(context, event, state)
                    .value_or(EventStatus::Succeeded());
              });
        })),
            py::arg("callback"),
            "Constructs an UnrestrictedUpdateEvent with the given callback "
            "function.")
        .def(py::init(WrapCallbacks([](const SystemCallback& system_callback) {
          return std::make_unique<UnrestrictedUpdateEvent<T>>(
              [system_callback](const System<T>& system,
                  const Context<T>& context,
                  const UnrestrictedUpdateEvent<T>& event, State<T>* state) {
                return system_callback(system, context, event, state)
                    .value_or(EventStatus::Succeeded());
              });
        })),
            py::arg("system_callback"),
            "Constructs an UnrestrictedUpdateEvent with the given callback "
            "function.");
  }
}

template <typename T>
void DoDefineFrameworkDiagramBuilder(py::module m) {
  using internal::BuilderLifeSupport;
  DefineTemplateClassWithDefault<DiagramBuilder<T>>(m, "DiagramBuilder",
      GetPyParam<T>(), doc.DiagramBuilder.doc, std::nullopt, py::dynamic_attr())
      .def(py::init<>(), doc.DiagramBuilder.ctor.doc)
      .def(
          "AddSystem",
          [](DiagramBuilder<T>* self, System<T>& system) {
            // Using BuilderLifeSupport::stash makes the builder
            // temporarily immortal (uncollectible self cycle). This will be
            // resolved by the Build() step. See BuilderLifeSupport for
            // rationale.
            BuilderLifeSupport<T>::stash(self);
            // The C++ method doesn't offer a bare-pointer overload, only
            // shared_ptr. Because object lifetime is already handled by the
            // ref_cycle annotation below, we can pass the `system` as an
            // unowned shared_ptr.
            return self->AddSystem(make_unowned_shared_ptr_from_raw(&system));
          },
          py::arg("system"), internal::ref_cycle<1, 2>(), py_rvp::reference,
          doc.DiagramBuilder.AddSystem.doc)
      .def(
          "AddNamedSystem",
          [](DiagramBuilder<T>* self, std::string& name, System<T>& system) {
            // Using BuilderLifeSupport::stash makes the builder
            // temporarily immortal (uncollectible self cycle). This will be
            // resolved by the Build() step. See BuilderLifeSupport for
            // rationale.
            BuilderLifeSupport<T>::stash(self);
            // Ditto with "AddSystem" above for how we handle the `&system`.
            return self->AddNamedSystem(
                name, make_unowned_shared_ptr_from_raw(&system));
          },
          py::arg("name"), py::arg("system"), internal::ref_cycle<1, 3>(),
          py_rvp::reference, doc.DiagramBuilder.AddNamedSystem.doc)
      .def("RemoveSystem", &DiagramBuilder<T>::RemoveSystem, py::arg("system"),
          doc.DiagramBuilder.RemoveSystem.doc)
      .def("empty", &DiagramBuilder<T>::empty, doc.DiagramBuilder.empty.doc)
      .def("already_built", &DiagramBuilder<T>::already_built,
          doc.DiagramBuilder.already_built.doc)
      .def("GetSystems", &DiagramBuilder<T>::GetSystems,
          py_rvp::reference_internal, doc.DiagramBuilder.GetSystems.doc)
      .def("GetMutableSystems", &DiagramBuilder<T>::GetMutableSystems,
          py_rvp::reference_internal, doc.DiagramBuilder.GetMutableSystems.doc)
      .def("HasSubsystemNamed", &DiagramBuilder<T>::HasSubsystemNamed,
          py::arg("name"), doc.DiagramBuilder.HasSubsystemNamed.doc)
      .def("GetSubsystemByName", &DiagramBuilder<T>::GetSubsystemByName,
          py::arg("name"), py_rvp::reference_internal,
          doc.DiagramBuilder.GetSubsystemByName.doc)
      .def("GetMutableSubsystemByName",
          &DiagramBuilder<T>::GetMutableSubsystemByName, py::arg("name"),
          py_rvp::reference_internal,
          doc.DiagramBuilder.GetMutableSubsystemByName.doc)
      .def(
          "connection_map",
          [](DiagramBuilder<T>* self) {
            // N.B. This code is duplicated with Diagram's same-named function.
            // Keep the two copies in sync. The detailed unit test is written
            // against the Diagram copy of this function, not this one.
            py::dict out;
            py::object self_py = py::cast(self, py_rvp::reference);
            for (auto& [input_locator, output_locator] :
                self->connection_map()) {
              // Keep alive, ownership: `input_system_py` keeps `self` alive.
              py::object input_system_py = py::cast(
                  input_locator.first, py_rvp::reference_internal, self_py);
              py::object input_port_index_py = py::cast(input_locator.second);

              py::tuple input_locator_py(2);
              input_locator_py[0] = input_system_py;
              input_locator_py[1] = input_port_index_py;

              // Keep alive, ownership: `output_system_py` keeps `self` alive.
              py::object output_system_py = py::cast(
                  output_locator.first, py_rvp::reference_internal, self_py);
              py::object output_port_index_py = py::cast(output_locator.second);

              py::tuple output_locator_py(2);
              output_locator_py[0] = output_system_py;
              output_locator_py[1] = output_port_index_py;

              out[input_locator_py] = output_locator_py;
            }
            return out;
          },
          doc.DiagramBuilder.connection_map.doc)
      .def("Connect",
          py::overload_cast<const OutputPort<T>&, const InputPort<T>&>(
              &DiagramBuilder<T>::Connect),
          doc.DiagramBuilder.Connect.doc)
      .def("ExportInput", &DiagramBuilder<T>::ExportInput, py::arg("input"),
          py::arg("name") = kUseDefaultName, py_rvp::reference_internal,
          doc.DiagramBuilder.ExportInput.doc)
      .def("ConnectInput",
          py::overload_cast<std::string_view, const InputPort<T>&>(
              &DiagramBuilder<T>::ConnectInput),
          py::arg("diagram_port_name"), py::arg("input"),
          doc.DiagramBuilder.ConnectInput.doc_2args_diagram_port_name_input)
      .def("ConnectInput",
          py::overload_cast<InputPortIndex, const InputPort<T>&>(
              &DiagramBuilder<T>::ConnectInput),
          py::arg("diagram_port_index"), py::arg("input"),
          doc.DiagramBuilder.ConnectInput.doc_2args_diagram_port_index_input)
      .def("ConnectToSame", &DiagramBuilder<T>::ConnectToSame,
          py::arg("exemplar"), py::arg("dest"),
          doc.DiagramBuilder.ConnectToSame.doc)
      .def("ExportOutput", &DiagramBuilder<T>::ExportOutput, py::arg("output"),
          py::arg("name") = kUseDefaultName, py_rvp::reference_internal,
          doc.DiagramBuilder.ExportOutput.doc)
      .def("Disconnect", &DiagramBuilder<T>::Disconnect, py::arg("source"),
          py::arg("dest"), doc.DiagramBuilder.Disconnect.doc)
      .def(
          "Build",
          [](DiagramBuilder<T>* self) {
            // The c++ Build() step would pass life support to the
            // diagram. Instead of relying on its one-way, uncollectible
            // support here, abandon it in favor of the builder-diagram
            // ref_cycle invoked below. We can't have both; that would create
            // an uncollectible builder-diagram cycle and make those objects
            // immortal.
            BuilderLifeSupport<T>::abandon(self);
            return self->Build();
          },
          internal::ref_cycle<0, 1>(), doc.DiagramBuilder.Build.doc)
      .def(
          "BuildInto",
          [](DiagramBuilder<T>* self, Diagram<T>* target) {
            // The c++ BuildInto() step would pass life support to the
            // diagram. Instead of relying on its one-way, uncollectible
            // support here, abandon it in favor of the builder-diagram
            // ref_cycle invoked below. We can't have both; that would create
            // an uncollectible builder-diagram cycle and make those objects
            // immortal.
            BuilderLifeSupport<T>::abandon(self);
            self->BuildInto(target);
          },
          internal::ref_cycle<1, 2>(), py::arg("target"),
          doc.DiagramBuilder.BuildInto.doc)
      .def("IsConnectedOrExported", &DiagramBuilder<T>::IsConnectedOrExported,
          py::arg("port"), doc.DiagramBuilder.IsConnectedOrExported.doc)
      .def("num_input_ports", &DiagramBuilder<T>::num_input_ports,
          doc.DiagramBuilder.num_input_ports.doc)
      .def("num_output_ports", &DiagramBuilder<T>::num_output_ports,
          doc.DiagramBuilder.num_output_ports.doc);
}

// TODO(jwnimmer-tri) This function is just a grab-bag of several classes. We
// should split it up into smaller pieces.
template <typename T>
void DefineRemainingScalarDependentDefinitions(py::module m) {
  DefineTemplateClassWithDefault<OutputPort<T>>(m, "OutputPort",
      GetPyParam<T>(), doc.OutputPort.doc, std::nullopt, py::dynamic_attr())
      .def("size", &OutputPort<T>::size, doc.PortBase.size.doc)
      .def("get_data_type", &OutputPort<T>::get_data_type,
          doc.PortBase.get_data_type.doc)
      .def("get_index", &OutputPort<T>::get_index,
          doc.OutputPortBase.get_index.doc)
      .def("get_name", &OutputPort<T>::get_name, doc.PortBase.get_name.doc)
      .def(
          "Eval",
          [](const OutputPort<T>* self, const Context<T>& context) {
            return DoEval(self, context);
          },
          doc.OutputPort.Eval.doc)
      .def(
          "EvalAbstract",
          [](const OutputPort<T>* self, const Context<T>& c) {
            const auto& abstract = self->template Eval<AbstractValue>(c);
            return py::cast(&abstract);
          },
          py::arg("context"),
          "(Advanced.) Returns the value of this output port, typed "
          "as an AbstractValue. Most users should call Eval() instead. "
          "This method is only needed when the result will be passed "
          "into some other API that only accepts an AbstractValue.",
          py_rvp::reference_internal)
      .def(
          "EvalBasicVector",
          [](const OutputPort<T>* self, const Context<T>& c) {
            const auto& basic = self->template Eval<BasicVector<T>>(c);
            return py::cast(&basic);
          },
          py::arg("context"),
          "(Advanced.) Returns the value of this output port, typed "
          "as a BasicVector. Most users should call Eval() instead. "
          "This method is only needed when the result will be passed "
          "into some other API that only accepts a BasicVector.",
          py_rvp::reference_internal)
      .def("Allocate", &OutputPort<T>::Allocate, doc.OutputPort.Allocate.doc)
      .def(
          "get_system",
          [](const OutputPort<T>& self) {
            // The `class System` is not bound yet, so we must use a dynamic
            // return type to avoid referring to a class that doesn't exist.
            // TODO(jwnimmer-tri) With major surgery to the bindings we could
            // change the order of operations to work around this, but at the
            // moment that's too much churn to for the payoff.
            const System<T>& result = self.get_system();
            py::object self_py = py::cast(self, py_rvp::reference);
            return py::cast(&result, py_rvp::reference_internal, self_py);
          },
          doc.OutputPort.get_system.doc);

  DefineTemplateClassWithDefault<LeafOutputPort<T>, OutputPort<T>>(
      m, "LeafOutputPort", GetPyParam<T>(), doc.LeafOutputPort.doc)
      .def("cache_entry", &LeafOutputPort<T>::cache_entry,
          py_rvp::reference_internal, doc.LeafOutputPort.cache_entry.doc)
      .def("disable_caching_by_default",
          &LeafOutputPort<T>::disable_caching_by_default,
          doc.LeafOutputPort.disable_caching_by_default.doc);

  auto system_output = DefineTemplateClassWithDefault<SystemOutput<T>>(
      m, "SystemOutput", GetPyParam<T>(), doc.SystemOutput.doc);
  system_output
      .def("num_ports", &SystemOutput<T>::num_ports,
          doc.SystemOutput.num_ports.doc)
      .def("get_data", &SystemOutput<T>::get_data, py_rvp::reference_internal,
          doc.SystemOutput.get_data.doc)
      .def("get_vector_data", &SystemOutput<T>::get_vector_data,
          py_rvp::reference_internal, doc.SystemOutput.get_vector_data.doc);

  DefineTemplateClassWithDefault<InputPort<T>>(m, "InputPort", GetPyParam<T>(),
      doc.InputPort.doc, std::nullopt, py::dynamic_attr())
      .def("get_name", &InputPort<T>::get_name, doc.PortBase.get_name.doc)
      .def("GetFullDescription", &InputPort<T>::GetFullDescription,
          doc.PortBase.GetFullDescription.doc)
      .def("get_index", &InputPort<T>::get_index,
          doc.InputPortBase.get_index.doc)
      .def("get_data_type", &InputPort<T>::get_data_type,
          doc.PortBase.get_data_type.doc)
      .def("size", &InputPort<T>::size, doc.PortBase.size.doc)
      .def("ticket", &InputPort<T>::ticket, doc.PortBase.ticket.doc)
      .def(
          "Eval",
          [](const InputPort<T>* self, const Context<T>& context) {
            return DoEval(self, context);
            DRAKE_UNREACHABLE();
          },
          doc.InputPort.Eval.doc)
      .def("EvalAbstract", &InputPort<T>::template Eval<AbstractValue>,
          py::arg("context"),
          "(Advanced.) Returns the value of this input port, typed "
          "as an AbstractValue. Most users should call Eval() instead. "
          "This method is only needed when the result will be passed "
          "into some other API that only accepts an AbstractValue.",
          py_rvp::reference_internal)
      .def(
          "EvalBasicVector",
          [](const InputPort<T>* self, const Context<T>& c) {
            const auto& basic = self->template Eval<BasicVector<T>>(c);
            return py::cast(&basic);
          },
          py::arg("context"),
          "(Advanced.) Returns the value of this input port, typed "
          "as a BasicVector. Most users should call Eval() instead. "
          "This method is only needed when the result will be passed "
          "into some other API that only accepts a BasicVector.",
          py_rvp::reference_internal)
      // For FixValue, treat an already-erased AbstractValue specially ...
      .def(
          "FixValue",
          [](const InputPort<T>* self, Context<T>* context,
              const AbstractValue& value) {
            FixedInputPortValue& result = self->FixValue(context, value);
            return &result;
          },
          py::arg("context"), py::arg("value"), py_rvp::reference,
          // Keep alive, ownership: `return` keeps `context` alive.
          py::keep_alive<0, 2>(), doc.InputPort.FixValue.doc)
      // ... but then for anything not yet erased, use set_value to copy.
      .def(
          "FixValue",
          [](const InputPort<T>* self, Context<T>* context,
              const py::object& value) {
            const auto& system = self->get_system();
            // Allocate is a bit wasteful, but FixValue is already expensive.
            std::unique_ptr<AbstractValue> storage =
                system.AllocateInputAbstract(*self);
            py::cast(storage.get(), py_rvp::reference).attr("set_value")(value);
            FixedInputPortValue& result = self->FixValue(context, *storage);
            return &result;
          },
          py::arg("context"), py::arg("value"), py_rvp::reference,
          // Keep alive, ownership: `return` keeps `context` alive.
          py::keep_alive<0, 2>(), doc.InputPort.FixValue.doc)
      .def("HasValue", &InputPort<T>::HasValue, py::arg("context"),
          doc.InputPort.HasValue.doc)
      .def("Allocate", &InputPort<T>::Allocate, doc.InputPortBase.Allocate.doc)
      .def(
          "get_system",
          [](const InputPort<T>& self) {
            // The `class System` is not bound yet, so we must use a dynamic
            // return type to avoid referring to a class that doesn't exist.
            // TODO(jwnimmer-tri) With major surgery to the bindings we could
            // change the order of operations to work around this, but at the
            // moment that's too much churn to for the payoff.
            const System<T>& result = self.get_system();
            py::object self_py = py::cast(self, py_rvp::reference);
            return py::cast(&result, py_rvp::reference_internal, self_py);
          },

          doc.InputPort.get_system.doc);

  // TODO(russt): Bind relevant WitnessFunction methods.  This is the
  // minimal binding required to support DeclareWitnessFunction.
  DefineTemplateClassWithDefault<WitnessFunction<T>>(
      m, "WitnessFunction", GetPyParam<T>(), doc.WitnessFunction.doc);
}  // NOLINT(readability/fn_size)

template <typename T>
void DefineParameters(py::module m) {
  auto parameters = DefineTemplateClassWithDefault<Parameters<T>>(
      m, "Parameters", GetPyParam<T>(), doc.Parameters.doc);
  DefClone(&parameters);
  parameters  // BR
      .def(py::init<>(), doc.Parameters.ctor.doc_0args)
      .def(py::init([](const std::vector<const BasicVector<T>*>& numeric,
                        const std::vector<const AbstractValue*>& abstract) {
        return std::make_unique<Parameters<T>>(
            CloneVectorOfPointers(numeric), CloneVectorOfPointers(abstract));
      }),
          py::arg("numeric"), py::arg("abstract"),
          doc.Parameters.ctor.doc_2args_numeric_abstract)
      .def(py::init([](const std::vector<const BasicVector<T>*>& numeric) {
        return std::make_unique<Parameters<T>>(CloneVectorOfPointers(numeric));
      }),
          py::arg("numeric"), doc.Parameters.ctor.doc_1args_numeric)
      .def(py::init([](const std::vector<const AbstractValue*>& abstract) {
        return std::make_unique<Parameters<T>>(CloneVectorOfPointers(abstract));
      }),
          py::arg("abstract"), doc.Parameters.ctor.doc_1args_abstract)
      .def(py::init([](const BasicVector<T>& vec) {
        return std::make_unique<Parameters<T>>(vec.Clone());
      }),
          py::arg("vec"), doc.Parameters.ctor.doc_1args_vec)
      .def(py::init([](const AbstractValue& value) {
        return std::make_unique<Parameters<T>>(value.Clone());
      }),
          py::arg("value"), doc.Parameters.ctor.doc_1args_value)
      .def("num_numeric_parameter_groups",
          &Parameters<T>::num_numeric_parameter_groups,
          doc.Parameters.num_numeric_parameter_groups.doc)
      .def("num_abstract_parameters", &Parameters<T>::num_abstract_parameters,
          doc.Parameters.num_abstract_parameters.doc)
      .def("get_numeric_parameter", &Parameters<T>::get_numeric_parameter,
          py_rvp::reference_internal, py::arg("index"),
          doc.Parameters.get_numeric_parameter.doc)
      .def("get_mutable_numeric_parameter",
          &Parameters<T>::get_mutable_numeric_parameter,
          py_rvp::reference_internal, py::arg("index"),
          doc.Parameters.get_mutable_numeric_parameter.doc)
      .def("get_numeric_parameters", &Parameters<T>::get_numeric_parameters,
          py_rvp::reference_internal, doc.Parameters.get_numeric_parameters.doc)
      .def(
          "set_numeric_parameters",
          [](Parameters<T>& self, const DiscreteValues<T>& numeric_params) {
            // TODO(eric.cousineau): Should this C++ code constrain the number
            // of parameters???
            //
            // WARNING: This will DELETE the existing parameters. See C++
            // `AddValueInstantiation` for more information.
            self.set_numeric_parameters(numeric_params.Clone());
          },
          py::arg("numeric_params"), doc.Parameters.set_numeric_parameters.doc)
      .def(
          "get_abstract_parameter",
          [](const Parameters<T>* self, int index) -> auto& {
            return self->get_abstract_parameter(index);
          },
          py_rvp::reference_internal, py::arg("index"),
          doc.Parameters.get_abstract_parameter.doc_1args_index)
      .def(
          "get_mutable_abstract_parameter",
          [](Parameters<T>* self, int index) -> AbstractValue& {
            return self->get_mutable_abstract_parameter(index);
          },
          py_rvp::reference_internal, py::arg("index"),
          doc.Parameters.get_mutable_abstract_parameter.doc_1args_index)
      .def("get_abstract_parameters", &Parameters<T>::get_abstract_parameters,
          py_rvp::reference_internal,
          doc.Parameters.get_abstract_parameters.doc)
      .def(
          "set_abstract_parameters",
          [](Parameters<T>& self, const AbstractValues& abstract_params) {
            // WARNING: This will DELETE the existing parameters. See C++
            // `AddValueInstantiation` for more information.
            self.set_abstract_parameters(abstract_params.Clone());
          },
          py::arg("abstract_params"),
          doc.Parameters.set_abstract_parameters.doc)
      .def(
          "SetFrom",
          [](Parameters<T>* self, const Parameters<double>& other) {
            self->SetFrom(other);
          },
          doc.Parameters.SetFrom.doc);
}

template <typename T>
void DefineState(py::module m) {
  DefineTemplateClassWithDefault<State<T>>(
      m, "State", GetPyParam<T>(), doc.State.doc)
      .def(py::init<>(), doc.State.ctor.doc)
      .def("get_continuous_state", &State<T>::get_continuous_state,
          py_rvp::reference_internal, doc.State.get_continuous_state.doc)
      .def("get_mutable_continuous_state",
          &State<T>::get_mutable_continuous_state, py_rvp::reference_internal,
          doc.State.get_mutable_continuous_state.doc)
      .def("get_discrete_state",
          overload_cast_explicit<const DiscreteValues<T>&>(
              &State<T>::get_discrete_state),
          py_rvp::reference_internal, doc.State.get_discrete_state.doc)
      .def("get_mutable_discrete_state",
          overload_cast_explicit<DiscreteValues<T>&>(
              &State<T>::get_mutable_discrete_state),
          py_rvp::reference_internal, doc.State.get_mutable_discrete_state.doc)
      .def("get_mutable_discrete_state",
          overload_cast_explicit<BasicVector<T>&, int>(
              &State<T>::get_mutable_discrete_state),
          py::arg("index"), py_rvp::reference_internal,
          doc.State.get_mutable_discrete_state.doc)
      .def("get_abstract_state",
          static_cast<const AbstractValues& (State<T>::*)() const>(
              &State<T>::get_abstract_state),
          py_rvp::reference_internal, doc.State.get_abstract_state.doc)
      .def(
          "get_abstract_state",
          [](State<T>* self, int index) -> const AbstractValue& {
            return self->get_abstract_state().get_value(index);
          },
          py::arg("index"), py_rvp::reference_internal,
          doc.State.get_abstract_state.doc)
      .def(
          "get_mutable_abstract_state",
          [](State<T>* self) -> AbstractValues& {
            return self->get_mutable_abstract_state();
          },
          py_rvp::reference_internal,
          doc.State.get_mutable_abstract_state.doc_0args)
      .def(
          "get_mutable_abstract_state",
          [](State<T>* self, int index) -> AbstractValue& {
            return self->get_mutable_abstract_state().get_mutable_value(index);
          },
          py::arg("index"), py_rvp::reference_internal,
          doc.State.get_mutable_abstract_state.doc_1args);
}

template <typename T>
void DefineContinuousState(py::module m) {
  auto continuous_state = DefineTemplateClassWithDefault<ContinuousState<T>>(
      m, "ContinuousState", GetPyParam<T>(), doc.ContinuousState.doc);
  DefClone(&continuous_state);
  continuous_state  // BR
      .def(py::init<>(), doc.ContinuousState.ctor.doc_0args)
      // In the next pair of overloads, we'll try matching on BasicVector in
      // order to preserve its subtype across cloning. In the subsequent pair
      // of overloads, we'll also allow VectorBase.
      .def(py::init([](const BasicVector<T>& state) {
        return std::make_unique<ContinuousState<T>>(state.Clone());
      }),
          py::arg("state"), doc.ContinuousState.ctor.doc_1args_state)
      .def(py::init([](const BasicVector<T>& state, int num_q, int num_v,
                        int num_z) {
        return std::make_unique<ContinuousState<T>>(
            state.Clone(), num_q, num_v, num_z);
      }),
          py::arg("state"), py::arg("num_q"), py::arg("num_v"),
          py::arg("num_z"),
          doc.ContinuousState.ctor.doc_4args_state_num_q_num_v_num_z)
      .def(py::init([](const VectorBase<T>& state) {
        return std::make_unique<ContinuousState<T>>(
            std::make_unique<BasicVector<T>>(state.CopyToVector()));
      }),
          py::arg("state"), doc.ContinuousState.ctor.doc_1args_state)
      .def(py::init(
               [](const VectorBase<T>& state, int num_q, int num_v, int num_z) {
                 return std::make_unique<ContinuousState<T>>(
                     std::make_unique<BasicVector<T>>(state.CopyToVector()),
                     num_q, num_v, num_z);
               }),
          py::arg("state"), py::arg("num_q"), py::arg("num_v"),
          py::arg("num_z"),
          doc.ContinuousState.ctor.doc_4args_state_num_q_num_v_num_z)
      .def("size", &ContinuousState<T>::size, doc.ContinuousState.size.doc)
      .def("num_q", &ContinuousState<T>::num_q, doc.ContinuousState.num_q.doc)
      .def("num_v", &ContinuousState<T>::num_v, doc.ContinuousState.num_v.doc)
      .def("num_z", &ContinuousState<T>::num_z, doc.ContinuousState.num_z.doc)
      .def("__getitem__",
          overload_cast_explicit<const T&, std::size_t>(
              &ContinuousState<T>::operator[]),
          doc.ContinuousState.operator_array.doc)
      .def(
          "__setitem__",
          [](ContinuousState<T>& self, int index, T& value) {
            self[index] = value;
          },
          doc.ContinuousState.operator_array.doc)
      .def("get_vector", &ContinuousState<T>::get_vector,
          py_rvp::reference_internal, doc.ContinuousState.get_vector.doc)
      .def("get_mutable_vector", &ContinuousState<T>::get_mutable_vector,
          py_rvp::reference_internal,
          doc.ContinuousState.get_mutable_vector.doc)
      .def("get_generalized_position",
          &ContinuousState<T>::get_generalized_position,
          py_rvp::reference_internal,
          doc.ContinuousState.get_generalized_position.doc)
      .def("get_mutable_generalized_position",
          &ContinuousState<T>::get_mutable_generalized_position,
          py_rvp::reference_internal,
          doc.ContinuousState.get_mutable_generalized_position.doc)
      .def("get_generalized_velocity",
          &ContinuousState<T>::get_generalized_velocity,
          py_rvp::reference_internal,
          doc.ContinuousState.get_generalized_velocity.doc)
      .def("get_mutable_generalized_velocity",
          &ContinuousState<T>::get_mutable_generalized_velocity,
          py_rvp::reference_internal,
          doc.ContinuousState.get_mutable_generalized_velocity.doc)
      .def("get_misc_continuous_state",
          &ContinuousState<T>::get_misc_continuous_state,
          py_rvp::reference_internal,
          doc.ContinuousState.get_misc_continuous_state.doc)
      .def("get_mutable_misc_continuous_state",
          &ContinuousState<T>::get_mutable_misc_continuous_state,
          py_rvp::reference_internal,
          doc.ContinuousState.get_mutable_misc_continuous_state.doc)
      .def(
          "SetFrom",
          [](ContinuousState<T>* self, const ContinuousState<double>& other) {
            self->SetFrom(other);
          },
          doc.ContinuousState.SetFrom.doc)
      .def("SetFromVector", &ContinuousState<T>::SetFromVector,
          py::arg("value"), doc.ContinuousState.SetFromVector.doc)
      .def("CopyToVector", &ContinuousState<T>::CopyToVector,
          doc.ContinuousState.CopyToVector.doc);
}

template <typename T>
void DefineDiscreteValues(py::module m) {
  auto discrete_values = DefineTemplateClassWithDefault<DiscreteValues<T>>(
      m, "DiscreteValues", GetPyParam<T>(), doc.DiscreteValues.doc);
  DefClone(&discrete_values);
  discrete_values
      .def(py::init([](const BasicVector<T>& datum) {
        return std::make_unique<DiscreteValues<T>>(datum.Clone());
      }),
          py::arg("datum"), doc.DiscreteValues.ctor.doc_1args_datum)
      .def(py::init([](const std::vector<const BasicVector<T>*>& data) {
        return std::make_unique<DiscreteValues<T>>(CloneVectorOfPointers(data));
      }),
          py::arg("data"), doc.DiscreteValues.ctor.doc_1args_data)
      .def(py::init<>(), doc.DiscreteValues.ctor.doc_0args)
      .def("num_groups", &DiscreteValues<T>::num_groups,
          doc.DiscreteValues.num_groups.doc)
      .def("size", &DiscreteValues<T>::size, doc.DiscreteValues.size.doc)
      .def("get_data", &DiscreteValues<T>::get_data, py_rvp::reference_internal,
          doc.DiscreteValues.get_data.doc)
      .def("set_value",
          overload_cast_explicit<void, const Eigen::Ref<const VectorX<T>>&>(
              &DiscreteValues<T>::set_value),
          py::arg("value"), doc.DiscreteValues.set_value.doc_1args)
      .def("value",
          overload_cast_explicit<const VectorX<T>&, int>(
              &DiscreteValues<T>::value),
          return_value_policy_for_scalar_type<T>(), py::arg("index") = 0,
          doc.DiscreteValues.value.doc_1args)
      .def("get_vector",
          overload_cast_explicit<const BasicVector<T>&, int>(
              &DiscreteValues<T>::get_vector),
          py_rvp::reference_internal, py::arg("index") = 0,
          doc.DiscreteValues.get_vector.doc_1args)
      .def("get_mutable_vector",
          overload_cast_explicit<BasicVector<T>&, int>(
              &DiscreteValues<T>::get_mutable_vector),
          py_rvp::reference_internal, py::arg("index") = 0,
          doc.DiscreteValues.get_mutable_vector.doc_1args)
      .def("set_value",
          overload_cast_explicit<void, int,
              const Eigen::Ref<const VectorX<T>>&>(
              &DiscreteValues<T>::set_value),
          py::arg("index"), py::arg("value"),
          doc.DiscreteValues.set_value.doc_2args)
      .def(
          "get_value",
          [](const DiscreteValues<T>* self,
              int index) -> Eigen::Ref<const VectorX<T>> {
            return self->get_value(index);
          },
          return_value_policy_for_scalar_type<T>(), py::arg("index") = 0,
          doc.DiscreteValues.get_value.doc_1args)
      .def(
          "get_mutable_value",
          [](DiscreteValues<T>* self, int index) -> Eigen::Ref<VectorX<T>> {
            return self->get_mutable_value(index);
          },
          // N.B. We explicitly want a failure when T != double due to #8116.
          py_rvp::reference_internal, py::arg("index") = 0,
          doc.DiscreteValues.get_mutable_value.doc_1args)
      .def(
          "SetFrom",
          [](DiscreteValues<T>* self, const DiscreteValues<double>& other) {
            self->SetFrom(other);
          },
          doc.DiscreteValues.SetFrom.doc)
      .def("__getitem__",
          overload_cast_explicit<const T&, std::size_t>(
              &DiscreteValues<T>::operator[]),
          doc.DiscreteValues.operator_array.doc_1args_idx_const)
      .def(
          "__setitem__",
          [](DiscreteValues<T>& self, int index, T& value) {
            self[index] = value;
          },
          doc.DiscreteValues.operator_array.doc_1args_idx_nonconst);
}
}  // namespace

void DefineFrameworkDiagramBuilder(py::module m) {
  type_visit(
      [m](auto dummy) {
        using T = decltype(dummy);
        DoDefineFrameworkDiagramBuilder<T>(m);
      },
      CommonScalarPack{});
}

void DefineFrameworkPySemantics(py::module m) {
  // This list of calls to helpers must remain in topological dependency order.
  DoScalarIndependentDefinitions(m);
  type_visit(
      [m](auto dummy) {
        using T = decltype(dummy);
        DefineContinuousState<T>(m);
        DefineDiscreteValues<T>(m);
        DefineState<T>(m);
        DefineParameters<T>(m);
      },
      CommonScalarPack{});
  {
    // The Context classes form a dependency cycle due to built-in scalar
    // conversion, so we must declare all of them prior to defining any of them.
    auto cls_context_double = DefineContext<double>(m);
    auto cls_context_autodiff = DefineContext<AutoDiffXd>(m);
    auto cls_context_expression = DefineContext<symbolic::Expression>(m);
    DefineContextMethodsTemplatedOnASecondaryScalar<double>(
        &cls_context_double);
    DefineContextMethodsTemplatedOnASecondaryScalar<AutoDiffXd>(
        &cls_context_autodiff);
    DefineContextMethodsTemplatedOnASecondaryScalar<symbolic::Expression>(
        &cls_context_expression);
  }
  type_visit(
      [m](auto dummy) {
        using T = decltype(dummy);
        DefineLeafContext<T>(m);
        DefineEventAndEventSubclasses<T>(m);
        DefineRemainingScalarDependentDefinitions<T>(m);
      },
      CommonScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
