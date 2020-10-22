#include "drake/bindings/pydrake/systems/framework_py_semantics.h"

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/common/type_safe_index_pybind.h"
#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_output_port.h"
#include "drake/systems/framework/system_output.h"

using std::string;
using std::unique_ptr;
using std::vector;

namespace drake {
namespace pydrake {

namespace {
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
      py::object value_ref = py::cast(&abstract);
      return value_ref.attr("get_value")();
    }
  }
  DRAKE_UNREACHABLE();
}
}  // namespace

const char* doc_fix_input_port_deprecation =
    "Use input_port.FixValue() instead of context.FixInputPort(). This will be "
    "removed from Drake on or after 2021-01-01.";

void DefineFrameworkPySemantics(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  constexpr auto& doc = pydrake_doc.drake.systems;

  py::class_<UseDefaultName> use_default_name_cls(
      m, "UseDefaultName", doc.UseDefaultName.doc);
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
  BindTypeSafeIndex<DiscreteStateIndex>(m, "DiscreteStateIndex");
  BindTypeSafeIndex<AbstractStateIndex>(m, "AbstractStateIndex");
  BindTypeSafeIndex<NumericParameterIndex>(m, "NumericParameterIndex");
  BindTypeSafeIndex<AbstractParameterIndex>(m, "AbstractParameterIndex");

  py::class_<FixedInputPortValue>(
      m, "FixedInputPortValue", doc.FixedInputPortValue.doc)
      .def("GetMutableData", &FixedInputPortValue::GetMutableData,
          py_rvp::reference_internal,
          doc.FixedInputPortValue.GetMutableData.doc);

  using AbstractValuePtrList = vector<unique_ptr<AbstractValue>>;
  // N.B. `AbstractValues` provides the ability to reference non-owned values,
  // without copying them. For consistency with other model-value Python
  // bindings, only the ownership variant is exposed.
  py::class_<AbstractValues> abstract_values(
      m, "AbstractValues", doc.AbstractValues.doc);
  DefClone(&abstract_values);
  abstract_values  // BR
      .def(py::init<>(), doc.AbstractValues.ctor.doc_0args)
      .def(py::init<AbstractValuePtrList>(), doc.AbstractValues.ctor.doc_1args)
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

  auto event_data = py::class_<EventData>(m, "EventData", doc.EventData.doc);
  DefClone(&event_data);
  py::class_<PeriodicEventData, EventData>(
      m, "PeriodicEventData", doc.PeriodicEventData.doc)
      .def("period_sec", &PeriodicEventData::period_sec,
          doc.PeriodicEventData.period_sec.doc)
      .def("offset_sec", &PeriodicEventData::offset_sec,
          doc.PeriodicEventData.offset_sec.doc);

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

    DefCopyAndDeepCopy(&cls);
    cls  // BR
        .def_static("DidNothing", &Class::DidNothing, cls_doc.DidNothing.doc)
        .def_static("Succeeded", &Class::Succeeded, cls_doc.Succeeded.doc)
        .def_static("ReachedTermination", &Class::ReachedTermination,
            py::arg("system"), py::arg("message"),
            cls_doc.ReachedTermination.doc)
        .def_static("Failed", &Class::Failed, py::arg("system"),
            py::arg("message"), cls_doc.Failed.doc)
        .def("severity", &Class::severity, cls_doc.severity.doc)
        .def("system", &Class::system, py_rvp::reference, cls_doc.system.doc)
        .def("message", &Class::message, cls_doc.message.doc)
        .def("KeepMoreSevere", &Class::KeepMoreSevere, py::arg("candidate"),
            cls_doc.KeepMoreSevere.doc);
  }

  // N.B. Capturing `&doc` should not be required; workaround per #9600.
  auto bind_common_scalar_types = [m, &doc](auto dummy) {
    using T = decltype(dummy);
    auto context_cls = DefineTemplateClassWithDefault<Context<T>>(
        m, "Context", GetPyParam<T>(), doc.Context.doc);
    context_cls
        // Bindings for Context methods inherited from ContextBase.
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
            doc.ContextBase.is_cache_frozen.doc)
        // TODO(russt): Add remaining methods from ContextBase here.
        // Bindings for the Context methods in the Doxygen group titled
        // "Accessors for locally-stored values", placed in the same order
        // as the header file.
        .def("get_time", &Context<T>::get_time, doc.Context.get_time.doc)
        .def("get_state", &Context<T>::get_state, py_rvp::reference_internal,
            doc.Context.get_state.doc)
        .def("is_stateless", &Context<T>::is_stateless,
            doc.Context.is_stateless.doc)
        .def("has_only_continuous_state",
            &Context<T>::has_only_continuous_state,
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
            &Context<T>::get_continuous_state_vector,
            py_rvp::reference_internal,
            doc.Context.get_continuous_state_vector.doc)
        .def("num_discrete_state_groups",
            &Context<T>::num_discrete_state_groups,
            doc.Context.num_discrete_state_groups.doc)
        .def("get_discrete_state",
            overload_cast_explicit<const DiscreteValues<T>&>(
                &Context<T>::get_discrete_state),
            py_rvp::reference_internal,
            doc.Context.get_discrete_state.doc_0args)
        .def("get_discrete_state_vector",
            &Context<T>::get_discrete_state_vector, py_rvp::reference_internal,
            doc.Context.get_discrete_state_vector.doc)
        .def("get_discrete_state",
            overload_cast_explicit<const BasicVector<T>&, int>(
                &Context<T>::get_discrete_state),
            py_rvp::reference_internal,
            doc.Context.get_discrete_state.doc_1args)
        .def("num_abstract_states", &Context<T>::num_abstract_states,
            doc.Context.num_abstract_states.doc)
        .def("get_abstract_state",
            static_cast<const AbstractValues& (Context<T>::*)() const>(
                &Context<T>::get_abstract_state),
            py_rvp::reference_internal,
            doc.Context.get_abstract_state.doc_0args)
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
        .def("SetTimeAndContinuousState",
            &Context<T>::SetTimeAndContinuousState,
            doc.Context.SetTimeAndContinuousState.doc)
        .def("SetDiscreteState",
            overload_cast_explicit<void, const Eigen::Ref<const VectorX<T>>&>(
                &Context<T>::SetDiscreteState),
            py::arg("xd"), doc.Context.SetDiscreteState.doc_1args)
        .def("SetDiscreteState",
            overload_cast_explicit<void, int,
                const Eigen::Ref<const VectorX<T>>&>(
                &Context<T>::SetDiscreteState),
            py::arg("group_index"), py::arg("xd"),
            doc.Context.SetDiscreteState.doc_2args)
        .def(
            "SetAbstractState",
            [](py::object self, int index, py::object value) {
              // Use type erasure from Python bindings of Value[T].set_value.
              py::object abstract_value =
                  self.attr("get_mutable_abstract_state")(index);
              abstract_value.attr("set_value")(value);
            },
            py::arg("index"), py::arg("value"),
            doc.Context.SetAbstractState.doc);
    // NOTE: SetTimeStateAndParametersFrom is bound below in
    // bind_context_methods_templated_on_a_secondary_scalar
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    context_cls
        .def(
            "FixInputPort",
            [](Context<T>* self, int index, const BasicVector<T>& vec) {
              WarnDeprecated(doc_fix_input_port_deprecation);
              return &self->FixInputPort(index, vec);
            },
            py::arg("index"), py::arg("vec"), py_rvp::reference_internal,
            doc.Context.FixInputPort.doc_deprecated_deprecated_2args_index_vec)
        .def(
            "FixInputPort",
            [](Context<T>* self, int index, unique_ptr<AbstractValue> value) {
              WarnDeprecated(doc_fix_input_port_deprecation);
              return &self->FixInputPort(index, std::move(value));
            },
            py::arg("index"), py::arg("value"), py_rvp::reference_internal,
            // Keep alive, ownership: `AbstractValue` keeps `self` alive.
            py::keep_alive<3, 1>(), doc.ContextBase.FixInputPort.doc)
        .def(
            "FixInputPort",
            [](Context<T>* self, int index,
                const Eigen::Ref<const VectorX<T>>& data) {
              WarnDeprecated(doc_fix_input_port_deprecation);
              return &self->FixInputPort(index, data);
            },
            py::arg("index"), py::arg("data"), py_rvp::reference_internal,
            doc.Context.FixInputPort
                .doc_deprecated_deprecated_2args_index_data);
#pragma GCC diagnostic pop
    context_cls
        .def("SetAccuracy", &Context<T>::SetAccuracy, py::arg("accuracy"),
            doc.Context.SetAccuracy.doc)
        // Bindings for the Context methods in the Doxygen group titled
        // "Dangerous methods for changing locally-stored values", placed in the
        // same order as the header file.
        .def("get_mutable_state", &Context<T>::get_mutable_state,
            py_rvp::reference_internal, doc.Context.get_mutable_state.doc)
        .def("get_mutable_continuous_state",
            &Context<T>::get_mutable_continuous_state,
            py_rvp::reference_internal,
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
              return self->get_mutable_abstract_state().get_mutable_value(
                  index);
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

    auto bind_context_methods_templated_on_a_secondary_scalar =
        [m, &doc, &context_cls](auto dummy_u) {
          using U = decltype(dummy_u);
          context_cls.def(
              "SetTimeStateAndParametersFrom",
              [](Context<T>* self, const Context<U>& source) {
                self->SetTimeStateAndParametersFrom(source);
              },
              py::arg("source"), doc.Context.SetTimeStateAndParametersFrom.doc);
        };
    type_visit(bind_context_methods_templated_on_a_secondary_scalar,
        CommonScalarPack{});

    DefineTemplateClassWithDefault<LeafContext<T>, Context<T>>(
        m, "LeafContext", GetPyParam<T>(), doc.LeafContext.doc);

    // Event mechanisms.
    DefineTemplateClassWithDefault<Event<T>>(
        m, "Event", GetPyParam<T>(), doc.Event.doc)
        .def("get_trigger_type", &Event<T>::get_trigger_type,
            doc.Event.get_trigger_type.doc);
    DefineTemplateClassWithDefault<PublishEvent<T>, Event<T>>(
        m, "PublishEvent", GetPyParam<T>(), doc.PublishEvent.doc)
        .def(py::init(WrapCallbacks(
                 [](const typename PublishEvent<T>::PublishCallback& callback) {
                   return std::make_unique<PublishEvent<T>>(callback);
                 })),
            py::arg("callback"), doc.PublishEvent.ctor.doc_1args)
        .def(
            py::init(WrapCallbacks(
                [](const TriggerType& trigger_type,
                    const typename PublishEvent<T>::PublishCallback& callback) {
                  return std::make_unique<PublishEvent<T>>(
                      trigger_type, callback);
                })),
            py::arg("trigger_type"), py::arg("callback"),
            "Users should not be calling these");
    DefineTemplateClassWithDefault<DiscreteUpdateEvent<T>, Event<T>>(
        m, "DiscreteUpdateEvent", GetPyParam<T>(), doc.DiscreteUpdateEvent.doc);
    DefineTemplateClassWithDefault<UnrestrictedUpdateEvent<T>, Event<T>>(m,
        "UnrestrictedUpdateEvent", GetPyParam<T>(),
        doc.UnrestrictedUpdateEvent.doc)
        .def(py::init(
                 WrapCallbacks([](const typename UnrestrictedUpdateEvent<
                                   T>::UnrestrictedUpdateCallback& callback) {
                   return std::make_unique<UnrestrictedUpdateEvent<T>>(
                       callback);
                 })),
            py::arg("callback"), doc.UnrestrictedUpdateEvent.ctor.doc_1args);

    // Glue mechanisms.
    DefineTemplateClassWithDefault<DiagramBuilder<T>>(
        m, "DiagramBuilder", GetPyParam<T>(), doc.DiagramBuilder.doc)
        .def(py::init<>(), doc.DiagramBuilder.ctor.doc)
        .def(
            "AddSystem",
            [](DiagramBuilder<T>* self, unique_ptr<System<T>> system) {
              return self->AddSystem(std::move(system));
            },
            py::arg("system"),
            // Keep alive, ownership: `system` keeps `self` alive.
            py::keep_alive<2, 1>(), doc.DiagramBuilder.AddSystem.doc)
        .def("empty", &DiagramBuilder<T>::empty, doc.DiagramBuilder.empty.doc)
        .def(
            "GetSystems",
            [](DiagramBuilder<T>* self) {
              py::list out;
              py::object self_py = py::cast(self, py_rvp::reference);
              for (const auto* system : self->GetSystems()) {
                py::object system_py = py::cast(system, py_rvp::reference);
                // Keep alive, ownership: `system` keeps `self` alive.
                py_keep_alive(system_py, self_py);
                out.append(system_py);
              }
              return out;
            },
            doc.DiagramBuilder.GetSystems.doc)
        .def(
            "GetMutableSystems",
            [](DiagramBuilder<T>* self) {
              py::list out;
              py::object self_py = py::cast(self, py_rvp::reference);
              for (auto* system : self->GetMutableSystems()) {
                py::object system_py = py::cast(system, py_rvp::reference);
                // Keep alive, ownership: `system` keeps `self` alive.
                py_keep_alive(system_py, self_py);
                out.append(system_py);
              }
              return out;
            },
            doc.DiagramBuilder.GetMutableSystems.doc)
        .def("Connect",
            py::overload_cast<const OutputPort<T>&, const InputPort<T>&>(
                &DiagramBuilder<T>::Connect),
            doc.DiagramBuilder.Connect.doc)
        .def("ExportInput", &DiagramBuilder<T>::ExportInput, py::arg("input"),
            py::arg("name") = kUseDefaultName, py_rvp::reference_internal,
            doc.DiagramBuilder.ExportInput.doc)
        .def("ExportOutput", &DiagramBuilder<T>::ExportOutput,
            py::arg("output"), py::arg("name") = kUseDefaultName,
            py_rvp::reference_internal, doc.DiagramBuilder.ExportOutput.doc)
        .def("Build", &DiagramBuilder<T>::Build,
            // Keep alive, ownership (tr.): `return` keeps `self` alive.
            py::keep_alive<1, 0>(), doc.DiagramBuilder.Build.doc)
        .def("BuildInto", &DiagramBuilder<T>::BuildInto, py::arg("target"),
            // Keep alive, ownership (tr.): `target` keeps `self` alive.
            py::keep_alive<2, 1>(), doc.DiagramBuilder.BuildInto.doc);

    DefineTemplateClassWithDefault<OutputPort<T>>(
        m, "OutputPort", GetPyParam<T>(), doc.OutputPort.doc)
        .def("size", &OutputPort<T>::size, doc.PortBase.size.doc)
        .def("get_data_type", &OutputPort<T>::get_data_type,
            doc.PortBase.get_data_type.doc)
        .def("get_index", &OutputPort<T>::get_index,
            doc.OutputPortBase.get_index.doc)
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
        .def("get_system", &OutputPort<T>::get_system, py_rvp::reference,
            doc.OutputPort.get_system.doc);

    DefineTemplateClassWithDefault<LeafOutputPort<T>, OutputPort<T>>(
        m, "LeafOutputPort", GetPyParam<T>(), doc.LeafOutputPort.doc)
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

    DefineTemplateClassWithDefault<InputPort<T>>(
        m, "InputPort", GetPyParam<T>(), doc.InputPort.doc)
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
              py::cast(storage.get(), py_rvp::reference)
                  .attr("set_value")(value);
              FixedInputPortValue& result = self->FixValue(context, *storage);
              return &result;
            },
            py::arg("context"), py::arg("value"), py_rvp::reference,
            // Keep alive, ownership: `return` keeps `context` alive.
            py::keep_alive<0, 2>(), doc.InputPort.FixValue.doc)
        .def("HasValue", &InputPort<T>::HasValue, py::arg("context"),
            doc.InputPort.HasValue.doc)
        .def("get_system", &InputPort<T>::get_system, py_rvp::reference,
            doc.InputPort.get_system.doc);

    // TODO(russt): Bind relevant WitnessFunction methods.  This is the
    // minimal binding required to support DeclareWitnessFunction.
    DefineTemplateClassWithDefault<WitnessFunction<T>>(
        m, "WitnessFunction", GetPyParam<T>(), doc.WitnessFunction.doc);

    // Parameters.
    auto parameters = DefineTemplateClassWithDefault<Parameters<T>>(
        m, "Parameters", GetPyParam<T>(), doc.Parameters.doc);
    DefClone(&parameters);
    using BasicVectorPtrList = vector<unique_ptr<BasicVector<T>>>;
    parameters
        .def(py::init<>(), doc.Parameters.ctor.doc_0args)
        // TODO(eric.cousineau): Ensure that we can respect keep alive behavior
        // with lists of pointers.
        .def(py::init<BasicVectorPtrList, AbstractValuePtrList>(),
            py::arg("numeric"), py::arg("abstract"),
            doc.Parameters.ctor.doc_2args_numeric_abstract)
        .def(py::init<BasicVectorPtrList>(), py::arg("numeric"),
            doc.Parameters.ctor.doc_1args_numeric)
        .def(py::init<AbstractValuePtrList>(), py::arg("abstract"),
            doc.Parameters.ctor.doc_1args_abstract)
        .def(py::init<unique_ptr<BasicVector<T>>>(), py::arg("vec"),
            // Keep alive, ownership: `vec` keeps `self` alive.
            py::keep_alive<2, 1>(), doc.Parameters.ctor.doc_1args_vec)
        .def(py::init<unique_ptr<AbstractValue>>(), py::arg("value"),
            // Keep alive, ownership: `value` keeps `self` alive.
            py::keep_alive<2, 1>(), doc.Parameters.ctor.doc_1args_value)
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
            py_rvp::reference_internal,
            doc.Parameters.get_numeric_parameters.doc)
        // TODO(eric.cousineau): Should this C++ code constrain the number of
        // parameters???
        .def("set_numeric_parameters", &Parameters<T>::set_numeric_parameters,
            // WARNING: This will DELETE the existing parameters. See C++
            // `AddValueInstantiation` for more information.
            // Keep alive, ownership: `value` keeps `self` alive.
            py::keep_alive<2, 1>(), py::arg("numeric_params"),
            doc.Parameters.set_numeric_parameters.doc)
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
        .def("set_abstract_parameters", &Parameters<T>::set_abstract_parameters,
            // WARNING: This will DELETE the existing parameters. See C++
            // `AddValueInstantiation` for more information.
            // Keep alive, ownership: `value` keeps `self` alive.
            py::keep_alive<2, 1>(), py::arg("abstract_params"),
            doc.Parameters.set_abstract_parameters.doc)
        .def(
            "SetFrom",
            [](Parameters<T>* self, const Parameters<double>& other) {
              self->SetFrom(other);
            },
            doc.Parameters.SetFrom.doc);

    // State.
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
            py_rvp::reference_internal,
            doc.State.get_mutable_discrete_state.doc)
        .def("get_abstract_state",
            static_cast<const AbstractValues& (State<T>::*)() const>(
                &State<T>::get_abstract_state),
            py_rvp::reference_internal, doc.State.get_abstract_state.doc)
        .def(
            "get_mutable_abstract_state",
            [](State<T>* self) -> AbstractValues& {
              return self->get_mutable_abstract_state();
            },
            py_rvp::reference_internal,
            doc.State.get_mutable_abstract_state.doc);

    // - Constituents.
    auto continuous_state = DefineTemplateClassWithDefault<ContinuousState<T>>(
        m, "ContinuousState", GetPyParam<T>(), doc.ContinuousState.doc);
    DefClone(&continuous_state);
    continuous_state
        .def(py::init<unique_ptr<VectorBase<T>>>(), py::arg("state"),
            doc.ContinuousState.ctor.doc_1args_state)
        .def(py::init<unique_ptr<VectorBase<T>>, int, int, int>(),
            py::arg("state"), py::arg("num_q"), py::arg("num_v"),
            py::arg("num_z"),
            doc.ContinuousState.ctor.doc_4args_state_num_q_num_v_num_z)
        .def(py::init<>(), doc.ContinuousState.ctor.doc_0args)
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

    auto discrete_values = DefineTemplateClassWithDefault<DiscreteValues<T>>(
        m, "DiscreteValues", GetPyParam<T>(), doc.DiscreteValues.doc);
    DefClone(&discrete_values);
    discrete_values
        .def(py::init<unique_ptr<BasicVector<T>>>(), py::arg("datum"),
            doc.DiscreteValues.ctor.doc_1args_datum)
        .def(py::init<std::vector<std::unique_ptr<BasicVector<T>>>&&>(),
            py::arg("data"), doc.DiscreteValues.ctor.doc_1args_data)
        .def(py::init<>(), doc.DiscreteValues.ctor.doc_0args)
        .def("num_groups", &DiscreteValues<T>::num_groups,
            doc.DiscreteValues.num_groups.doc)
        .def("size", &DiscreteValues<T>::size, doc.DiscreteValues.size.doc)
        .def("get_data", &DiscreteValues<T>::get_data,
            py_rvp::reference_internal, doc.DiscreteValues.get_data.doc)
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
  };
  type_visit(bind_common_scalar_types, CommonScalarPack{});
}  // NOLINT(readability/fn_size)

}  // namespace pydrake
}  // namespace drake
