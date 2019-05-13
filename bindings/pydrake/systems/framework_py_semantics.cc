#include "drake/bindings/pydrake/systems/framework_py_semantics.h"

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/drake_optional_pybind.h"
#include "drake/bindings/pydrake/common/drake_variant_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/common/type_safe_index_pybind.h"
#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_context.h"
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
      // TODO(#9398): Figure out why `py_reference` is necessary.
      py::object value_ref = py::cast(&abstract, py_reference);
      return value_ref.attr("get_value")();
    }
  }
  DRAKE_UNREACHABLE();
}
}  // namespace

void DefineFrameworkPySemantics(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  constexpr auto& doc = pydrake_doc.drake.systems;

  m.attr("kAutoSize") = kAutoSize;

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
          // TODO(#9398): Why is `py_reference` necessary?
          py_reference_internal, doc.FixedInputPortValue.GetMutableData.doc);

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
          py_reference_internal, doc.AbstractValues.get_value.doc)
      .def("get_mutable_value", &AbstractValues::get_mutable_value,
          py::arg("index"), py_reference_internal,
          doc.AbstractValues.get_mutable_value.doc)
      .def("CopyFrom",
          [](AbstractValues* self, const AbstractValues& other) {
            WarnDeprecated(
                "Use SetFrom instead of CopyFrom. "
                "This method will be removed on 2019-06-01.");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
            self->CopyFrom(other);
#pragma GCC diagnostic pop
          },
          doc.AbstractValues.CopyFrom.doc_deprecated)
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

  // N.B. Capturing `&doc` should not be required; workaround per #9600.
  auto bind_common_scalar_types = [m, &doc](auto dummy) {
    using T = decltype(dummy);
    auto context_cls = DefineTemplateClassWithDefault<Context<T>>(
        m, "Context", GetPyParam<T>(), doc.Context.doc);
    context_cls
        .def("__str__", &Context<T>::to_string, doc.Context.to_string.doc)
        .def("num_input_ports", &Context<T>::num_input_ports,
            doc.ContextBase.num_input_ports.doc)
        .def("get_num_input_ports",
            [](const Context<T>* self) {
              WarnDeprecated(
                  "Use num_input_ports() instead. Will be removed on or after "
                  "2019-07-01.");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
              self->get_num_input_ports();
#pragma GCC diagnostic pop
            },
            doc.ContextBase.get_num_input_ports.doc_deprecated)
        .def("num_output_ports", &Context<T>::num_output_ports,
            doc.ContextBase.num_output_ports.doc)
        .def("FixInputPort",
            py::overload_cast<int, const BasicVector<T>&>(
                &Context<T>::FixInputPort),
            py::arg("index"), py::arg("vec"), py_reference_internal,
            doc.Context.FixInputPort.doc_2args_index_vec)
        .def("FixInputPort",
            py::overload_cast<int, unique_ptr<AbstractValue>>(
                &Context<T>::FixInputPort),
            py::arg("index"), py::arg("value"), py_reference_internal,
            // Keep alive, ownership: `AbstractValue` keeps `self` alive.
            py::keep_alive<3, 1>(), doc.ContextBase.FixInputPort.doc)
        .def("FixInputPort",
            py::overload_cast<int, const Eigen::Ref<const VectorX<T>>&>(
                &Context<T>::FixInputPort),
            py::arg("index"), py::arg("data"), py_reference_internal,
            doc.Context.FixInputPort.doc_2args_index_data)
        .def("get_time", &Context<T>::get_time, doc.Context.get_time.doc)
        .def("SetTime", &Context<T>::SetTime, py::arg("time_sec"),
            doc.Context.SetTime.doc)
        .def("set_time",
            [](Context<T>* self, const T& time) {
              WarnDeprecated(
                  "Use SetTime() instead. Will be removed on or after "
                  "2019-07-01.");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
              self->set_time(time);
#pragma GCC diagnostic pop
            },
            doc.Context.set_time.doc_deprecated)
        .def("SetAccuracy", &Context<T>::SetAccuracy, py::arg("accuracy"),
            doc.Context.SetAccuracy.doc)
        .def("set_accuracy",
            [](Context<T>* self, const optional<double>& accuracy) {
              WarnDeprecated(
                  "Use SetAccuracy() instead. Will be removed on or after "
                  "2019-07-01.");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
              self->set_accuracy(accuracy);
#pragma GCC diagnostic pop
            },
            doc.Context.set_accuracy.doc_deprecated)
        .def("get_accuracy", &Context<T>::get_accuracy,
            doc.Context.get_accuracy.doc)
        .def("Clone", &Context<T>::Clone, doc.Context.Clone.doc)
        .def("__copy__", &Context<T>::Clone)
        .def("__deepcopy__", [](const Context<T>* self,
                                 py::dict /* memo */) { return self->Clone(); })
        .def("get_state", &Context<T>::get_state, py_reference_internal,
            doc.Context.get_state.doc)
        .def("get_mutable_state", &Context<T>::get_mutable_state,
            py_reference_internal, doc.Context.get_mutable_state.doc)
        // Sugar methods
        // - Continuous.
        .def("num_continuous_states", &Context<T>::num_continuous_states,
            doc.Context.num_continuous_states.doc)
        .def("get_continuous_state", &Context<T>::get_continuous_state,
            py_reference_internal, doc.Context.get_continuous_state.doc)
        .def("get_mutable_continuous_state",
            &Context<T>::get_mutable_continuous_state, py_reference_internal,
            doc.Context.get_mutable_continuous_state.doc)
        .def("get_continuous_state_vector",
            &Context<T>::get_continuous_state_vector, py_reference_internal,
            doc.Context.get_continuous_state_vector.doc)
        .def("SetContinuousState", &Context<T>::SetContinuousState,
            doc.Context.SetContinuousState.doc)
        .def("get_mutable_continuous_state_vector",
            &Context<T>::get_mutable_continuous_state_vector,
            py_reference_internal,
            doc.Context.get_mutable_continuous_state_vector.doc)
        // - Discrete.
        .def("num_discrete_state_groups",
            &Context<T>::num_discrete_state_groups,
            doc.Context.num_discrete_state_groups.doc)
        .def("get_num_discrete_state_groups",
            [](const Context<T>* self) {
              WarnDeprecated(
                  "Use num_discrete_state_groups() instead. Will be removed "
                  "on or after 2019-07-01.");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
              self->get_num_discrete_state_groups();
#pragma GCC diagnostic pop
            },
            doc.Context.get_num_discrete_state_groups.doc_deprecated)
        .def("get_discrete_state",
            overload_cast_explicit<const DiscreteValues<T>&>(
                &Context<T>::get_discrete_state),
            py_reference_internal, doc.Context.get_discrete_state.doc_0args)
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
        .def("get_mutable_discrete_state",
            overload_cast_explicit<DiscreteValues<T>&>(
                &Context<T>::get_mutable_discrete_state),
            py_reference_internal,
            doc.Context.get_mutable_discrete_state.doc_0args)
        .def("get_discrete_state_vector",
            &Context<T>::get_discrete_state_vector, py_reference_internal,
            doc.Context.get_discrete_state_vector.doc)
        .def("get_mutable_discrete_state_vector",
            &Context<T>::get_mutable_discrete_state_vector,
            py_reference_internal,
            doc.Context.get_mutable_discrete_state_vector.doc)
        .def("get_discrete_state",
            overload_cast_explicit<const BasicVector<T>&, int>(
                &Context<T>::get_discrete_state),
            py_reference_internal, doc.Context.get_discrete_state.doc_1args)
        .def("get_mutable_discrete_state",
            overload_cast_explicit<BasicVector<T>&, int>(
                &Context<T>::get_mutable_discrete_state),
            py_reference_internal,
            doc.Context.get_mutable_discrete_state.doc_1args)
        // - Abstract.
        .def("num_abstract_states", &Context<T>::num_abstract_states,
            doc.Context.num_abstract_states.doc)
        .def("get_num_abstract_states",
            [](const Context<T>* self) {
              WarnDeprecated(
                  "Use num_abstract_states() instead. Will be removed on or "
                  "after 2019-07-01.");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
              self->get_num_abstract_states();
#pragma GCC diagnostic pop
            },
            doc.Context.get_num_abstract_states.doc_deprecated)
        .def("get_abstract_state",
            static_cast<const AbstractValues& (Context<T>::*)() const>(
                &Context<T>::get_abstract_state),
            py_reference_internal, doc.Context.get_abstract_state.doc_0args)
        .def("get_abstract_state",
            [](const Context<T>* self, int index) -> auto& {
              return self->get_abstract_state().get_value(index);
            },
            py::arg("index"), py_reference_internal,
            doc.Context.get_abstract_state.doc_1args)
        .def("get_mutable_abstract_state",
            [](Context<T>* self) -> AbstractValues& {
              return self->get_mutable_abstract_state();
            },
            py_reference_internal,
            doc.Context.get_mutable_abstract_state.doc_0args)
        .def("get_mutable_abstract_state",
            [](Context<T>* self, int index) -> AbstractValue& {
              return self->get_mutable_abstract_state().get_mutable_value(
                  index);
            },
            py::arg("index"), py_reference_internal,
            doc.Context.get_mutable_abstract_state.doc_1args)
        .def("SetAbstractState",
            [](py::object self, int index, py::object value) {
              // Use type erasure from Python bindings of Value[T].set_value.
              py::object abstract_value =
                  self.attr("get_mutable_abstract_state")(index);
              abstract_value.attr("set_value")(value);
            },
            py::arg("index"), py::arg("value"),
            doc.Context.SetAbstractState.doc)
        .def("get_parameters", &Context<T>::get_parameters,
            py_reference_internal, doc.Context.get_parameters.doc)
        .def("num_numeric_parameter_groups",
            &Context<T>::num_numeric_parameter_groups,
            doc.Context.num_numeric_parameter_groups.doc)
        .def("get_numeric_parameter", &Context<T>::get_numeric_parameter,
            py::arg("index"), py_reference_internal,
            doc.Context.get_numeric_parameter.doc)
        .def("num_abstract_parameters", &Context<T>::num_abstract_parameters,
            doc.Context.num_abstract_parameters.doc)
        .def("get_abstract_parameter", &Context<T>::get_abstract_parameter,
            py::arg("index"), py_reference_internal,
            doc.Context.get_numeric_parameter.doc);

    auto bind_context_methods_templated_on_a_secondary_scalar =
        [m, &doc, &context_cls](auto dummy_u) {
          using U = decltype(dummy_u);
          context_cls.def("SetTimeStateAndParametersFrom",
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
        .def("AddSystem",
            [](DiagramBuilder<T>* self, unique_ptr<System<T>> arg1) {
              return self->AddSystem(std::move(arg1));
            },
            // Keep alive, ownership: `System` keeps `self` alive.
            py::keep_alive<2, 1>(), doc.DiagramBuilder.AddSystem.doc)
        .def("Connect",
            py::overload_cast<const OutputPort<T>&, const InputPort<T>&>(
                &DiagramBuilder<T>::Connect),
            doc.DiagramBuilder.Connect.doc)
        .def("ExportInput", &DiagramBuilder<T>::ExportInput, py::arg("input"),
            py::arg("name") = kUseDefaultName, py_reference_internal,
            doc.DiagramBuilder.ExportInput.doc)
        .def("ExportOutput", &DiagramBuilder<T>::ExportOutput,
            py::arg("output"), py::arg("name") = kUseDefaultName,
            py_reference_internal, doc.DiagramBuilder.ExportOutput.doc)
        .def("Build", &DiagramBuilder<T>::Build,
            // Keep alive, transitive: `return` keeps `self` alive.
            py::keep_alive<1, 0>(), doc.DiagramBuilder.Build.doc)
        .def("BuildInto", &DiagramBuilder<T>::BuildInto,
            // Keep alive, transitive: `Diagram` keeps `self` alive.
            py::keep_alive<2, 1>(), doc.DiagramBuilder.BuildInto.doc);

    DefineTemplateClassWithDefault<OutputPort<T>>(
        m, "OutputPort", GetPyParam<T>(), doc.OutputPort.doc)
        .def("size", &OutputPort<T>::size, doc.PortBase.size.doc)
        .def("get_data_type", &OutputPort<T>::get_data_type,
            doc.PortBase.get_data_type.doc)
        .def("get_index", &OutputPort<T>::get_index,
            doc.OutputPortBase.get_index.doc)
        .def("Eval",
            [](const OutputPort<T>* self, const Context<T>& context) {
              return DoEval(self, context);
            },
            doc.OutputPort.Eval.doc)
        .def("EvalAbstract",
            [](const OutputPort<T>* self, const Context<T>& c) {
              const auto& abstract = self->template Eval<AbstractValue>(c);
              // TODO(#9398): Figure out why `py_reference` is necessary.
              return py::cast(&abstract, py_reference);
            },
            py::arg("context"),
            "(Advanced.) Returns the value of this output port, typed "
            "as an AbstractValue. Most users should call Eval() instead. "
            "This method is only needed when the result will be passed "
            "into some other API that only accepts an AbstractValue.",
            py_reference_internal)
        .def("EvalBasicVector",
            [](const OutputPort<T>* self, const Context<T>& c) {
              const auto& basic = self->template Eval<BasicVector<T>>(c);
              // TODO(#9398): Figure out why `py_reference` is necessary.
              return py::cast(&basic, py_reference);
            },
            py::arg("context"),
            "(Advanced.) Returns the value of this output port, typed "
            "as a BasicVector. Most users should call Eval() instead. "
            "This method is only needed when the result will be passed "
            "into some other API that only accepts a BasicVector.",
            py_reference_internal);

    auto system_output = DefineTemplateClassWithDefault<SystemOutput<T>>(
        m, "SystemOutput", GetPyParam<T>(), doc.SystemOutput.doc);
    system_output
        .def("num_ports", &SystemOutput<T>::num_ports,
            doc.SystemOutput.num_ports.doc)
        .def("get_num_ports",
            [](const SystemOutput<T>* self) {
              WarnDeprecated(
                  "Use num_ports() instead. Will be removed on or after "
                  "2019-07-01.");
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
              self->get_num_ports();
#pragma GCC diagnostic pop
            },
            doc.SystemOutput.get_num_ports.doc_deprecated)
        .def("get_data", &SystemOutput<T>::get_data, py_reference_internal,
            doc.SystemOutput.get_data.doc)
        .def("get_vector_data", &SystemOutput<T>::get_vector_data,
            py_reference_internal, doc.SystemOutput.get_vector_data.doc);

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
        .def("Eval",
            [](const InputPort<T>* self, const Context<T>& context) {
              return DoEval(self, context);
              DRAKE_UNREACHABLE();
            },
            doc.InputPort.Eval.doc)
        .def("EvalAbstract",
            [](const InputPort<T>* self, const Context<T>& c) {
              const auto& abstract = self->template Eval<AbstractValue>(c);
              // TODO(#9398): Figure out why `py_reference` is necessary.
              return py::cast(&abstract, py_reference);
            },
            py::arg("context"),
            "(Advanced.) Returns the value of this input port, typed "
            "as an AbstractValue. Most users should call Eval() instead. "
            "This method is only needed when the result will be passed "
            "into some other API that only accepts an AbstractValue.",
            py_reference_internal)
        .def("EvalBasicVector",
            [](const InputPort<T>* self, const Context<T>& c) {
              const auto& basic = self->template Eval<BasicVector<T>>(c);
              // TODO(#9398): Figure out why `py_reference` is necessary.
              return py::cast(&basic, py_reference);
            },
            py::arg("context"),
            "(Advanced.) Returns the value of this input port, typed "
            "as a BasicVector. Most users should call Eval() instead. "
            "This method is only needed when the result will be passed "
            "into some other API that only accepts a BasicVector.",
            py_reference_internal)
        // For FixValue, treat an already-erased AbstractValue specially ...
        .def("FixValue",
            [](const InputPort<T>* self, Context<T>* context,
                const AbstractValue& value) {
              FixedInputPortValue& result = self->FixValue(context, value);
              return &result;
            },
            py::arg("context"), py::arg("value"), py_reference,
            // Keep alive, ownership: `return` keeps `context` alive.
            py::keep_alive<0, 2>(), doc.InputPort.FixValue.doc)
        // ... but then for anything not yet erased, use set_value to copy.
        .def("FixValue",
            [](const InputPort<T>* self, Context<T>* context,
                const py::object& value) {
              const auto& system = self->get_system();
              // Allocate is a bit wasteful, but FixValue is already expensive.
              std::unique_ptr<AbstractValue> storage =
                  system.AllocateInputAbstract(*self);
              py::cast(storage.get(), py_reference).attr("set_value")(value);
              FixedInputPortValue& result = self->FixValue(context, *storage);
              return &result;
            },
            py::arg("context"), py::arg("value"), py_reference,
            // Keep alive, ownership: `return` keeps `context` alive.
            py::keep_alive<0, 2>(), doc.InputPort.FixValue.doc);

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
            py_reference_internal, py::arg("index"),
            doc.Parameters.get_numeric_parameter.doc)
        .def("get_mutable_numeric_parameter",
            &Parameters<T>::get_mutable_numeric_parameter,
            py_reference_internal, py::arg("index"),
            doc.Parameters.get_mutable_numeric_parameter.doc)
        .def("get_numeric_parameters", &Parameters<T>::get_numeric_parameters,
            py_reference_internal, doc.Parameters.get_numeric_parameters.doc)
        // TODO(eric.cousineau): Should this C++ code constrain the number of
        // parameters???
        .def("set_numeric_parameters", &Parameters<T>::set_numeric_parameters,
            // WARNING: This will DELETE the existing parameters. See C++
            // `AddValueInstantiation` for more information.
            // Keep alive, ownership: `value` keeps `self` alive.
            py::keep_alive<2, 1>(), py::arg("numeric_params"),
            doc.Parameters.set_numeric_parameters.doc)
        .def("get_abstract_parameter",
            [](const Parameters<T>* self, int index) -> auto& {
              return self->get_abstract_parameter(index);
            },
            py_reference_internal, py::arg("index"),
            doc.Parameters.get_abstract_parameter.doc_1args_index)
        .def("get_mutable_abstract_parameter",
            [](Parameters<T>* self, int index) -> AbstractValue& {
              return self->get_mutable_abstract_parameter(index);
            },
            py_reference_internal, py::arg("index"),
            doc.Parameters.get_mutable_abstract_parameter.doc_1args_index)
        .def("get_abstract_parameters", &Parameters<T>::get_abstract_parameters,
            py_reference_internal, doc.Parameters.get_abstract_parameters.doc)
        .def("set_abstract_parameters", &Parameters<T>::set_abstract_parameters,
            // WARNING: This will DELETE the existing parameters. See C++
            // `AddValueInstantiation` for more information.
            // Keep alive, ownership: `value` keeps `self` alive.
            py::keep_alive<2, 1>(), py::arg("abstract_params"),
            doc.Parameters.set_abstract_parameters.doc)
        .def("SetFrom",
            [](Parameters<T>* self, const Parameters<double>& other) {
              self->SetFrom(other);
            },
            doc.Parameters.SetFrom.doc);

    // State.
    DefineTemplateClassWithDefault<State<T>>(
        m, "State", GetPyParam<T>(), doc.State.doc)
        .def(py::init<>(), doc.State.ctor.doc)
        .def("get_continuous_state", &State<T>::get_continuous_state,
            py_reference_internal, doc.State.get_continuous_state.doc)
        .def("get_mutable_continuous_state",
            &State<T>::get_mutable_continuous_state, py_reference_internal,
            doc.State.get_mutable_continuous_state.doc)
        .def("get_discrete_state",
            overload_cast_explicit<const DiscreteValues<T>&>(
                &State<T>::get_discrete_state),
            py_reference_internal, doc.State.get_discrete_state.doc)
        .def("get_mutable_discrete_state",
            overload_cast_explicit<DiscreteValues<T>&>(
                &State<T>::get_mutable_discrete_state),
            py_reference_internal, doc.State.get_mutable_discrete_state.doc)
        .def("get_abstract_state",
            static_cast<const AbstractValues& (State<T>::*)() const>(
                &State<T>::get_abstract_state),
            py_reference_internal, doc.State.get_abstract_state.doc)
        .def("get_mutable_abstract_state",
            [](State<T>* self) -> AbstractValues& {
              return self->get_mutable_abstract_state();
            },
            py_reference_internal, doc.State.get_mutable_abstract_state.doc);

    // - Constituents.
    DefineTemplateClassWithDefault<ContinuousState<T>>(
        m, "ContinuousState", GetPyParam<T>(), doc.ContinuousState.doc)
        .def(py::init<>(), doc.ContinuousState.ctor.doc_0args)
        .def("size", &ContinuousState<T>::size, doc.ContinuousState.size.doc)
        .def("get_vector", &ContinuousState<T>::get_vector,
            py_reference_internal, doc.ContinuousState.get_vector.doc)
        .def("get_mutable_vector", &ContinuousState<T>::get_mutable_vector,
            py_reference_internal, doc.ContinuousState.get_mutable_vector.doc);

    auto discrete_values = DefineTemplateClassWithDefault<DiscreteValues<T>>(
        m, "DiscreteValues", GetPyParam<T>(), doc.DiscreteValues.doc);
    DefClone(&discrete_values);
    discrete_values
        .def("num_groups", &DiscreteValues<T>::num_groups,
            doc.DiscreteValues.num_groups.doc)
        .def("size", &DiscreteValues<T>::size, doc.DiscreteValues.size.doc)
        .def("get_data", &DiscreteValues<T>::get_data, py_reference_internal,
            doc.DiscreteValues.get_data.doc)
        .def("get_vector",
            overload_cast_explicit<const BasicVector<T>&, int>(
                &DiscreteValues<T>::get_vector),
            py_reference_internal, py::arg("index") = 0,
            doc.DiscreteValues.get_vector.doc_1args)
        .def("get_mutable_vector",
            overload_cast_explicit<BasicVector<T>&, int>(
                &DiscreteValues<T>::get_mutable_vector),
            py_reference_internal, py::arg("index") = 0,
            doc.DiscreteValues.get_mutable_vector.doc_1args);
  };
  type_visit(bind_common_scalar_types, CommonScalarPack{});
}  // NOLINT(readability/fn_size)

}  // namespace pydrake
}  // namespace drake
