#include "drake/bindings/pydrake/systems/framework_py_semantics.h"

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/bindings/pydrake/util/drake_optional_pybind.h"
#include "drake/bindings/pydrake/util/eigen_pybind.h"
#include "drake/bindings/pydrake/util/type_safe_index_pybind.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/output_port_value.h"

using std::string;
using std::unique_ptr;
using std::vector;

namespace drake {
namespace pydrake {

using pysystems::DefClone;

void DefineFrameworkPySemantics(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;

  m.attr("kAutoSize") = kAutoSize;

  py::enum_<PortDataType>(m, "PortDataType")
    .value("kVectorValued", kVectorValued)
    .value("kAbstractValued", kAbstractValued);

  BindTypeSafeIndex<DependencyTicket>(m, "DependencyTicket");
  BindTypeSafeIndex<CacheIndex>(m, "CacheIndex");
  BindTypeSafeIndex<SubsystemIndex>(m, "SubsystemIndex");
  BindTypeSafeIndex<InputPortIndex>(m, "InputPortIndex");
  BindTypeSafeIndex<OutputPortIndex>(m, "OutputPortIndex");
  BindTypeSafeIndex<DiscreteStateIndex>(m, "DiscreteStateIndex");
  BindTypeSafeIndex<AbstractStateIndex>(m, "AbstractStateIndex");
  BindTypeSafeIndex<NumericParameterIndex>(m, "NumericParameterIndex");
  BindTypeSafeIndex<AbstractParameterIndex>(m, "AbstractParameterIndex");

  py::class_<FixedInputPortValue>(m, "FixedInputPortValue");

  using AbstractValuePtrList = vector<unique_ptr<AbstractValue>>;
  // N.B. `AbstractValues` provides the ability to reference non-owned values,
  // without copying them. For consistency with other model-value Python
  // bindings, only the ownership variant is exposed.
  py::class_<AbstractValues> abstract_values(m, "AbstractValues");
  DefClone(&abstract_values);
  abstract_values
    .def(py::init<>())
    .def(py::init<AbstractValuePtrList>())
    .def("size", &AbstractValues::size)
    .def("get_value", &AbstractValues::get_value, py_reference_internal)
    .def("get_mutable_value",
         &AbstractValues::get_mutable_value, py_reference_internal)
    .def("CopyFrom", &AbstractValues::CopyFrom);

  auto bind_common_scalar_types = [m](auto dummy) {
    using T = decltype(dummy);
    DefineTemplateClassWithDefault<Context<T>>(
        m, "Context", GetPyParam<T>())
      .def("get_num_input_ports", &Context<T>::get_num_input_ports)
      .def("FixInputPort",
           py::overload_cast<int, const BasicVector<T>&>(
               &Context<T>::FixInputPort),
           py::arg("index"), py::arg("vec"),
           py_reference_internal)
      .def("FixInputPort",
           py::overload_cast<int, unique_ptr<AbstractValue>>(
               &Context<T>::FixInputPort),
           py::arg("index"), py::arg("value"),
           py_reference_internal,
           // Keep alive, ownership: `AbstractValue` keeps `self` alive.
           py::keep_alive<3, 1>())
      .def("FixInputPort",
           py::overload_cast<int, const Eigen::Ref<const VectorX<T>>&>(
               &Context<T>::FixInputPort),
           py::arg("index"), py::arg("data"),
           py_reference_internal)
      .def("get_time", &Context<T>::get_time)
      .def("set_time", &Context<T>::set_time)
      .def("set_accuracy", &Context<T>::set_accuracy)
      .def("get_accuracy", &Context<T>::get_accuracy)
      .def("Clone", &Context<T>::Clone)
      .def("__copy__", &Context<T>::Clone)
      .def("__deepcopy__", [](const Context<T>* self, py::dict /* memo */) {
        return self->Clone();
      })
      .def("get_state", &Context<T>::get_state, py_reference_internal)
      .def("get_mutable_state", &Context<T>::get_mutable_state,
           py_reference_internal)
      // Sugar methods
      // - Continuous.
      .def("get_continuous_state_vector",
           &Context<T>::get_continuous_state_vector,
           py_reference_internal)
      .def("get_mutable_continuous_state_vector",
           &Context<T>::get_mutable_continuous_state_vector,
           py_reference_internal)
      // - Discrete.
      .def("get_discrete_state_vector",
           &Context<T>::get_discrete_state_vector,
           py_reference_internal)
      .def("get_mutable_discrete_state_vector",
           [](Context<T>* self) -> auto& {
             return self->get_mutable_discrete_state().get_mutable_vector();
           },
           py_reference_internal)
      // - Abstract.
      .def("get_num_abstract_states", &Context<T>::get_num_abstract_states)
      .def("get_abstract_state",
           [](const Context<T>* self) -> auto& {
             return self->get_abstract_state();
           },
           py_reference_internal)
      .def("get_abstract_state",
           [](const Context<T>* self, int index) -> auto& {
             return self->get_abstract_state().get_value(index);
           },
           py_reference_internal)
      .def("get_mutable_abstract_state",
           [](Context<T>* self) -> auto& {
             return self->get_mutable_abstract_state();
           },
           py_reference_internal)
      .def("get_mutable_abstract_state",
           [](Context<T>* self, int index) -> auto& {
             return self->get_mutable_abstract_state().get_mutable_value(index);
           },
           py_reference_internal);

    DefineTemplateClassWithDefault<LeafContext<T>, Context<T>>(
        m, "LeafContext", GetPyParam<T>());

    // Event mechanisms.
    DefineTemplateClassWithDefault<Event<T>>(m, "Event", GetPyParam<T>());
    DefineTemplateClassWithDefault<PublishEvent<T>, Event<T>>(
        m, "PublishEvent", GetPyParam<T>());
    DefineTemplateClassWithDefault<DiscreteUpdateEvent<T>, Event<T>>(
        m, "DiscreteUpdateEvent", GetPyParam<T>());

    // Glue mechanisms.
    DefineTemplateClassWithDefault<DiagramBuilder<T>>(
        m, "DiagramBuilder", GetPyParam<T>())
      .def(py::init<>())
      .def(
          "AddSystem",
          [](DiagramBuilder<T>* self, unique_ptr<System<T>> arg1) {
            return self->AddSystem(std::move(arg1));
          },
          // Keep alive, ownership: `System` keeps `self` alive.
          py::keep_alive<2, 1>())
      .def("Connect",
           py::overload_cast<
               const OutputPort<T>&, const InputPortDescriptor<T>&>(
               &DiagramBuilder<T>::Connect))
      .def("ExportInput", &DiagramBuilder<T>::ExportInput,
           py_reference_internal)
      .def("ExportOutput", &DiagramBuilder<T>::ExportOutput,
           py_reference_internal)
      .def("Build", &DiagramBuilder<T>::Build,
           // Keep alive, transitive: `return` keeps `self` alive.
           py::keep_alive<1, 0>())
      .def("BuildInto", &DiagramBuilder<T>::BuildInto,
           // Keep alive, transitive: `Diagram` keeps `self` alive.
           py::keep_alive<2, 1>());

    DefineTemplateClassWithDefault<OutputPort<T>>(
        m, "OutputPort", GetPyParam<T>())
      .def("size", &OutputPort<T>::size)
      .def("get_index", &OutputPort<T>::get_index);

    auto system_output = DefineTemplateClassWithDefault<SystemOutput<T>>(
        m, "SystemOutput", GetPyParam<T>());
    DefClone(&system_output);
    system_output
      .def("get_num_ports", &SystemOutput<T>::get_num_ports)
      .def("get_data", &SystemOutput<T>::get_data,
           py_reference_internal)
      .def("get_vector_data", &SystemOutput<T>::get_vector_data,
           py_reference_internal);

    DefineTemplateClassWithDefault<InputPortDescriptor<T>>(
        m, "InputPortDescriptor", GetPyParam<T>())
      .def("size", &InputPortDescriptor<T>::size)
      .def("get_data_type", &InputPortDescriptor<T>::get_data_type)
      .def("get_index", &InputPortDescriptor<T>::get_index);

    // Parameters.
    auto parameters = DefineTemplateClassWithDefault<Parameters<T>>(
        m, "Parameters", GetPyParam<T>());
    DefClone(&parameters);
    using BasicVectorPtrList = vector<unique_ptr<BasicVector<T>>>;
    parameters
      .def(py::init<>())
      // TODO(eric.cousineau): Ensure that we can respect keep alive behavior
      // with lists of pointers.
      .def(py::init<BasicVectorPtrList, AbstractValuePtrList>(),
           py::arg("numeric"), py::arg("abstract"))
      .def(py::init<BasicVectorPtrList>(), py::arg("numeric"))
      .def(py::init<AbstractValuePtrList>(), py::arg("abstract"))
      .def(py::init<unique_ptr<BasicVector<T>>>(), py::arg("vec"),
           // Keep alive, ownership: `vec` keeps `self` alive.
           py::keep_alive<2, 1>())
      .def(py::init<unique_ptr<AbstractValue>>(), py::arg("value"),
           // Keep alive, ownership: `value` keeps `self` alive.
           py::keep_alive<2, 1>())
      .def("num_numeric_parameters", &Parameters<T>::num_numeric_parameters)
      .def("num_abstract_parameters", &Parameters<T>::num_abstract_parameters)
      .def("get_numeric_parameter", &Parameters<T>::get_numeric_parameter,
           py_reference_internal, py::arg("index"))
      .def("get_mutable_numeric_parameter",
           &Parameters<T>::get_mutable_numeric_parameter,
           py_reference_internal, py::arg("index"))
      .def("get_numeric_parameters", &Parameters<T>::get_numeric_parameters,
           py_reference_internal)
      // TODO(eric.cousineau): Should this C++ code constrain the number of
      // parameters???
      .def("set_numeric_parameters", &Parameters<T>::set_numeric_parameters,
           // WARNING: This will DELETE the existing parameters. See C++
           // `AddValueInstantiation` for more information.
           // Keep alive, ownership: `value` keeps `self` alive.
           py::keep_alive<2, 1>(), py::arg("numeric_params"))
      .def("get_abstract_parameter",
           [](const Parameters<T>* self, int index) -> auto& {
             return self->get_abstract_parameter(index);
           },
           py_reference_internal, py::arg("index"))
      .def("get_mutable_abstract_parameter",
           [](Parameters<T>* self, int index) -> auto& {
             return self->get_mutable_abstract_parameter(index);
           },
           py_reference_internal, py::arg("index"))
      .def("get_abstract_parameters", &Parameters<T>::get_abstract_parameters,
           py_reference_internal)
      .def("set_abstract_parameters", &Parameters<T>::set_abstract_parameters,
           // WARNING: This will DELETE the existing parameters. See C++
           // `AddValueInstantiation` for more information.
           // Keep alive, ownership: `value` keeps `self` alive.
           py::keep_alive<2, 1>(), py::arg("abstract_params"))
      .def("SetFrom", &Parameters<T>::SetFrom);

    // State.
    DefineTemplateClassWithDefault<State<T>>(m, "State", GetPyParam<T>())
        .def(py::init<>())
        .def("get_continuous_state", &State<T>::get_continuous_state,
             py_reference_internal)
        .def("get_mutable_continuous_state",
             &State<T>::get_mutable_continuous_state, py_reference_internal)
        .def("get_discrete_state",
             overload_cast_explicit<const DiscreteValues<T>&>(
                 &State<T>::get_discrete_state),
             py_reference_internal)
        .def("get_mutable_discrete_state",
             overload_cast_explicit<DiscreteValues<T>&>(
                 &State<T>::get_mutable_discrete_state),
             py_reference_internal);

    // - Constituents.
    DefineTemplateClassWithDefault<ContinuousState<T>>(
        m, "ContinuousState", GetPyParam<T>())
      .def(py::init<>())
      .def("get_vector", &ContinuousState<T>::get_vector, py_reference_internal)
      .def("get_mutable_vector",
           &ContinuousState<T>::get_mutable_vector, py_reference_internal);

    auto discrete_values = DefineTemplateClassWithDefault<DiscreteValues<T>>(
        m, "DiscreteValues", GetPyParam<T>());
    DefClone(&discrete_values);
    discrete_values
      .def("num_groups", &DiscreteValues<T>::num_groups)
      .def("get_data", &DiscreteValues<T>::get_data, py_reference_internal)
      .def("get_vector",
           overload_cast_explicit<const BasicVector<T>&, int>(
              &DiscreteValues<T>::get_vector),
           py_reference_internal, py::arg("index") = 0)
      .def("get_mutable_vector",
           overload_cast_explicit<BasicVector<T>&, int>(
              &DiscreteValues<T>::get_mutable_vector),
           py_reference_internal, py::arg("index") = 0);
  };
  type_visit(bind_common_scalar_types, pysystems::CommonScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
