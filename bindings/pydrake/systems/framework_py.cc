#include <pybind11/eigen.h>
#include <pybind11/eval.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/drake_optional_pybind.h"
#include "drake/systems/framework/abstract_values.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/output_port_value.h"
#include "drake/systems/framework/subvector.h"
#include "drake/systems/framework/supervector.h"
#include "drake/systems/framework/system.h"

using std::make_unique;
using std::unique_ptr;
using std::vector;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(framework, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;

  m.doc() = "Bindings for the core Systems framework.";

  // TODO(eric.cousineau): At present, we only bind doubles.
  // In the future, we will bind more scalar types, and enable scalar
  // conversion.
  using T = double;

  // TODO(eric.cousineau): Resolve `str_py` workaround.
  auto str_py = py::eval("str");

  m.attr("kAutoSize") = kAutoSize;

  py::enum_<PortDataType>(m, "PortDataType")
    .value("kVectorValued", kVectorValued)
    .value("kAbstractValued", kAbstractValued);

  class PySystem : public py::wrapper<System<T>> {
   public:
    using Base = py::wrapper<System<T>>;
    using Base::Base;
    // Expose protected methods for binding.
    using Base::DeclareInputPort;
  };

  // TODO(eric.cousineau): Show constructor, but somehow make sure `pybind11`
  // knows this is abstract?
  py::class_<System<T>, PySystem>(m, "System")
    .def("set_name", &System<T>::set_name)
    // Topology.
    .def("get_input_port", &System<T>::get_input_port, py_reference_internal)
    .def("get_output_port", &System<T>::get_output_port, py_reference_internal)
    .def(
        "_DeclareInputPort", &PySystem::DeclareInputPort, py_reference_internal,
        py::arg("type"), py::arg("size"), py::arg("random_type") = nullopt)
    // Context.
    .def("CreateDefaultContext", &System<T>::CreateDefaultContext)
    .def("AllocateOutput", &System<T>::AllocateOutput)
    .def(
        "EvalVectorInput",
        [](const System<T>* self, const Context<T>& arg1, int arg2) {
          return self->EvalVectorInput(arg1, arg2);
        }, py_reference_internal)
    .def("CalcOutput", &System<T>::CalcOutput)
    // Sugar.
    .def(
        "GetGraphvizString",
        [str_py](const System<T>* self) {
          // @note This is a workaround; for some reason,
          // casting this using `py::str` does not work, but directly
          // calling the Python function (`str_py`) does.
          return str_py(self->GetGraphvizString());
        });

  class LeafSystemPublic : public LeafSystem<T> {
   public:
    using Base = LeafSystem<T>;
    using Base::Base;

    // N.B. These function methods are still typed as (LeafSystem<T>::*)(...),
    // since they are more or less visibility imports.
    // Defining methods here won't work, as it will become
    // (LeafSystemPublic::*)(...), since this typeid is not exposed in pybind.
    // If needed, solution is to expose it as an intermediate type if needed.

    // Expose protected methods for binding.
    using Base::DeclareVectorOutputPort;
    using Base::DeclarePeriodicPublish;
    // Because `LeafSystem<T>::DoPublish` is protected, and we had to override
    // this method in `PyLeafSystem`, expose the method here for direct(-ish)
    // access.
    // (Otherwise, we get an error about inaccessible downcasting when trying to
    // bind `PyLeafSystem::DoPublish` to `py::class_<LeafSystem<T>, ...>`.
    using Base::DoPublish;
  };

  class PyLeafSystem : public py::wrapper<LeafSystemPublic> {
   public:
    using Base = py::wrapper<LeafSystemPublic>;
    using Base::Base;

    // Trampoline virtual methods.
    void DoPublish(
        const Context<T>& context,
        const vector<const PublishEvent<T>*>& events) const override {
      // Yuck! We have to dig in and use internals :(
      // We must ensure that pybind only sees pointers, since this method may
      // be called from C++, and pybind will not have seen these objects yet.
      // @see https://github.com/pybind/pybind11/issues/1241
      // TODO(eric.cousineau): Figure out how to supply different behavior,
      // possibly using function wrapping.
      PYBIND11_OVERLOAD_INT(
          void, LeafSystem<T>, "_DoPublish", &context, events);
      // If the macro did not return, use default functionality.
      LeafSystem<T>::DoPublish(context, events);
    }
  };

  // Don't use a const-rvalue as a function handle parameter, as pybind11 wants
  // to copy it?
  // TODO(eric.cousineau): Make a helper wrapper for this; file a bug in
  // pybind11 (since these are arguments).
  using CalcVectorPtrCallback =
      std::function<void(const Context<T>*, BasicVector<T>*)>;

  py::class_<LeafSystem<T>, PyLeafSystem, System<T>>(m, "LeafSystem")
    .def(py::init<>())
    .def(
        "_DeclareVectorOutputPort",
        [](PyLeafSystem* self, const BasicVector<T>& arg1,
           CalcVectorPtrCallback arg2) -> auto&& {
          typename LeafOutputPort<T>::CalcVectorCallback wrapped =
              [arg2](const Context<T>& nest_arg1, BasicVector<T>* nest_arg2) {
                return arg2(&nest_arg1, nest_arg2);
              };
          return self->DeclareVectorOutputPort(arg1, wrapped);
        }, py_reference_internal)
    .def("_DeclarePeriodicPublish", &PyLeafSystem::DeclarePeriodicPublish,
         py::arg("period"), py::arg("offset") = 0.)
    .def("_DoPublish", &LeafSystemPublic::DoPublish);

  py::class_<Context<T>>(m, "Context")
    .def("get_num_input_ports", &Context<T>::get_num_input_ports)
    .def("FixInputPort",
         py::overload_cast<int, unique_ptr<BasicVector<T>>>(
             &Context<T>::FixInputPort), py_reference_internal,
         // Keep alive, ownership: `BasicVector` keeps `self` alive.
         py::keep_alive<3, 1>())
    .def("get_time", &Context<T>::get_time)
    .def("Clone", &Context<T>::Clone)
    .def("__copy__", &Context<T>::Clone)
    .def("get_state", &Context<T>::get_state, py_reference_internal)
    .def("get_mutable_state", &Context<T>::get_mutable_state,
         py_reference_internal);

  py::class_<LeafContext<T>, Context<T>>(m, "LeafContext");

  py::class_<Diagram<T>, System<T>>(m, "Diagram")
    .def("GetMutableSubsystemState",
        [](Diagram<T>* self, const System<T>& arg1, Context<T>* arg2)
        -> auto&& {
          // @note Use `auto&&` to get perfect forwarding.
          // @note Compiler does not like `py::overload_cast` with this setup?
          return self->GetMutableSubsystemState(arg1, arg2);
        }, py_reference,
        // Keep alive, ownership: `return` keeps `Context` alive.
        py::keep_alive<0, 3>());

  // Event mechanisms.
  py::class_<Event<T>>(m, "Event");
  py::class_<PublishEvent<T>, Event<T>>(m, "PublishEvent");

  // Glue mechanisms.
  py::class_<DiagramBuilder<T>>(m, "DiagramBuilder")
    .def(py::init<>())
    .def(
        "AddSystem",
        [](DiagramBuilder<T>* self, unique_ptr<System<T>> arg1) {
          return self->AddSystem(std::move(arg1));
        },
        // Keep alive, ownership: `System` keeps `self` alive.
        py::keep_alive<2, 1>())
    .def("Connect",
         py::overload_cast<const OutputPort<T>&, const InputPortDescriptor<T>&>(
             &DiagramBuilder<T>::Connect))
    .def("ExportInput", &DiagramBuilder<T>::ExportInput, py_reference_internal)
    .def("ExportOutput", &DiagramBuilder<T>::ExportOutput,
         py_reference_internal)
    .def("Build", &DiagramBuilder<T>::Build,
         // Keep alive, transitive: `return` keeps `self` alive.
         py::keep_alive<1, 0>())
    .def("BuildInto", &DiagramBuilder<T>::BuildInto,
         // Keep alive, transitive: `Diagram` keeps `self` alive.
         py::keep_alive<2, 1>());

  // TODO(eric.cousineau): Figure out how to handle template-specialized method
  // signatures(e.g. GetValue<T>()).
  py::class_<FreestandingInputPortValue>(m, "FreestandingInputPortValue");

  py::class_<OutputPort<T>>(m, "OutputPort");

  py::class_<SystemOutput<T>>(m, "SystemOutput")
    .def("get_num_ports", &SystemOutput<T>::get_num_ports)
    .def("get_vector_data", &SystemOutput<T>::get_vector_data,
         py_reference_internal);

  py::class_<InputPortDescriptor<T>>(m, "InputPortDescriptor");

  // Value types.
  py::class_<VectorBase<T>>(m, "VectorBase")
    .def("CopyToVector", &VectorBase<T>::CopyToVector)
    .def("SetFromVector", &VectorBase<T>::SetFromVector);

  // TODO(eric.cousineau): Make a helper function for the Eigen::Ref<> patterns.
  py::class_<BasicVector<T>, VectorBase<T>>(m, "BasicVector")
    .def(py::init<int>())
    .def(py::init<VectorX<T>>())
    .def("get_value",
        [](const BasicVector<T>* self) -> Eigen::Ref<const VectorX<T>> {
          return self->get_value();
        }, py_reference_internal)
    .def("get_mutable_value",
        [](BasicVector<T>* self) -> Eigen::Ref<VectorX<T>> {
          return self->get_mutable_value();
        }, py_reference_internal);

  py::class_<Supervector<T>, VectorBase<T>>(m, "Supervector");

  py::class_<Subvector<T>, VectorBase<T>>(m, "Subvector");

  // TODO(eric.cousineau): Interfacing with the C++ abstract value types may be
  // a tad challenging. This should be more straightforward once
  // scalar-type conversion is supported, as the template-exposure mechanisms
  // should be relatively similar.
  py::class_<AbstractValue>(m, "AbstractValue");

  // Parameters.
  // TODO(eric.cousineau): Fill this out.
  py::class_<Parameters<T>>(m, "Parameters");

  // State.
  py::class_<State<T>>(m, "State")
    .def(py::init<>())
    .def("get_continuous_state",
         &State<T>::get_continuous_state, py_reference_internal)
    .def("get_mutable_continuous_state",
         &State<T>::get_mutable_continuous_state, py_reference_internal)
    .def("get_discrete_state",
        &State<T>::get_discrete_state, py_reference_internal);

  // - Constituents.
  py::class_<ContinuousState<T>>(m, "ContinuousState")
    .def(py::init<>())
    .def("get_vector", &ContinuousState<T>::get_vector, py_reference_internal)
    .def("get_mutable_vector",
         &ContinuousState<T>::get_mutable_vector, py_reference_internal);

  py::class_<DiscreteValues<T>>(m, "DiscreteValues")
    .def("num_groups", &DiscreteValues<T>::num_groups)
    .def("get_data", &DiscreteValues<T>::get_data, py_reference_internal);

  py::class_<AbstractValues>(m, "AbstractValues");
}

}  // namespace pydrake
}  // namespace drake
