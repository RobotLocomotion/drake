#include "pybind11/eigen.h"
#include "pybind11/eval.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/bindings/pydrake/util/drake_optional_pybind.h"
#include "drake/bindings/pydrake/util/eigen_pybind.h"
#include "drake/bindings/pydrake/util/type_safe_index_pybind.h"
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
#include "drake/systems/framework/vector_system.h"

using std::make_unique;
using std::string;
using std::unique_ptr;
using std::vector;

namespace drake {
namespace pydrake {

namespace {

// TODO(eric.cousineau): This file is growing in size. Consider delegating
// these bindings to multiple source files.

// TODO(eric.cousineau): At present, we only bind doubles.
// In the future, we will bind more scalar types, and enable scalar
// conversion.
using T = double;

using systems::System;
using systems::LeafSystem;
using systems::Context;
using systems::VectorSystem;
using systems::PublishEvent;
using systems::DiscreteUpdateEvent;
using systems::DiscreteValues;

using pysystems::AddValueInstantiation;
using pysystems::DefClone;

class PySystem : public py::wrapper<System<T>> {
 public:
  using Base = py::wrapper<System<T>>;
  using Base::Base;
  // Expose protected methods for binding.
  using Base::DeclareInputPort;
};

class LeafSystemPublic : public LeafSystem<T> {
 public:
  using Base = LeafSystem<T>;
  using Base::Base;

  // N.B. These function methods are still typed as (LeafSystem<T>::*)(...),
  // since they are more or less visibility imports.
  // Defining methods here won't work, as it will become
  // (LeafSystemPublic::*)(...), since this typeid is not exposed in pybind.
  // If needed, solution is to expose it as an intermediate type if needed.

  // Expose protected methods for binding, no need for virtual overrides
  // (ordered by how they are bound).
  using Base::DeclareVectorOutputPort;
  using Base::DeclarePeriodicPublish;
  using Base::DeclareContinuousState;
  using Base::DeclareDiscreteState;
  using Base::DeclarePeriodicDiscreteUpdate;
  using Base::DeclareAbstractState;

  // Because `LeafSystem<T>::DoPublish` is protected, and we had to override
  // this method in `PyLeafSystem`, expose the method here for direct(-ish)
  // access.
  // (Otherwise, we get an error about inaccessible downcasting when trying to
  // bind `PyLeafSystem::DoPublish` to `py::class_<LeafSystem<T>, ...>`.
  using Base::DoPublish;
  using Base::DoHasDirectFeedthrough;
  using Base::DoCalcDiscreteVariableUpdates;
};

// Provide flexible inheritance to leverage prior binding information, per
// documentation:
// http://pybind11.readthedocs.io/en/stable/advanced/classes.html#combining-virtual-functions-and-inheritance  // NOLINT
template <typename LeafSystemBase = LeafSystemPublic>
class PyLeafSystemBase : public py::wrapper<LeafSystemBase> {
 public:
  using Base = py::wrapper<LeafSystemBase>;
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
    Base::DoPublish(context, events);
  }

  optional<bool> DoHasDirectFeedthrough(
      int input_port, int output_port) const override {
    PYBIND11_OVERLOAD_INT(
        optional<bool>, LeafSystem<T>, "_DoHasDirectFeedthrough",
        input_port, output_port);
    // If the macro did not return, use default functionality.
    return Base::DoHasDirectFeedthrough(input_port, output_port);
  }

  void DoCalcDiscreteVariableUpdates(
      const Context<T>& context,
      const std::vector<const DiscreteUpdateEvent<T>*>& events,
      DiscreteValues<T>* discrete_state) const override {
    // See `DoPublish` for explanation.
    PYBIND11_OVERLOAD_INT(
        void, LeafSystem<T>, "_DoCalcDiscreteVariableUpdates",
        &context, events, discrete_state);
    // If the macro did not return, use default functionality.
    Base::DoCalcDiscreteVariableUpdates(context, events, discrete_state);
  }
};

using PyLeafSystem = PyLeafSystemBase<>;

class VectorSystemPublic : public VectorSystem<T> {
 public:
  using Base = VectorSystem<T>;

  VectorSystemPublic(int inputs, int outputs)
      : Base(inputs, outputs) {}

  using Base::EvalVectorInput;
  using Base::GetVectorState;

  // Virtual methods, for explicit bindings.
  using Base::DoCalcVectorOutput;
  using Base::DoCalcVectorTimeDerivatives;
  using Base::DoCalcVectorDiscreteVariableUpdates;
};

class PyVectorSystem : public py::wrapper<VectorSystemPublic> {
 public:
  using Base = py::wrapper<VectorSystemPublic>;
  using Base::Base;

  // Trampoline virtual methods.
  void DoPublish(
      const Context<T>& context,
      const vector<const PublishEvent<T>*>& events) const override {
    // Copied from above, since we cannot use `PyLeafSystemBase` due to final
    // overrides of some methods.
    // TODO(eric.cousineau): Make this more granular?
    PYBIND11_OVERLOAD_INT(
        void, VectorSystem<T>, "_DoPublish", &context, events);
    // If the macro did not return, use default functionality.
    Base::DoPublish(context, events);
  }

  optional<bool> DoHasDirectFeedthrough(
      int input_port, int output_port) const override {
    PYBIND11_OVERLOAD_INT(
        optional<bool>, VectorSystem<T>, "_DoHasDirectFeedthrough",
        input_port, output_port);
    // If the macro did not return, use default functionality.
    return Base::DoHasDirectFeedthrough(input_port, output_port);
  }

  void DoCalcVectorOutput(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* output) const override {
    // WARNING: Mutating `output` will not work when T is AutoDiffXd,
    // Expression, etc.
    // @see https://github.com/pybind/pybind11/pull/1152#issuecomment-340091423
    // TODO(eric.cousineau): Either wrap to have this return a value (blech),
    // or solve the upstream issue.
    PYBIND11_OVERLOAD_INT(
        void, VectorSystem<T>, "_DoCalcVectorOutput",
        // N.B. Passing `Eigen::Map<>` derived classes by reference rather than
        // pointer to ensure conceptual clarity. pybind11 `type_caster`
        // struggles with types of `Map<Derived>*`, but not `Map<Derived>&`.
        &context, input, state, ToEigenRef(output));
    // If the macro did not return, use default functionality.
    Base::DoCalcVectorOutput(context, input, state, output);
  }

  void DoCalcVectorTimeDerivatives(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* derivatives) const override {
    // WARNING: Mutating `derivatives` will not work when T is AutoDiffXd,
    // Expression, etc. See above.
    PYBIND11_OVERLOAD_INT(
        void, VectorSystem<T>, "_DoCalcVectorTimeDerivatives",
        &context, input, state, ToEigenRef(derivatives));
    // If the macro did not return, use default functionality.
    Base::DoCalcVectorOutput(context, input, state, derivatives);
  }

  void DoCalcVectorDiscreteVariableUpdates(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* next_state) const override {
    // WARNING: Mutating `next_state` will not work when T is AutoDiffXd,
    // Expression, etc. See above.
    PYBIND11_OVERLOAD_INT(
        void, VectorSystem<T>, "_DoCalcVectorDiscreteVariableUpdates",
        &context, input, state, ToEigenRef(next_state));
    // If the macro did not return, use default functionality.
    Base::DoCalcVectorDiscreteVariableUpdates(
        context, input, state, next_state);
  }
};

}  // namespace

PYBIND11_MODULE(framework, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;

  m.doc() = "Bindings for the core Systems framework.";

  // TODO(eric.cousineau): Resolve `str_py` workaround.
  auto str_py = py::eval("str");

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
    // - Feedthrough.
    .def("HasAnyDirectFeedthrough", &System<T>::HasAnyDirectFeedthrough)
    .def("HasDirectFeedthrough",
         overload_cast_explicit<bool, int>(&System<T>::HasDirectFeedthrough),
         py::arg("output_port"))
    .def("HasDirectFeedthrough",
         overload_cast_explicit<bool, int, int>(
             &System<T>::HasDirectFeedthrough),
         py::arg("input_port"), py::arg("output_port"))
    // Context.
    .def("CreateDefaultContext", &System<T>::CreateDefaultContext)
    .def("AllocateOutput", &System<T>::AllocateOutput)
    .def(
        "EvalVectorInput",
        [](const System<T>* self, const Context<T>& arg1, int arg2) {
          return self->EvalVectorInput(arg1, arg2);
        }, py_reference,
        // Keep alive, ownership: `return` keeps `Context` alive.
        py::keep_alive<0, 2>())
    .def(
        "EvalAbstractInput",
        [](const System<T>* self, const Context<T>& arg1, int arg2) {
          return self->EvalAbstractInput(arg1, arg2);
        }, py_reference,
        // Keep alive, ownership: `return` keeps `Context` alive.
        py::keep_alive<0, 2>())
    .def("CalcOutput", &System<T>::CalcOutput)
    // Sugar.
    .def(
        "GetGraphvizString",
        [str_py](const System<T>* self) {
          // @note This is a workaround; for some reason,
          // casting this using `py::str` does not work, but directly
          // calling the Python function (`str_py`) does.
          return str_py(self->GetGraphvizString());
        })
    // Events.
    .def("Publish",
         overload_cast_explicit<void, const Context<T>&>(&System<T>::Publish));

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
         py::arg("period_sec"), py::arg("offset_sec") = 0.)
    .def("_DoPublish", &LeafSystemPublic::DoPublish)
    // System attributes.
    .def("_DoHasDirectFeedthrough",
         &LeafSystemPublic::DoHasDirectFeedthrough)
    // Continuous state.
    .def("_DeclareContinuousState",
         py::overload_cast<int>(&LeafSystemPublic::DeclareContinuousState),
         py::arg("num_state_variables"))
    .def("_DeclareContinuousState",
         py::overload_cast<int, int, int>(
            &LeafSystemPublic::DeclareContinuousState),
         py::arg("num_q"), py::arg("num_v"), py::arg("num_z"))
    .def("_DeclareContinuousState",
         py::overload_cast<const BasicVector<T>&>(
            &LeafSystemPublic::DeclareContinuousState),
         py::arg("model_vector"))
    // TODO(eric.cousineau): Ideally the downstream class of `BasicVector<T>`
    // should expose `num_q`, `num_v`, and `num_z`?
    .def("_DeclareContinuousState",
         py::overload_cast<const BasicVector<T>&, int, int, int>(
            &LeafSystemPublic::DeclareContinuousState),
         py::arg("model_vector"),
         py::arg("num_q"), py::arg("num_v"), py::arg("num_z"))
    // Discrete state.
    // TODO(eric.cousineau): Should there be a `BasicVector<>` overload?
    .def("_DeclareDiscreteState", &LeafSystemPublic::DeclareDiscreteState)
    .def("_DeclarePeriodicDiscreteUpdate",
         &LeafSystemPublic::DeclarePeriodicDiscreteUpdate,
         py::arg("period_sec"), py::arg("offset_sec") = 0.)
    .def("_DoCalcDiscreteVariableUpdates",
         &LeafSystemPublic::DoCalcDiscreteVariableUpdates)
    // Abstract state.
    .def("_DeclareAbstractState",
         &LeafSystemPublic::DeclareAbstractState,
         // Keep alive, ownership: `AbstractValue` keeps `self` alive.
         py::keep_alive<2, 1>());

  py::class_<Context<T>>(m, "Context")
    .def("get_num_input_ports", &Context<T>::get_num_input_ports)
    .def("FixInputPort",
         py::overload_cast<int, unique_ptr<BasicVector<T>>>(
             &Context<T>::FixInputPort),
         py_reference_internal,
         // Keep alive, ownership: `BasicVector` keeps `self` alive.
         py::keep_alive<3, 1>())
    .def("FixInputPort",
         py::overload_cast<int, unique_ptr<AbstractValue>>(
             &Context<T>::FixInputPort),
         py_reference_internal,
         // Keep alive, ownership: `AbstractValue` keeps `self` alive.
         py::keep_alive<3, 1>())
    .def("get_time", &Context<T>::get_time)
    .def("set_time", &Context<T>::set_time)
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

  py::class_<LeafContext<T>, Context<T>>(m, "LeafContext");

  py::class_<Diagram<T>, System<T>>(m, "Diagram")
    .def("GetMutableSubsystemState",
        [](Diagram<T>* self, const System<T>& arg1, Context<T>* arg2)
        -> auto& {
          // @note Compiler does not like `py::overload_cast` with this setup?
          return self->GetMutableSubsystemState(arg1, arg2);
        }, py_reference,
        // Keep alive, ownership: `return` keeps `Context` alive.
        py::keep_alive<0, 3>())
    .def("GetMutableSubsystemContext",
        [](Diagram<T>* self, const System<T>& arg1, Context<T>* arg2)
        -> auto&& {
          return self->GetMutableSubsystemContext(arg1, arg2);
        }, py_reference,
        // Keep alive, ownership: `return` keeps `Context` alive.
        py::keep_alive<0, 3>());

  // Event mechanisms.
  py::class_<Event<T>>(m, "Event");
  py::class_<PublishEvent<T>, Event<T>>(m, "PublishEvent");
  py::class_<DiscreteUpdateEvent<T>, Event<T>>(m, "DiscreteUpdateEvent");

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

  py::class_<OutputPort<T>>(m, "OutputPort")
    .def("size", &OutputPort<T>::size)
    .def("get_index", &OutputPort<T>::get_index);

  py::class_<SystemOutput<T>> system_output(m, "SystemOutput");
  DefClone(&system_output);
  system_output
    .def("get_num_ports", &SystemOutput<T>::get_num_ports)
    .def("get_data", &SystemOutput<T>::get_data,
         py_reference_internal)
    .def("get_vector_data", &SystemOutput<T>::get_vector_data,
         py_reference_internal);

  py::class_<InputPortDescriptor<T>>(m, "InputPortDescriptor")
    .def("size", &InputPortDescriptor<T>::size)
    .def("get_data_type", &InputPortDescriptor<T>::get_data_type)
    .def("get_index", &InputPortDescriptor<T>::get_index);

  // Value types.
  py::class_<VectorBase<T>>(m, "VectorBase")
    .def("CopyToVector", &VectorBase<T>::CopyToVector)
    .def("SetFromVector", &VectorBase<T>::SetFromVector)
    .def("size", &VectorBase<T>::size);

  // TODO(eric.cousineau): Make a helper function for the Eigen::Ref<> patterns.
  py::class_<BasicVector<T>, VectorBase<T>> basic_vector(m, "BasicVector");
  DefClone(&basic_vector);
  basic_vector
    // N.B. Place `init<VectorX<T>>` `init<int>` so that we do not implicitly
    // convert scalar-size `np.array` objects to `int` (since this is normally
    // permitted).
    .def(py::init<VectorX<T>>())
    .def(py::init<int>())
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

  // `AddValueInstantiation` will define methods specific to `T` for
  // `Value<T>`. Since Python is nominally dynamic, these methods are
  // effectively "virtual".
  auto abstract_stub = [](const std::string& method) {
    return [method](const AbstractValue* self, py::args, py::kwargs) {
      string type_name = NiceTypeName::Get(*self);
      throw std::runtime_error(
          "This derived class of `AbstractValue`, `" + type_name + "`, " +
          "is not exposed to pybind11, so `" + method + "` cannot be " +
          "called. See `AddValueInstantiation` for how to bind it.");
    };
  };

  py::class_<AbstractValue> abstract_value(m, "AbstractValue");
  DefClone(&abstract_value);
  abstract_value
    // Only bind the exception variant, `SetFromOrThrow`, for use in Python.
    // Otherwise, a user could encounter undefind behavior via `SetFrom`.
    .def("SetFrom", &AbstractValue::SetFromOrThrow)
    .def("get_value", abstract_stub("get_value"))
    .def("get_mutable_value", abstract_stub("get_mutable_value"))
    .def("set_value", abstract_stub("set_value"));

  // Add `Value<std::string>` instantiation (visible in Python as `Value[str]`).
  AddValueInstantiation<string>(m);

  // Add `Value[object]` instantiation.
  // N.B. If any code explicitly uses `Value<py::object>` for whatever reason,
  // then this should turn into a specialization of `Value<>`, rather than an
  // extension.
  class PyObjectValue : public Value<py::object> {
   public:
    using Base = Value<py::object>;
    using Base::Base;
    // Override `Value<py::object>::Clone()` to perform a deep copy on the
    // object.
    std::unique_ptr<AbstractValue> Clone() const override {
      py::object py_copy = py::module::import("copy").attr("deepcopy");
      return std::make_unique<PyObjectValue>(py_copy(get_value()));
    }
  };
  AddValueInstantiation<py::object, PyObjectValue>(m);

  py::object py_type_func = py::eval("type");
  py::object py_object_type = py::eval("object");
  // `Value` was defined by the first call to `AddValueInstantiation`.
  py::object py_value_template = m.attr("Value");
  abstract_value.def_static(
      "Make",
      [py_type_func, py_value_template, py_object_type](py::object value) {
        // Try to infer type from the object. If that does not work, just return
        // `Value[object]`.
        py::object py_type = py_type_func(value);
        py::tuple py_result =
            py_value_template.attr("get_instantiation")(py_type, false);
        py::object py_value_class = py_result[0];
        if (py_value_class.is_none()) {
          py_value_class = py_value_template[py_object_type];
        }
        return py_value_class(value);
      });

  // Parameters.
  py::class_<Parameters<T>> parameters(m, "Parameters");
  DefClone(&parameters);
  using BasicVectorPtrList = vector<unique_ptr<BasicVector<T>>>;
  using AbstractValuePtrList = vector<unique_ptr<AbstractValue>>;
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
  py::class_<State<T>>(m, "State")
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
  py::class_<ContinuousState<T>>(m, "ContinuousState")
    .def(py::init<>())
    .def("get_vector", &ContinuousState<T>::get_vector, py_reference_internal)
    .def("get_mutable_vector",
         &ContinuousState<T>::get_mutable_vector, py_reference_internal);

  py::class_<DiscreteValues<T>> discrete_values(m, "DiscreteValues");
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

  // Additional derivative Systems which are not glue, but do not fall under
  // `//systems/primitives`.

  // N.B. This will effectively allow derived classes of `VectorSystem` to
  // override `LeafSystem` methods, disrespecting `final`-ity.
  // This could be changed (see https://stackoverflow.com/a/2425785), but meh,
  // we're already abusing Python and C++ enough.
  py::class_<VectorSystem<T>, PyVectorSystem, LeafSystem<T>>(m, "VectorSystem")
    .def(py::init([](int inputs, int outputs) {
      return new PyVectorSystem(inputs, outputs);
    }));
    // TODO(eric.cousineau): Bind virtual methods once we provide a function
    // wrapper to convert `Map<Derived>*` arguments.
    // N.B. This could be mitigated by using `EigenPtr` in public interfaces in
    // upstream code.
}

}  // namespace pydrake
}  // namespace drake
