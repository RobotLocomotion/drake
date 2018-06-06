#include "drake/bindings/pydrake/systems/framework_py_systems.h"

#include "pybind11/eigen.h"
#include "pybind11/eval.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/bindings/pydrake/util/drake_optional_pybind.h"
#include "drake/bindings/pydrake/util/eigen_pybind.h"
#include "drake/bindings/pydrake/util/wrap_pybind.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_scalar_converter.h"
#include "drake/systems/framework/vector_system.h"

using std::make_unique;
using std::string;
using std::unique_ptr;
using std::vector;

namespace drake {
namespace pydrake {

namespace {

using symbolic::Expression;
using systems::System;
using systems::LeafSystem;
using systems::Context;
using systems::ContinuousState;
using systems::VectorSystem;
using systems::PublishEvent;
using systems::DiscreteUpdateEvent;
using systems::DiscreteValues;
using systems::SystemScalarConverter;

// Provides a templated 'namespace'.
template <typename T>
struct Impl {
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

    // Explicitly forward constructors as opposed to `using Base::Base`, as we
    // want the protected `SystemScalarConverter` exposed publicly.
    LeafSystemPublic() = default;
    explicit LeafSystemPublic(SystemScalarConverter converter)
        : Base(std::move(converter)) {}

    // N.B. These function methods are still typed as (LeafSystem<T>::*)(...),
    // since they are more or less visibility imports.
    // Defining methods here won't work, as it will become
    // (LeafSystemPublic::*)(...), since this typeid is not exposed in pybind.
    // If needed, solution is to expose it as an intermediate type if needed.

    // Expose protected methods for binding, no need for virtual overrides
    // (ordered by how they are bound).
    using Base::DeclareAbstractOutputPort;
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
    using Base::DoCalcTimeDerivatives;
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

    void DoCalcTimeDerivatives(
        const Context<T>& context,
        ContinuousState<T>* derivatives) const override {
      // See `DoPublish` for explanation.
      PYBIND11_OVERLOAD_INT(
          void, LeafSystem<T>, "_DoCalcTimeDerivatives",
          &context, derivatives);
      // If the macro did not return, use default functionality.
      Base::DoCalcTimeDerivatives(context, derivatives);
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
      // Expression, etc. See
      // https://github.com/pybind/pybind11/pull/1152#issuecomment-340091423
      // TODO(eric.cousineau): This will be resolved once dtype=custom is
      // resolved.
      PYBIND11_OVERLOAD_INT(
          void, VectorSystem<T>, "_DoCalcVectorOutput",
          // N.B. Passing `Eigen::Map<>` derived classes by reference rather
          // than pointer to ensure conceptual clarity. pybind11 `type_caster`
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

  static void DoDefinitions(py::module m) {
    // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
    using namespace drake::systems;

    // TODO(eric.cousineau): Resolve `str_py` workaround.
    auto str_py = py::eval("str");

    // TODO(eric.cousineau): Show constructor, but somehow make sure `pybind11`
    // knows this is abstract?
    DefineTemplateClassWithDefault<System<T>, PySystem>(
        m, "System", GetPyParam<T>())
        .def("set_name", &System<T>::set_name)
        // Topology.
        .def("get_num_input_ports", &System<T>::get_num_input_ports)
        .def("get_input_port",
             &System<T>::get_input_port, py_reference_internal)
        .def("get_num_output_ports", &System<T>::get_num_output_ports)
        .def("get_output_port",
             &System<T>::get_output_port, py_reference_internal)
        .def(
            "_DeclareInputPort", &PySystem::DeclareInputPort,
            py_reference_internal,
            py::arg("type"), py::arg("size"), py::arg("random_type") = nullopt)
        // - Feedthrough.
        .def("HasAnyDirectFeedthrough", &System<T>::HasAnyDirectFeedthrough)
        .def("HasDirectFeedthrough",
             overload_cast_explicit<bool, int>(
                 &System<T>::HasDirectFeedthrough),
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
             overload_cast_explicit<void, const Context<T>&>(
                 &System<T>::Publish))
        // Scalar types.
        .def("ToAutoDiffXd", [](const System<T>& self) {
           return self.ToAutoDiffXd();
        })
        .def("ToAutoDiffXdMaybe", &System<T>::ToAutoDiffXdMaybe)
        .def("ToSymbolic", [](const System<T>& self) {
           return self.ToSymbolic();
        })
        .def("ToSymbolicMaybe", &System<T>::ToSymbolicMaybe);

    using AllocCallback = typename LeafOutputPort<T>::AllocCallback;
    using CalcCallback = typename LeafOutputPort<T>::CalcCallback;
    using CalcVectorCallback = typename LeafOutputPort<T>::CalcVectorCallback;

    DefineTemplateClassWithDefault<LeafSystem<T>, PyLeafSystem, System<T>>(
      m, "LeafSystem", GetPyParam<T>())
      .def(py::init<>())
      // TODO(eric.cousineau): It'd be nice if we did not need the user to
      // propagate scalar conversion information. Ideally, if we could
      // intercept `self` at this point, when constructing `PyLeafSystem` for
      // extending Python, we could figure out what user-defined template is
      // being used, and pass that as the converter. However, that requires an
      // old-style `py::init`, which is deprecated in Python...
      .def(py::init<SystemScalarConverter>(), py::arg("converter"))
      .def(
          "_DeclareAbstractOutputPort",
          WrapCallbacks(
              [](PyLeafSystem* self, AllocCallback arg1,
                 CalcCallback arg2) -> auto& {
                return self->DeclareAbstractOutputPort(arg1, arg2);
              }),
          py_reference_internal)
      .def(
          "_DeclareVectorOutputPort",
          WrapCallbacks(
              [](PyLeafSystem* self, const BasicVector<T>& arg1,
                 CalcVectorCallback arg2) -> auto& {
                return self->DeclareVectorOutputPort(arg1, arg2);
              }),
          py_reference_internal)
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
      .def("_DoCalcTimeDerivatives", &LeafSystemPublic::DoCalcTimeDerivatives)
      .def("_DoCalcDiscreteVariableUpdates",
           &LeafSystemPublic::DoCalcDiscreteVariableUpdates)
      // Abstract state.
      .def("_DeclareAbstractState",
           &LeafSystemPublic::DeclareAbstractState,
           // Keep alive, ownership: `AbstractValue` keeps `self` alive.
           py::keep_alive<2, 1>());

    DefineTemplateClassWithDefault<Diagram<T>, System<T>>(
        m, "Diagram", GetPyParam<T>())
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

    // N.B. This will effectively allow derived classes of `VectorSystem` to
    // override `LeafSystem` methods, disrespecting `final`-ity.
    // This could be changed (see https://stackoverflow.com/a/2425785), but meh,
    // we're already abusing Python and C++ enough.
    DefineTemplateClassWithDefault<
        VectorSystem<T>, PyVectorSystem, LeafSystem<T>>(
        m, "VectorSystem", GetPyParam<T>())
        .def(py::init([](int inputs, int outputs) {
          return new PyVectorSystem(inputs, outputs);
        }));
    // TODO(eric.cousineau): Bind virtual methods once we provide a function
    // wrapper to convert `Map<Derived>*` arguments.
    // N.B. This could be mitigated by using `EigenPtr` in public interfaces in
    // upstream code.
  }
};

template <typename ... Packs>
py::tuple GetPyParamList(type_pack<Packs...> = {}) {
  return py::make_tuple(GetPyParam(Packs{})...);
}

}  // namespace

void DefineFrameworkPySystems(py::module m) {
  // System scalar conversion.
  py::class_<SystemScalarConverter> converter(m, "SystemScalarConverter");
  converter
    .def(py::init())
    .def("__copy__",
         [](const SystemScalarConverter& in) -> SystemScalarConverter {
           return in;
         });
  // Bind templated instantiations.
  auto converter_methods = [converter](auto pack) {
    using Pack = decltype(pack);
    using T = typename Pack::template type_at<0>;
    using U = typename Pack::template type_at<1>;
    AddTemplateMethod(
        converter, "Add", WrapCallbacks(&SystemScalarConverter::Add<T, U>),
        GetPyParam<T, U>());
    AddTemplateMethod(
        converter, "IsConvertible",
        &SystemScalarConverter::IsConvertible<T, U>, GetPyParam<T, U>());
  };
  // N.B. When changing the pairs of supported types below, ensure that these
  // reflect the stanzas for the advanced constructor of
  // `SystemScalarConverter`.
  using ConversionPairs = type_pack<
      type_pack<AutoDiffXd, double>,
      type_pack<Expression, double>,
      type_pack<double, AutoDiffXd>,
      type_pack<Expression, AutoDiffXd>,
      type_pack<double, Expression>,
      type_pack<AutoDiffXd, Expression>
      >;
  type_visit(converter_methods, ConversionPairs{});
  // Add mention of what scalars are supported via `SystemScalarConverter`
  // through Python.
  converter.attr("SupportedScalars") =
      GetPyParam(pysystems::CommonScalarPack{});
  converter.attr("SupportedConversionPairs") =
      GetPyParamList(ConversionPairs{});

  // Do templated instantiations of system types.
  auto bind_common_scalar_types = [m](auto dummy) {
    using T = decltype(dummy);
    Impl<T>::DoDefinitions(m);
  };
  type_visit(bind_common_scalar_types, pysystems::CommonScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
