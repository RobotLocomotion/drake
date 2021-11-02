#include "drake/bindings/pydrake/systems/framework_py_systems.h"

#include "pybind11/eigen.h"
#include "pybind11/eval.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_html.h"
#include "drake/systems/framework/system_scalar_converter.h"
#include "drake/systems/framework/vector_system.h"
#include "drake/systems/framework/witness_function.h"

using std::make_unique;
using std::string;
using std::unique_ptr;
using std::vector;

namespace drake {
namespace pydrake {

namespace {

using symbolic::Expression;
using systems::Context;
using systems::ContinuousState;
using systems::Diagram;
using systems::DiscreteUpdateEvent;
using systems::DiscreteValues;
using systems::LeafSystem;
using systems::PublishEvent;
using systems::System;
using systems::SystemBase;
using systems::SystemScalarConverter;
using systems::VectorSystem;
using systems::WitnessFunction;

class SystemBasePublic : public SystemBase {
 public:
  using SystemBase::DeclareCacheEntry;
};

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
    using Base::DeclareAbstractInputPort;
    using Base::DeclareAbstractOutputPort;
    using Base::DeclareAbstractParameter;
    using Base::DeclareAbstractState;
    using Base::DeclareContinuousState;
    using Base::DeclareDiscreteState;
    using Base::DeclareInitializationEvent;
    using Base::DeclareNumericParameter;
    using Base::DeclarePeriodicDiscreteUpdate;
    using Base::DeclarePeriodicEvent;
    using Base::DeclarePeriodicPublish;
    using Base::DeclarePerStepEvent;
    using Base::DeclareStateOutputPort;
    using Base::DeclareVectorInputPort;
    using Base::DeclareVectorOutputPort;
    using Base::MakeWitnessFunction;

    // Because `LeafSystem<T>::DoPublish` is protected, and we had to override
    // this method in `PyLeafSystem`, expose the method here for direct(-ish)
    // access.
    // (Otherwise, we get an error about inaccessible downcasting when trying to
    // bind `PyLeafSystem::DoPublish` to `py::class_<LeafSystem<T>, ...>`.
    using Base::DoCalcDiscreteVariableUpdates;
    using Base::DoCalcTimeDerivatives;
    using Base::DoPublish;
  };

  // Provide flexible inheritance to leverage prior binding information, per
  // documentation:
  // http://pybind11.readthedocs.io/en/stable/advanced/classes.html#combining-virtual-functions-and-inheritance
  template <typename LeafSystemBase = LeafSystemPublic>
  class PyLeafSystemBase : public py::wrapper<LeafSystemBase> {
   public:
    using Base = py::wrapper<LeafSystemBase>;
    using Base::Base;

    // Trampoline virtual methods.
    void DoPublish(const Context<T>& context,
        const vector<const PublishEvent<T>*>& events) const override {
      // Yuck! We have to dig in and use internals :(
      // We must ensure that pybind only sees pointers, since this method may
      // be called from C++, and pybind will not have seen these objects yet.
      // @see https://github.com/pybind/pybind11/issues/1241
      // TODO(eric.cousineau): Figure out how to supply different behavior,
      // possibly using function wrapping.
      PYBIND11_OVERLOAD_INT(void, LeafSystem<T>, "DoPublish", &context, events);
      // If the macro did not return, use default functionality.
      Base::DoPublish(context, events);
    }

    void DoCalcTimeDerivatives(const Context<T>& context,
        ContinuousState<T>* derivatives) const override {
      // See `DoPublish` for explanation.
      PYBIND11_OVERLOAD_INT(
          void, LeafSystem<T>, "DoCalcTimeDerivatives", &context, derivatives);
      // If the macro did not return, use default functionality.
      Base::DoCalcTimeDerivatives(context, derivatives);
    }

    void DoCalcDiscreteVariableUpdates(const Context<T>& context,
        const std::vector<const DiscreteUpdateEvent<T>*>& events,
        DiscreteValues<T>* discrete_state) const override {
      // See `DoPublish` for explanation.
      PYBIND11_OVERLOAD_INT(void, LeafSystem<T>,
          "DoCalcDiscreteVariableUpdates", &context, events, discrete_state);
      // If the macro did not return, use default functionality.
      Base::DoCalcDiscreteVariableUpdates(context, events, discrete_state);
    }

    // This actually changes the signature of DoGetWitnessFunction,
    // expecting the python overload to return a list of witnesses (instead
    // of taking in an empty pointer to std::vector<>.
    // TODO(russt): This is actually a System method, so make a PySystem
    // trampoline if this is needed outside of LeafSystem.
    void DoGetWitnessFunctions(const Context<T>& context,
        std::vector<const WitnessFunction<T>*>* witnesses) const override {
      auto wrapped = [&]() -> std::vector<const WitnessFunction<T>*> {
        PYBIND11_OVERLOAD_INT(std::vector<const WitnessFunction<T>*>,
            LeafSystem<T>, "DoGetWitnessFunctions", &context);
        std::vector<const WitnessFunction<T>*> result;
        // If the macro did not return, use default functionality.
        Base::DoGetWitnessFunctions(context, &result);
        return result;
      };
      *witnesses = wrapped();
    }
  };

  using PyLeafSystem = PyLeafSystemBase<>;

  class DiagramPublic : public Diagram<T> {
   public:
    using Base = Diagram<T>;

    DiagramPublic() = default;
  };

  // Provide flexible inheritance to leverage prior binding information, per
  // documentation:
  // http://pybind11.readthedocs.io/en/stable/advanced/classes.html#combining-virtual-functions-and-inheritance
  template <typename DiagramBase = DiagramPublic>
  class PyDiagramBase : public py::wrapper<DiagramBase> {
   public:
    using Base = py::wrapper<DiagramBase>;
    using Base::Base;
  };

  using PyDiagram = PyDiagramBase<>;

  class VectorSystemPublic : public VectorSystem<T> {
   public:
    using Base = VectorSystem<T>;

    VectorSystemPublic(
        int input_size, int output_size, std::optional<bool> direct_feedthrough)
        : Base(input_size, output_size, direct_feedthrough) {}

    using Base::EvalVectorInput;
    using Base::GetVectorState;

    // Virtual methods, for explicit bindings.
    using Base::DoCalcVectorDiscreteVariableUpdates;
    using Base::DoCalcVectorOutput;
    using Base::DoCalcVectorTimeDerivatives;
  };

  class PyVectorSystem : public py::wrapper<VectorSystemPublic> {
   public:
    using Base = py::wrapper<VectorSystemPublic>;
    using Base::Base;

    // Trampoline virtual methods.
    void DoPublish(const Context<T>& context,
        const vector<const PublishEvent<T>*>& events) const override {
      // Copied from above, since we cannot use `PyLeafSystemBase` due to final
      // overrides of some methods.
      // TODO(eric.cousineau): Make this more granular?
      PYBIND11_OVERLOAD_INT(
          void, VectorSystem<T>, "DoPublish", &context, events);
      // If the macro did not return, use default functionality.
      Base::DoPublish(context, events);
    }

    void DoCalcVectorOutput(const Context<T>& context,
        const Eigen::VectorBlock<const VectorX<T>>& input,
        const Eigen::VectorBlock<const VectorX<T>>& state,
        Eigen::VectorBlock<VectorX<T>>* output) const override {
      // WARNING: Mutating `output` will not work when T is AutoDiffXd,
      // Expression, etc. See
      // https://github.com/pybind/pybind11/pull/1152#issuecomment-340091423
      // TODO(eric.cousineau): This will be resolved once dtype=custom is
      // resolved.
      PYBIND11_OVERLOAD_INT(void, VectorSystem<T>, "DoCalcVectorOutput",
          // N.B. Passing `Eigen::Map<>` derived classes by reference rather
          // than pointer to ensure conceptual clarity. pybind11 `type_caster`
          // struggles with types of `Map<Derived>*`, but not `Map<Derived>&`.
          &context, input, state, ToEigenRef(output));
      // If the macro did not return, use default functionality.
      Base::DoCalcVectorOutput(context, input, state, output);
    }

    void DoCalcVectorTimeDerivatives(const Context<T>& context,
        const Eigen::VectorBlock<const VectorX<T>>& input,
        const Eigen::VectorBlock<const VectorX<T>>& state,
        Eigen::VectorBlock<VectorX<T>>* derivatives) const override {
      // WARNING: Mutating `derivatives` will not work when T is AutoDiffXd,
      // Expression, etc. See above.
      PYBIND11_OVERLOAD_INT(void, VectorSystem<T>,
          "DoCalcVectorTimeDerivatives", &context, input, state,
          ToEigenRef(derivatives));
      // If the macro did not return, use default functionality.
      Base::DoCalcVectorOutput(context, input, state, derivatives);
    }

    void DoCalcVectorDiscreteVariableUpdates(const Context<T>& context,
        const Eigen::VectorBlock<const VectorX<T>>& input,
        const Eigen::VectorBlock<const VectorX<T>>& state,
        Eigen::VectorBlock<VectorX<T>>* next_state) const override {
      // WARNING: Mutating `next_state` will not work when T is AutoDiffXd,
      // Expression, etc. See above.
      PYBIND11_OVERLOAD_INT(void, VectorSystem<T>,
          "DoCalcVectorDiscreteVariableUpdates", &context, input, state,
          ToEigenRef(next_state));
      // If the macro did not return, use default functionality.
      Base::DoCalcVectorDiscreteVariableUpdates(
          context, input, state, next_state);
    }
  };

  static void DoScalarDependentDefinitions(py::module m) {
    // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
    using namespace drake::systems;
    constexpr auto& doc = pydrake_doc.drake.systems;

    // TODO(eric.cousineau): Resolve `str_py` workaround.
    auto str_py = py::eval("str");

    // TODO(eric.cousineau): Show constructor, but somehow make sure `pybind11`
    // knows this is abstract?
    auto system_cls =
        DefineTemplateClassWithDefault<System<T>, SystemBase, PySystem>(
            m, "System", GetPyParam<T>(), doc.System.doc);
    system_cls  // BR
        .def("get_input_port",
            overload_cast_explicit<const InputPort<T>&, int>(
                &System<T>::get_input_port),
            py_rvp::reference_internal, py::arg("port_index"),
            doc.System.get_input_port.doc_1args)
        .def("get_input_port",
            overload_cast_explicit<const InputPort<T>&>(
                &System<T>::get_input_port),
            py_rvp::reference_internal, doc.System.get_input_port.doc_0args)
        .def("GetInputPort", &System<T>::GetInputPort,
            py_rvp::reference_internal, py::arg("port_name"),
            doc.System.GetInputPort.doc)
        .def("get_output_port",
            overload_cast_explicit<const OutputPort<T>&, int>(
                &System<T>::get_output_port),
            py_rvp::reference_internal, py::arg("port_index"),
            doc.System.get_output_port.doc_1args)
        .def("get_output_port",
            overload_cast_explicit<const OutputPort<T>&>(
                &System<T>::get_output_port),
            py_rvp::reference_internal, doc.System.get_output_port.doc_0args)
        .def("GetOutputPort", &System<T>::GetOutputPort,
            py_rvp::reference_internal, py::arg("port_name"),
            doc.System.GetOutputPort.doc)
        .def("DeclareInputPort",
            overload_cast_explicit<InputPort<T>&,
                std::variant<std::string, UseDefaultName>, PortDataType, int,
                std::optional<RandomDistribution>>(&PySystem::DeclareInputPort),
            py_rvp::reference_internal, py::arg("name"), py::arg("type"),
            py::arg("size"), py::arg("random_type") = std::nullopt,
            doc.System.DeclareInputPort.doc)
        // Feedthrough.
        .def("HasAnyDirectFeedthrough", &System<T>::HasAnyDirectFeedthrough,
            doc.System.HasAnyDirectFeedthrough.doc)
        .def("HasDirectFeedthrough",
            overload_cast_explicit<bool, int>(  // BR
                &System<T>::HasDirectFeedthrough),
            py::arg("output_port"), doc.System.HasDirectFeedthrough.doc_1args)
        .def("HasDirectFeedthrough",
            overload_cast_explicit<bool, int, int>(
                &System<T>::HasDirectFeedthrough),
            py::arg("input_port"), py::arg("output_port"),
            doc.System.HasDirectFeedthrough.doc_2args)
        // Context.
        .def("AllocateContext", &System<T>::AllocateContext,
            doc.System.AllocateContext.doc)
        .def("CreateDefaultContext", &System<T>::CreateDefaultContext,
            doc.System.CreateDefaultContext.doc)
        .def("SetDefaultContext", &System<T>::SetDefaultContext,
            doc.System.SetDefaultContext.doc)
        .def("SetRandomContext", &System<T>::SetRandomContext,
            py::arg("context"), py::arg("generator"),
            doc.System.SetRandomContext.doc)
        .def("AllocateOutput",
            overload_cast_explicit<unique_ptr<SystemOutput<T>>>(
                &System<T>::AllocateOutput),
            doc.System.AllocateOutput.doc)
        .def("AllocateTimeDerivatives",
            overload_cast_explicit<unique_ptr<ContinuousState<T>>>(
                &System<T>::AllocateTimeDerivatives),
            doc.System.AllocateTimeDerivatives.doc)
        .def("AllocateDiscreteVariables",
            overload_cast_explicit<unique_ptr<DiscreteValues<T>>>(
                &System<T>::AllocateDiscreteVariables),
            doc.System.AllocateDiscreteVariables.doc)
        .def(
            "EvalVectorInput",
            [](const System<T>* self, const Context<T>& arg1, int arg2) {
              return self->EvalVectorInput(arg1, arg2);
            },
            py_rvp::reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 2>(), doc.System.EvalVectorInput.doc)
        .def(
            "EvalAbstractInput",
            [](const System<T>* self, const Context<T>& arg1, int arg2) {
              return self->EvalAbstractInput(arg1, arg2);
            },
            py_rvp::reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 2>(), doc.SystemBase.EvalAbstractInput.doc)
        // Computation.
        .def("CalcOutput", &System<T>::CalcOutput, py::arg("context"),
            py::arg("outputs"), doc.System.CalcOutput.doc)
        .def("CalcPotentialEnergy", &System<T>::CalcPotentialEnergy,
            py::arg("context"), doc.System.CalcPotentialEnergy.doc)
        .def("CalcKineticEnergy", &System<T>::CalcKineticEnergy,
            py::arg("context"), doc.System.CalcKineticEnergy.doc)
        .def("CalcConservativePower", &System<T>::CalcConservativePower,
            py::arg("context"), doc.System.CalcConservativePower.doc)
        .def("CalcNonConservativePower", &System<T>::CalcNonConservativePower,
            py::arg("context"), doc.System.CalcNonConservativePower.doc)
        .def("CalcTimeDerivatives", &System<T>::CalcTimeDerivatives,
            py::arg("context"), py::arg("derivatives"),
            doc.System.CalcTimeDerivatives.doc)
        .def("CalcDiscreteVariableUpdates",
            overload_cast_explicit<void, const Context<T>&, DiscreteValues<T>*>(
                &System<T>::CalcDiscreteVariableUpdates),
            py::arg("context"), py::arg("discrete_state"),
            doc.System.CalcDiscreteVariableUpdates.doc_2args)
        .def("GetSubsystemContext",
            overload_cast_explicit<const Context<T>&, const System<T>&,
                const Context<T>&>(&System<T>::GetSubsystemContext),
            py_rvp::reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 3>(), doc.System.GetMutableSubsystemContext.doc)
        .def("GetMutableSubsystemContext",
            overload_cast_explicit<Context<T>&, const System<T>&, Context<T>*>(
                &System<T>::GetMutableSubsystemContext),
            py_rvp::reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 3>(), doc.System.GetMutableSubsystemContext.doc)
        .def("GetMyContextFromRoot",
            overload_cast_explicit<const Context<T>&, const Context<T>&>(
                &System<T>::GetMyContextFromRoot),
            py_rvp::reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 2>(), doc.System.GetMyMutableContextFromRoot.doc)
        .def("GetMyMutableContextFromRoot",
            overload_cast_explicit<Context<T>&, Context<T>*>(
                &System<T>::GetMyMutableContextFromRoot),
            py_rvp::reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 2>(), doc.System.GetMyMutableContextFromRoot.doc)
        // Sugar.
        .def(
            "GetGraphvizString",
            [str_py](const System<T>* self, int max_depth) {
              // @note This is a workaround; for some reason,
              // casting this using `py::str` does not work, but directly
              // calling the Python function (`str_py`) does.
              return str_py(self->GetGraphvizString(max_depth));
            },
            py::arg("max_depth") = std::numeric_limits<int>::max(),
            doc.System.GetGraphvizString.doc)
        // Events.
        .def("Publish",
            overload_cast_explicit<void, const Context<T>&>(
                &System<T>::Publish),
            doc.System.Publish.doc_1args)
        .def("GetUniquePeriodicDiscreteUpdateAttribute",
            &System<T>::GetUniquePeriodicDiscreteUpdateAttribute,
            doc.System.GetUniquePeriodicDiscreteUpdateAttribute.doc)
        .def(
            "IsDifferenceEquationSystem",
            [](const System<T>& self) {
              double period = 0.0;
              bool retval = self.IsDifferenceEquationSystem(&period);
              return std::pair<bool, double>(retval, period);
            },
            (string(doc.System.IsDifferenceEquationSystem.doc) + R""(
Note: The above is for the C++ documentation. For Python, use
`is_diff_eq, period = IsDifferenceEquationSystem()`)"")
                .c_str())
        // Cached evaluations.
        .def("EvalTimeDerivatives", &System<T>::EvalTimeDerivatives,
            py_rvp::reference_internal, doc.System.EvalTimeDerivatives.doc)
        .def("EvalPotentialEnergy", &System<T>::EvalPotentialEnergy,
            py::arg("context"), doc.System.EvalPotentialEnergy.doc)
        .def("EvalKineticEnergy", &System<T>::EvalKineticEnergy,
            py::arg("context"), doc.System.EvalKineticEnergy.doc)
        // Scalar types.
        .def(
            "ToAutoDiffXd",
            [](const System<T>& self) { return self.ToAutoDiffXd(); },
            doc.System.ToAutoDiffXd.doc_0args)
        .def("ToAutoDiffXdMaybe", &System<T>::ToAutoDiffXdMaybe,
            doc.System.ToAutoDiffXdMaybe.doc)
        .def(
            "ToSymbolic",
            [](const System<T>& self) { return self.ToSymbolic(); },
            doc.System.ToSymbolic.doc_0args)
        .def("ToSymbolicMaybe", &System<T>::ToSymbolicMaybe,
            doc.System.ToSymbolicMaybe.doc)
        .def("FixInputPortsFrom", &System<T>::FixInputPortsFrom,
            py::arg("other_system"), py::arg("other_context"),
            py::arg("target_context"), doc.System.FixInputPortsFrom.doc)
        .def(
            "GetWitnessFunctions",
            [](const System<T>& self, const Context<T>& context) {
              std::vector<const WitnessFunction<T>*> witnesses;
              self.GetWitnessFunctions(context, &witnesses);
              return witnesses;
            },
            py::arg("context"),
            (string(doc.System.DoGetWitnessFunctions.doc) + R""(
Note: The above is for the C++ documentation. For Python, use
`witnesses = GetWitnessFunctions(context)`)"")
                .c_str());
    auto def_to_scalar_type = [&system_cls, doc](auto dummy) {
      using U = decltype(dummy);
      AddTemplateMethod(
          system_cls, "ToScalarType",
          [](const System<T>& self) { return self.template ToScalarType<U>(); },
          GetPyParam<U>(), doc.System.ToScalarType.doc_0args);
    };
    type_visit(def_to_scalar_type, CommonScalarPack{});

    auto def_to_scalar_type_maybe = [&system_cls, doc](auto dummy) {
      using U = decltype(dummy);
      AddTemplateMethod(system_cls, "ToScalarTypeMaybe",
          &System<T>::template ToScalarTypeMaybe<U>, GetPyParam<U>(),
          doc.System.ToScalarTypeMaybe.doc);
    };
    type_visit(def_to_scalar_type_maybe, CommonScalarPack{});

    using AllocCallback = typename LeafOutputPort<T>::AllocCallback;
    using CalcCallback = typename LeafOutputPort<T>::CalcCallback;
    using CalcVectorCallback = typename LeafOutputPort<T>::CalcVectorCallback;

    auto leaf_system_cls =
        DefineTemplateClassWithDefault<LeafSystem<T>, PyLeafSystem, System<T>>(
            m, "LeafSystem", GetPyParam<T>(), doc.LeafSystem.doc);
    leaf_system_cls  // BR
        .def(py::init<>(), doc.LeafSystem.ctor.doc_0args)
        // TODO(eric.cousineau): It'd be nice if we did not need the user to
        // propagate scalar conversion information. Ideally, if we could
        // intercept `self` at this point, when constructing `PyLeafSystem` for
        // extending Python, we could figure out what user-defined template is
        // being used, and pass that as the converter. However, that requires an
        // old-style `py::init`, which is deprecated in Python...
        .def(py::init<SystemScalarConverter>(), py::arg("converter"),
            doc.LeafSystem.ctor.doc_1args)
        .def(
            "DeclareAbstractInputPort",
            [](PyLeafSystem* self, const std::string& name,
                const AbstractValue& model_value) -> const InputPort<T>& {
              return self->DeclareAbstractInputPort(name, model_value);
            },
            py_rvp::reference_internal, py::arg("name"), py::arg("model_value"),
            doc.LeafSystem.DeclareAbstractInputPort.doc)
        .def("DeclareAbstractParameter",
            &PyLeafSystem::DeclareAbstractParameter, py::arg("model_value"),
            doc.LeafSystem.DeclareAbstractParameter.doc)
        .def("DeclareNumericParameter", &PyLeafSystem::DeclareNumericParameter,
            py::arg("model_vector"), doc.LeafSystem.DeclareNumericParameter.doc)
        .def("DeclareAbstractOutputPort",
            WrapCallbacks([](PyLeafSystem* self, const std::string& name,
                              AllocCallback arg1, CalcCallback arg2,
                              const std::set<DependencyTicket>& arg3)
                              -> const OutputPort<T>& {
              return self->DeclareAbstractOutputPort(name, arg1, arg2, arg3);
            }),
            py_rvp::reference_internal, py::arg("name"), py::arg("alloc"),
            py::arg("calc"),
            py::arg("prerequisites_of_calc") =
                std::set<DependencyTicket>{SystemBase::all_sources_ticket()},
            doc.LeafSystem.DeclareAbstractOutputPort
                .doc_4args_name_alloc_function_calc_function_prerequisites_of_calc)
        .def(
            "DeclareVectorInputPort",
            [](PyLeafSystem* self, std::string name,
                const BasicVector<T>& model_vector,
                std::optional<RandomDistribution> random_type)
                -> InputPort<T>& {
              return self->DeclareVectorInputPort(
                  name, model_vector, random_type);
            },
            py_rvp::reference_internal, py::arg("name"),
            py::arg("model_vector"), py::arg("random_type") = std::nullopt,
            doc.LeafSystem.DeclareVectorInputPort.doc_3args_model_vector)
        .def(
            "DeclareVectorInputPort",
            [](PyLeafSystem* self, std::string name, int size,
                std::optional<RandomDistribution> random_type)
                -> InputPort<T>& {
              return self->DeclareVectorInputPort(name, size, random_type);
            },
            py_rvp::reference_internal, py::arg("name"), py::arg("size"),
            py::arg("random_type") = std::nullopt,
            doc.LeafSystem.DeclareVectorInputPort.doc_3args_size)
        .def("DeclareVectorOutputPort",
            WrapCallbacks(
                [](PyLeafSystem* self, const std::string& name,
                    const BasicVector<T>& arg1, CalcVectorCallback arg2,
                    const std::set<DependencyTicket>& arg3)
                    -> const OutputPort<T>& {
                  return self->DeclareVectorOutputPort(name, arg1, arg2, arg3);
                }),
            py_rvp::reference_internal, py::arg("name"), py::arg("model_value"),
            py::arg("calc"),
            py::arg("prerequisites_of_calc") =
                std::set<DependencyTicket>{SystemBase::all_sources_ticket()},
            doc.LeafSystem.DeclareVectorOutputPort.doc_4args_model_vector)
        .def("DeclareVectorOutputPort",
            WrapCallbacks(
                [](PyLeafSystem* self, const std::string& name, int size,
                    CalcVectorCallback calc,
                    const std::set<DependencyTicket>& prerequisites_of_calc)
                    -> const OutputPort<T>& {
                  return self->DeclareVectorOutputPort(
                      name, size, calc, prerequisites_of_calc);
                }),
            py_rvp::reference_internal, py::arg("name"), py::arg("size"),
            py::arg("calc"),
            py::arg("prerequisites_of_calc") =
                std::set<DependencyTicket>{SystemBase::all_sources_ticket()},
            doc.LeafSystem.DeclareVectorOutputPort.doc_4args_size)
        .def("DeclareStateOutputPort",
            py::overload_cast<std::variant<std::string, UseDefaultName>,
                ContinuousStateIndex>(
                &LeafSystemPublic::DeclareStateOutputPort),
            py::arg("name"), py::arg("state_index"), py_rvp::reference_internal,
            doc.LeafSystem.DeclareStateOutputPort.doc_continuous)
        .def("DeclareStateOutputPort",
            py::overload_cast<std::variant<std::string, UseDefaultName>,
                DiscreteStateIndex>(&LeafSystemPublic::DeclareStateOutputPort),
            py::arg("name"), py::arg("state_index"), py_rvp::reference_internal,
            doc.LeafSystem.DeclareStateOutputPort.doc_discrete)
        .def("DeclareStateOutputPort",
            py::overload_cast<std::variant<std::string, UseDefaultName>,
                AbstractStateIndex>(&LeafSystemPublic::DeclareStateOutputPort),
            py::arg("name"), py::arg("state_index"), py_rvp::reference_internal,
            doc.LeafSystem.DeclareStateOutputPort.doc_abstract)
        .def(
            "DeclareInitializationEvent",
            [](PyLeafSystem* self, const Event<T>& event) {
              self->DeclareInitializationEvent(event);
            },
            py::arg("event"), doc.LeafSystem.DeclareInitializationEvent.doc)
        .def("DeclarePeriodicPublish",
            &LeafSystemPublic::DeclarePeriodicPublish, py::arg("period_sec"),
            py::arg("offset_sec") = 0.,
            doc.LeafSystem.DeclarePeriodicPublish.doc)
        .def("DeclarePeriodicDiscreteUpdate",
            &LeafSystemPublic::DeclarePeriodicDiscreteUpdate,
            py::arg("period_sec"), py::arg("offset_sec") = 0.,
            doc.LeafSystem.DeclarePeriodicDiscreteUpdate.doc)
        .def(
            "DeclarePeriodicEvent",
            [](PyLeafSystem* self, double period_sec, double offset_sec,
                const Event<T>& event) {
              self->DeclarePeriodicEvent(period_sec, offset_sec, event);
            },
            py::arg("period_sec"), py::arg("offset_sec"), py::arg("event"),
            doc.LeafSystem.DeclarePeriodicEvent.doc)
        .def(
            "DeclarePerStepEvent",
            [](PyLeafSystem* self, const Event<T>& event) {
              self->DeclarePerStepEvent(event);
            },
            py::arg("event"), doc.LeafSystem.DeclarePerStepEvent.doc)
        .def("MakeWitnessFunction",
            WrapCallbacks([](PyLeafSystem* self, const std::string& description,
                              const WitnessFunctionDirection& direction_type,
                              std::function<T(const Context<T>&)> calc)
                              -> std::unique_ptr<WitnessFunction<T>> {
              return self->MakeWitnessFunction(
                  description, direction_type, calc);
            }),
            py_rvp::reference_internal, py::arg("description"),
            py::arg("direction_type"), py::arg("calc"),
            doc.LeafSystem.MakeWitnessFunction.doc_3args)
        .def("MakeWitnessFunction",
            WrapCallbacks(
                [](PyLeafSystem* self, const std::string& description,
                    const WitnessFunctionDirection& direction_type,
                    std::function<T(const Context<T>&)> calc,
                    const Event<T>& e) -> std::unique_ptr<WitnessFunction<T>> {
                  return self->MakeWitnessFunction(
                      description, direction_type, calc, e);
                }),
            py_rvp::reference_internal, py::arg("description"),
            py::arg("direction_type"), py::arg("calc"), py::arg("e"),
            doc.LeafSystem.MakeWitnessFunction.doc_4args)
        .def("DoPublish", &LeafSystemPublic::DoPublish,
            doc.LeafSystem.DoPublish.doc)
        // Continuous state.
        .def("DeclareContinuousState",
            py::overload_cast<int>(&LeafSystemPublic::DeclareContinuousState),
            py::arg("num_state_variables"),
            doc.LeafSystem.DeclareContinuousState.doc_1args_num_state_variables)
        .def("DeclareContinuousState",
            py::overload_cast<int, int, int>(
                &LeafSystemPublic::DeclareContinuousState),
            py::arg("num_q"), py::arg("num_v"), py::arg("num_z"),
            doc.LeafSystem.DeclareContinuousState.doc_3args_num_q_num_v_num_z)
        .def("DeclareContinuousState",
            py::overload_cast<const BasicVector<T>&>(
                &LeafSystemPublic::DeclareContinuousState),
            py::arg("model_vector"),
            doc.LeafSystem.DeclareContinuousState.doc_1args_model_vector)
        // TODO(eric.cousineau): Ideally the downstream class of
        // `BasicVector<T>` should expose `num_q`, `num_v`, and `num_z`?
        .def("DeclareContinuousState",
            py::overload_cast<const BasicVector<T>&, int, int, int>(
                &LeafSystemPublic::DeclareContinuousState),
            py::arg("model_vector"), py::arg("num_q"), py::arg("num_v"),
            py::arg("num_z"),
            doc.LeafSystem.DeclareContinuousState
                .doc_4args_model_vector_num_q_num_v_num_z)
        // Discrete state.
        .def("DeclareDiscreteState",
            py::overload_cast<const BasicVector<T>&>(
                &LeafSystemPublic::DeclareDiscreteState),
            py::arg("model_vector"),
            doc.LeafSystem.DeclareDiscreteState.doc_1args_model_vector)
        .def("DeclareDiscreteState",
            py::overload_cast<const Eigen::Ref<const VectorX<T>>&>(
                &LeafSystemPublic::DeclareDiscreteState),
            py::arg("vector"),
            doc.LeafSystem.DeclareDiscreteState.doc_1args_vector)
        .def("DeclareDiscreteState",
            py::overload_cast<int>(&LeafSystemPublic::DeclareDiscreteState),
            py::arg("num_state_variables"),
            doc.LeafSystem.DeclareDiscreteState.doc_1args_num_state_variables)
        .def("DoCalcTimeDerivatives", &LeafSystemPublic::DoCalcTimeDerivatives)
        .def("DoCalcDiscreteVariableUpdates",
            &LeafSystemPublic::DoCalcDiscreteVariableUpdates,
            doc.LeafSystem.DoCalcDiscreteVariableUpdates.doc)
        // Abstract state.
        .def("DeclareAbstractState",
            py::overload_cast<const AbstractValue&>(
                &LeafSystemPublic::DeclareAbstractState),
            doc.LeafSystem.DeclareAbstractState.doc);

    DefineTemplateClassWithDefault<Diagram<T>, PyDiagram, System<T>>(
        m, "Diagram", GetPyParam<T>(), doc.Diagram.doc)
        .def(py::init<>(), doc.Diagram.ctor.doc_0args)
        .def("GetMutableSubsystemState",
            overload_cast_explicit<State<T>&, const System<T>&, Context<T>*>(
                &Diagram<T>::GetMutableSubsystemState),
            py_rvp::reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 3>(),
            doc.Diagram.GetMutableSubsystemState.doc_2args_subsystem_context)
        .def("GetSubsystemByName", &Diagram<T>::GetSubsystemByName,
            py::arg("name"), py_rvp::reference_internal,
            doc.Diagram.GetSubsystemByName.doc)
        .def(
            "GetSystems",
            [](Diagram<T>* self) {
              py::list out;
              py::object self_py = py::cast(self, py_rvp::reference);
              for (auto* system : self->GetSystems()) {
                py::object system_py = py::cast(system, py_rvp::reference);
                // Keep alive, ownership: `system` keeps `self` alive.
                py_keep_alive(system_py, self_py);
                out.append(system_py);
              }
              return out;
            },
            doc.Diagram.GetSystems.doc);

    // N.B. This will effectively allow derived classes of `VectorSystem` to
    // override `LeafSystem` methods, disrespecting `final`-ity.
    // This could be changed (see https://stackoverflow.com/a/2425785), but meh,
    // we're already abusing Python and C++ enough.
    DefineTemplateClassWithDefault<VectorSystem<T>, PyVectorSystem,
        LeafSystem<T>>(m, "VectorSystem", GetPyParam<T>(), doc.VectorSystem.doc)
        .def(py::init([](int input_size, int output_size,
                          std::optional<bool> direct_feedthrough) {
          return new PyVectorSystem(
              input_size, output_size, direct_feedthrough);
        }),
            py::arg("input_size"), py::arg("output_size"),
            py::arg("direct_feedthrough") = std::nullopt,
            doc.VectorSystem.ctor.doc_3args);
    // TODO(eric.cousineau): Bind virtual methods once we provide a function
    // wrapper to convert `Map<Derived>*` arguments.
    // N.B. This could be mitigated by using `EigenPtr` in public interfaces in
    // upstream code.
  }
};

template <typename... Packs>
py::tuple GetPyParamList(type_pack<Packs...> = {}) {
  return py::make_tuple(GetPyParam(Packs{})...);
}

void DoScalarIndependentDefinitions(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  constexpr auto& doc = pydrake_doc.drake.systems;

  {
    using Class = SystemBase;
    constexpr auto& cls_doc = doc.SystemBase;
    // TODO(eric.cousineau): Bind remaining methods.
    py::class_<Class>(m, "SystemBase", cls_doc.doc)
        .def("GetSystemName", &Class::GetSystemName, cls_doc.GetSystemName.doc)
        .def("GetSystemPathname", &Class::GetSystemPathname,
            cls_doc.GetSystemPathname.doc)
        .def("GetSystemType", &Class::GetSystemType, cls_doc.GetSystemType.doc)
        .def("get_name", &Class::get_name, cls_doc.get_name.doc)
        .def(
            "set_name", &Class::set_name, py::arg("name"), cls_doc.set_name.doc)
        // Topology.
        .def("num_input_ports", &Class::num_input_ports,
            cls_doc.num_input_ports.doc)
        .def("num_output_ports", &Class::num_output_ports,
            cls_doc.num_output_ports.doc)
        // Parameters.
        .def("num_abstract_parameters", &Class::num_abstract_parameters,
            cls_doc.num_abstract_parameters.doc)
        .def("num_numeric_parameter_groups",
            &Class::num_numeric_parameter_groups,
            cls_doc.num_numeric_parameter_groups.doc)
        // Dependency tickets that do not have an index argument.
        .def_static("accuracy_ticket", &Class::accuracy_ticket,
            cls_doc.accuracy_ticket.doc)
        .def_static("all_input_ports_ticket", &Class::all_input_ports_ticket,
            cls_doc.all_input_ports_ticket.doc)
        .def_static("all_parameters_ticket", &Class::all_parameters_ticket,
            cls_doc.all_parameters_ticket.doc)
        .def_static("all_sources_except_input_ports_ticket",
            &Class::all_sources_except_input_ports_ticket,
            cls_doc.all_sources_except_input_ports_ticket.doc)
        .def_static("all_sources_ticket", &Class::all_sources_ticket,
            cls_doc.all_sources_ticket.doc)
        .def_static("all_state_ticket", &Class::all_state_ticket,
            cls_doc.all_state_ticket.doc)
        .def_static("configuration_ticket", &Class::configuration_ticket,
            cls_doc.configuration_ticket.doc)
        .def_static("ke_ticket", &Class::ke_ticket, cls_doc.ke_ticket.doc)
        .def_static("kinematics_ticket", &Class::kinematics_ticket,
            cls_doc.kinematics_ticket.doc)
        .def_static("nothing_ticket", &Class::nothing_ticket,
            cls_doc.nothing_ticket.doc)
        .def_static("pa_ticket", &Class::pa_ticket, cls_doc.pa_ticket.doc)
        .def_static("pc_ticket", &Class::pc_ticket, cls_doc.pc_ticket.doc)
        .def_static("pe_ticket", &Class::pe_ticket, cls_doc.pe_ticket.doc)
        .def_static("pn_ticket", &Class::pn_ticket, cls_doc.pn_ticket.doc)
        .def_static("pnc_ticket", &Class::pnc_ticket, cls_doc.pnc_ticket.doc)
        .def_static("q_ticket", &Class::q_ticket, cls_doc.q_ticket.doc)
        .def_static("time_ticket", &Class::time_ticket, cls_doc.time_ticket.doc)
        .def_static("v_ticket", &Class::v_ticket, cls_doc.v_ticket.doc)
        .def_static("xa_ticket", &Class::xa_ticket, cls_doc.xa_ticket.doc)
        .def_static("xc_ticket", &Class::xc_ticket, cls_doc.xc_ticket.doc)
        .def_static(
            "xcdot_ticket", &Class::xcdot_ticket, cls_doc.xcdot_ticket.doc)
        .def_static("xd_ticket", &Class::xd_ticket, cls_doc.xd_ticket.doc)
        .def_static("z_ticket", &Class::z_ticket, cls_doc.z_ticket.doc)
        // Dependency tickets that do have an index argument.
        // (We do not bind output_port_ticket because it's marked "internal".)
        .def("abstract_parameter_ticket", &Class::abstract_parameter_ticket,
            py::arg("index"), cls_doc.abstract_parameter_ticket.doc)
        .def("abstract_state_ticket", &Class::abstract_state_ticket,
            py::arg("index"), cls_doc.abstract_state_ticket.doc)
        .def("cache_entry_ticket", &Class::cache_entry_ticket, py::arg("index"),
            cls_doc.cache_entry_ticket.doc)
        .def("discrete_state_ticket", &Class::discrete_state_ticket,
            py::arg("index"), cls_doc.discrete_state_ticket.doc)
        .def("input_port_ticket", &Class::input_port_ticket, py::arg("index"),
            cls_doc.input_port_ticket.doc)
        .def("numeric_parameter_ticket", &Class::numeric_parameter_ticket,
            py::arg("index"), cls_doc.numeric_parameter_ticket.doc)
        .def("get_cache_entry", &Class::get_cache_entry, py::arg("index"),
            py_rvp::reference_internal, cls_doc.get_cache_entry.doc)
        // N.B. Since this method has template overloads, we must specify the
        // types `overload_cast_explicit`; we must also specify Class.
        // We do not use `static_cast<>` to avoid accidental type mixing.
        .def("DeclareCacheEntry",
            overload_cast_explicit<CacheEntry&, std::string, ValueProducer,
                std::set<DependencyTicket>>.operator()<Class>(
                &SystemBasePublic::DeclareCacheEntry),
            py_rvp::reference_internal, py::arg("description"),
            py::arg("value_producer"),
            py::arg("prerequisites_of_calc") =
                std::set<DependencyTicket>{Class::all_sources_ticket()},
            doc.SystemBase.DeclareCacheEntry
                .doc_3args_description_value_producer_prerequisites_of_calc);
  }

  {
    // System scalar conversion.
    py::class_<SystemScalarConverter> converter(m, "SystemScalarConverter");
    converter  // BR
        .def(py::init())
        .def("__copy__",
            [](const SystemScalarConverter& in) -> SystemScalarConverter {
              return in;
            });
    // Bind templated instantiations.
    auto converter_methods = [converter](auto pack) {
      constexpr auto& cls_doc = pydrake_doc.drake.systems.SystemScalarConverter;
      using Pack = decltype(pack);
      using T = typename Pack::template type_at<0>;
      using U = typename Pack::template type_at<1>;
      AddTemplateMethod(converter, "IsConvertible",
          &SystemScalarConverter::IsConvertible<T, U>, GetPyParam<T, U>(),
          cls_doc.IsConvertible.doc);
      using system_scalar_converter_internal::AddPydrakeConverterFunction;
      using ConverterFunction =
          std::function<std::unique_ptr<System<T>>(const System<U>&)>;
      AddTemplateMethod(converter, "_Add",
          WrapCallbacks(
              [](SystemScalarConverter* self, const ConverterFunction& func) {
                const std::function<System<T>*(const System<U>&)> bare_func =
                    [func](const System<U>& other) {
                      return func(other).release();
                    };
                AddPydrakeConverterFunction(self, bare_func);
              }),
          GetPyParam<T, U>());
    };
    // N.B. When changing the pairs of supported types below, ensure that these
    // reflect the stanzas for the advanced constructor of
    // `SystemScalarConverter`.
    using ConversionPairs = type_pack<      // BR
        type_pack<AutoDiffXd, double>,      //
        type_pack<Expression, double>,      //
        type_pack<double, AutoDiffXd>,      //
        type_pack<Expression, AutoDiffXd>,  //
        type_pack<double, Expression>,      //
        type_pack<AutoDiffXd, Expression>>;
    type_visit(converter_methods, ConversionPairs{});
    // Add mention of what scalars are supported via `SystemScalarConverter`
    // through Python.
    converter.attr("SupportedScalars") = GetPyParam(CommonScalarPack{});
    converter.attr("SupportedConversionPairs") =
        GetPyParamList(ConversionPairs{});
  }

  m.def("GenerateHtml", &GenerateHtml, py::arg("system"),
      py::arg("initial_depth") = 1, doc.GenerateHtml.doc);
}

}  // namespace

void DefineFrameworkPySystems(py::module m) {
  DoScalarIndependentDefinitions(m);

  // Do templated instantiations of system types.
  auto bind_common_scalar_types = [m](auto dummy) {
    using T = decltype(dummy);
    Impl<T>::DoScalarDependentDefinitions(m);
  };
  type_visit(bind_common_scalar_types, CommonScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
