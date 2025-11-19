#include "drake/bindings/pydrake/systems/framework_py_systems.h"

#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "pybind11/eval.h"

#include "drake/bindings/generated_docstrings/systems_framework.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/common/ref_cycle_pybind.h"
#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/value_producer_pybind.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_scalar_converter.h"
#include "drake/systems/framework/system_visitor.h"
#include "drake/systems/framework/vector_system.h"
#include "drake/systems/framework/witness_function.h"
#include "drake/systems/framework/wrapped_system.h"

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
using systems::EventStatus;
using systems::LeafSystem;
using systems::PublishEvent;
using systems::State;
using systems::System;
using systems::SystemBase;
using systems::SystemScalarConverter;
using systems::SystemVisitor;
using systems::UnrestrictedUpdateEvent;
using systems::VectorSystem;
using systems::WitnessFunction;
using systems::internal::WrappedSystem;

// NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
using namespace drake::systems;
constexpr auto& doc = pydrake_doc_systems_framework.drake.systems;

// TODO(jwnimmer-tri) Reformat this entire file to remove the unnecessary
// indentation.

class SystemBasePublic : public SystemBase {
 public:
  // This class is only used to expose some protected types.
  // It is never instantiated.
  SystemBasePublic() = delete;

  using SystemBase::DeclareCacheEntry;
  using SystemBase::DoGetGraphvizFragment;
  using SystemBase::GraphvizFragmentParams;
};

// Provides a templated 'namespace'.
template <typename T>
struct Impl {
  class PySystem : public System<T> {
   public:
    using Base = System<T>;
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

    // Expose protected methods for binding, no need for virtual overrides.
    using Base::DeclareAbstractInputPort;
    using Base::DeclareAbstractOutputPort;
    using Base::DeclareAbstractParameter;
    using Base::DeclareAbstractState;
    using Base::DeclareContinuousState;
    using Base::DeclareDiscreteState;
    using Base::DeclareInitializationEvent;
    using Base::DeclareNumericParameter;
    using Base::DeclarePeriodicEvent;
    using Base::DeclarePeriodicUnrestrictedUpdateEvent;
    using Base::DeclarePerStepEvent;
    using Base::DeclareStateOutputPort;
    using Base::DeclareVectorInputPort;
    using Base::DeclareVectorOutputPort;
    using Base::get_mutable_forced_discrete_update_events;
    using Base::get_mutable_forced_publish_events;
    using Base::get_mutable_forced_unrestricted_update_events;
    using Base::HandlePostConstructionScalarConversion;
    using Base::MakeWitnessFunction;

    // Because `LeafSystem<T>::DoCalcTimeDerivatives` is protected, and we had
    // to override this method in `PyLeafSystem`, expose the method here for
    // direct(-ish) access. (Otherwise, we get an error about inaccessible
    // downcasting when trying to bind `PyLeafSystem::DoCalcTimeDerivatives` to
    // `py::class_<LeafSystem<T>, ...>`.
    using Base::DoCalcTimeDerivatives;
  };

  // Provide flexible inheritance to leverage prior binding information, per
  // documentation:
  // http://pybind11.readthedocs.io/en/stable/advanced/classes.html#combining-virtual-functions-and-inheritance
  template <typename LeafSystemBase = LeafSystemPublic>
  class PyLeafSystemBase : public LeafSystemBase {
   public:
    using Base = LeafSystemBase;
    using Base::Base;

    // Trampoline virtual methods.

    void DoCalcTimeDerivatives(const Context<T>& context,
        ContinuousState<T>* derivatives) const override {
      // Yuck! We have to dig in and use internals :(
      // We must ensure that pybind only sees pointers, since this method may
      // be called from C++, and pybind will not have seen these objects yet.
      // @see https://github.com/pybind/pybind11/issues/1241
      // TODO(eric.cousineau): Figure out how to supply different behavior,
      // possibly using function wrapping.
      PYBIND11_OVERLOAD_INT(
          void, LeafSystem<T>, "DoCalcTimeDerivatives", &context, derivatives);
      // If the macro did not return, use default functionality.
      Base::DoCalcTimeDerivatives(context, derivatives);
    }

    // This actually changes the signature of DoGetWitnessFunction,
    // expecting the python overload to return a list of witnesses (instead
    // of taking in an empty pointer to std::vector<>.
    // TODO(russt): This is actually a System method, so make a PySystem
    // trampoline if this is needed outside of LeafSystem.
    void DoGetWitnessFunctions(const Context<T>& context,
        std::vector<const WitnessFunction<T>*>* witnesses) const override {
      py::gil_scoped_acquire guard;
      auto wrapped =
          [&]() -> std::optional<std::vector<const WitnessFunction<T>*>> {
        PYBIND11_OVERLOAD_INT(
            std::optional<std::vector<const WitnessFunction<T>*>>,
            LeafSystem<T>, "DoGetWitnessFunctions", &context);
        std::vector<const WitnessFunction<T>*> result;
        // If the macro did not return, use default functionality.
        Base::DoGetWitnessFunctions(context, &result);
        return {result};
      };
      auto result = wrapped();
      if (!result.has_value()) {
        // Give a good error message in case the user forgot to return anything.
        throw py::type_error(
            "Overrides of DoGetWitnessFunctions() must return "
            "List[WitnessFunction], not NoneType.");
      }
      *witnesses = std::move(*result);
    }

    SystemBase::GraphvizFragment DoGetGraphvizFragment(
        const SystemBase::GraphvizFragmentParams& params) const override {
      PYBIND11_OVERRIDE(SystemBase::GraphvizFragment, LeafSystem<T>,
          DoGetGraphvizFragment, params);
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
  class PyDiagramBase : public DiagramBase {
   public:
    using Base = DiagramBase;
    using Base::Base;

    SystemBase::GraphvizFragment DoGetGraphvizFragment(
        const SystemBase::GraphvizFragmentParams& params) const override {
      PYBIND11_OVERRIDE(SystemBase::GraphvizFragment, Diagram<T>,
          DoGetGraphvizFragment, params);
    }
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

  class PyVectorSystem : public VectorSystemPublic {
   public:
    using Base = VectorSystemPublic;
    using Base::Base;

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

  class PySystemVisitor : public SystemVisitor<T> {
   public:
    // Trampoline virtual methods.
    void VisitSystem(const System<T>& system) override {
      PYBIND11_OVERLOAD_PURE(void, SystemVisitor<T>, VisitSystem, system);
    };

    void VisitDiagram(const Diagram<T>& diagram) override {
      PYBIND11_OVERLOAD_PURE(void, SystemVisitor<T>, VisitDiagram, diagram);
    }
  };

  // Because Python doesn't offer static type checking to help remind
  // the user to return an EventStatus from an event handler function,
  // we'll bind the callback as optional<> to allow the user to omit a
  // return statement. (When declaring a periodic event in C++, the
  // user-provided callback function is similarly overloaded to return
  // either EventStatus or void. We can't overload based on return
  // values in pybind11, so that's another reason we'll use optional<>
  // in Python for the same effect.)
  template <typename... Args>
  using EventCallback = std::function<std::optional<EventStatus>(Args...)>;

  static py::class_<System<T>, SystemBase, PySystem> DefineSystem(
      py::module m) {
    // TODO(eric.cousineau): Show constructor, but somehow make sure `pybind11`
    // knows this is abstract?
    auto system_cls =
        DefineTemplateClassWithDefault<System<T>, SystemBase, PySystem>(m,
            "System", GetPyParam<T>(), doc.System.doc, std::nullopt,
            py::dynamic_attr());
    system_cls
        // Resource allocation and initialization.
        .def("AllocateContext", &System<T>::AllocateContext,
            doc.System.AllocateContext.doc)
        .def("AllocateInputVector", &System<T>::AllocateInputVector,
            py::arg("input_port"), doc.System.AllocateInputVector.doc)
        .def("AllocateInputAbstract", &System<T>::AllocateInputAbstract,
            py::arg("input_port"), doc.System.AllocateInputAbstract.doc)
        .def("AllocateOutput",
            overload_cast_explicit<unique_ptr<SystemOutput<T>>>(
                &System<T>::AllocateOutput),
            doc.System.AllocateOutput.doc)
        .def("AllocateTimeDerivatives",
            overload_cast_explicit<unique_ptr<ContinuousState<T>>>(
                &System<T>::AllocateTimeDerivatives),
            doc.System.AllocateTimeDerivatives.doc)
        .def("AllocateImplicitTimeDerivativesResidual",
            &System<T>::AllocateImplicitTimeDerivativesResidual,
            doc.System.AllocateImplicitTimeDerivativesResidual.doc)
        .def("AllocateDiscreteVariables",
            overload_cast_explicit<unique_ptr<DiscreteValues<T>>>(
                &System<T>::AllocateDiscreteVariables),
            doc.System.AllocateDiscreteVariables.doc)
        .def("CreateDefaultContext", &System<T>::CreateDefaultContext,
            doc.System.CreateDefaultContext.doc)
        .def("SetDefaultContext", &System<T>::SetDefaultContext,
            py::arg("context"), doc.System.SetDefaultContext.doc)
        .def("SetRandomContext", &System<T>::SetRandomContext,
            py::arg("context"), py::arg("generator"),
            doc.System.SetRandomContext.doc)
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
        // Publishing.
        .def("ForcedPublish", &System<T>::ForcedPublish, py::arg("context"),
            doc.System.ForcedPublish.doc)
        // Cached evaluations.
        .def("EvalTimeDerivatives", &System<T>::EvalTimeDerivatives,
            py_rvp::reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 2>(), py::arg("context"),
            doc.System.EvalTimeDerivatives.doc)
        .def("EvalPotentialEnergy", &System<T>::EvalPotentialEnergy,
            py::arg("context"), doc.System.EvalPotentialEnergy.doc)
        .def("EvalKineticEnergy", &System<T>::EvalKineticEnergy,
            py::arg("context"), doc.System.EvalKineticEnergy.doc)
        .def(
            "EvalVectorInput",
            [](const System<T>* self, const Context<T>& arg1, int arg2) {
              return self->EvalVectorInput(arg1, arg2);
            },
            py::arg("context"), py::arg("port_index"), py_rvp::reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 2>(), doc.System.EvalVectorInput.doc)
        // Constraints.
        .def("num_constraints", &System<T>::num_constraints,
            doc.System.num_constraints.doc)
        // (For now, bind privately since ExternalSystemConstraint is private.)
        .def("_AddExternalConstraint", &System<T>::AddExternalConstraint,
            py::arg("constraint"), doc.System.AddExternalConstraint.doc)
        // Calculations.
        .def("CalcTimeDerivatives", &System<T>::CalcTimeDerivatives,
            py::arg("context"), py::arg("derivatives"),
            doc.System.CalcTimeDerivatives.doc)
        .def("CalcImplicitTimeDerivativesResidual",
            &System<T>::CalcImplicitTimeDerivativesResidual, py::arg("context"),
            py::arg("proposed_derivatives"), py::arg("residual"),
            doc.System.CalcImplicitTimeDerivativesResidual.doc)
        .def(
            "CalcImplicitTimeDerivativesResidual",
            [](const System<T>* self, const Context<T>& context,
                const ContinuousState<T>& proposed_derivatives) {
              // Note: This is the only version of the method that works for
              // dtype object.
              VectorX<T> residual =
                  self->AllocateImplicitTimeDerivativesResidual();
              self->CalcImplicitTimeDerivativesResidual(
                  context, proposed_derivatives, &residual);
              return residual;
            },
            py::arg("context"), py::arg("proposed_derivatives"),
            doc.System.CalcImplicitTimeDerivativesResidual.doc)
        .def("CalcForcedDiscreteVariableUpdate",
            &System<T>::CalcForcedDiscreteVariableUpdate, py::arg("context"),
            py::arg("discrete_state"),
            doc.System.CalcForcedDiscreteVariableUpdate.doc)
        .def("CalcForcedUnrestrictedUpdate",
            &System<T>::CalcForcedUnrestrictedUpdate, py::arg("context"),
            py::arg("state"), doc.System.CalcForcedUnrestrictedUpdate.doc)
        .def("ExecuteInitializationEvents",
            &System<T>::ExecuteInitializationEvents, py::arg("context"),
            doc.System.ExecuteInitializationEvents.doc)
        .def("ExecuteForcedEvents", &System<T>::ExecuteForcedEvents,
            py::arg("context"), py::arg("publish") = true,
            doc.System.ExecuteForcedEvents.doc)
        .def("GetUniquePeriodicDiscreteUpdateAttribute",
            &System<T>::GetUniquePeriodicDiscreteUpdateAttribute,
            doc.System.GetUniquePeriodicDiscreteUpdateAttribute.doc)
        .def("EvalUniquePeriodicDiscreteUpdate",
            &System<T>::EvalUniquePeriodicDiscreteUpdate, py_rvp::reference,
            // Keep alive, ownership: `return` keeps `context` alive.
            py::keep_alive<0, 2>(), py::arg("context"),
            doc.System.EvalUniquePeriodicDiscreteUpdate.doc)
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
        .def("IsDifferentialEquationSystem",
            &System<T>::IsDifferentialEquationSystem,
            doc.System.IsDifferentialEquationSystem.doc)
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
        // Subcontext access.
        .def("GetSubsystemContext",
            overload_cast_explicit<const Context<T>&, const System<T>&,
                const Context<T>&>(&System<T>::GetSubsystemContext),
            py::arg("subsystem"), py::arg("context"), py_rvp::reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 3>(), doc.System.GetMutableSubsystemContext.doc)
        .def("GetMutableSubsystemContext",
            overload_cast_explicit<Context<T>&, const System<T>&, Context<T>*>(
                &System<T>::GetMutableSubsystemContext),
            py::arg("subsystem"), py::arg("context"), py_rvp::reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 3>(), doc.System.GetMutableSubsystemContext.doc)
        .def("GetMyContextFromRoot",
            overload_cast_explicit<const Context<T>&, const Context<T>&>(
                &System<T>::GetMyContextFromRoot),
            py::arg("root_context"), py_rvp::reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 2>(), doc.System.GetMyMutableContextFromRoot.doc)
        .def("GetMyMutableContextFromRoot",
            overload_cast_explicit<Context<T>&, Context<T>*>(
                &System<T>::GetMyMutableContextFromRoot),
            py::arg("root_context"), py_rvp::reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 2>(), doc.System.GetMyMutableContextFromRoot.doc)
        // Port access methods. All returned port references use a ref_cycle
        // (rather than the implicit keep-alive of reference_internal) to avoid
        // immortality hazards like #22515.
        .def("get_input_port",
            overload_cast_explicit<const InputPort<T>&, int, bool>(
                &System<T>::get_input_port),
            internal::ref_cycle<0, 1>(), py_rvp::reference,
            py::arg("port_index"), py::arg("warn_deprecated") = true,
            doc.System.get_input_port.doc_2args)
        .def("get_input_port",
            overload_cast_explicit<const InputPort<T>&>(
                &System<T>::get_input_port),
            internal::ref_cycle<0, 1>(), py_rvp::reference,
            doc.System.get_input_port.doc_0args)
        .def("GetInputPort", &System<T>::GetInputPort,
            internal::ref_cycle<0, 1>(), py_rvp::reference,
            py::arg("port_name"), doc.System.GetInputPort.doc)
        .def("HasInputPort", &System<T>::HasInputPort, py::arg("port_name"),
            doc.System.HasInputPort.doc)
        .def("get_output_port",
            overload_cast_explicit<const OutputPort<T>&, int, bool>(
                &System<T>::get_output_port),
            internal::ref_cycle<0, 1>(), py_rvp::reference,
            py::arg("port_index"), py::arg("warn_deprecated") = true,
            doc.System.get_output_port.doc_2args)
        .def("get_output_port",
            overload_cast_explicit<const OutputPort<T>&>(
                &System<T>::get_output_port),
            internal::ref_cycle<0, 1>(), py_rvp::reference,
            doc.System.get_output_port.doc_0args)
        .def("GetOutputPort", &System<T>::GetOutputPort,
            internal::ref_cycle<0, 1>(), py_rvp::reference,
            py::arg("port_name"), doc.System.GetOutputPort.doc)
        .def("HasOutputPort", &System<T>::HasOutputPort, py::arg("port_name"),
            doc.System.HasOutputPort.doc)
        // Witness functions.
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
                .c_str())
        // Protected System construction.
        .def("DeclareInputPort",
            overload_cast_explicit<InputPort<T>&,
                std::variant<std::string, UseDefaultName>, PortDataType, int,
                std::optional<RandomDistribution>>(&PySystem::DeclareInputPort),
            // Use a ref_cycle (rather than the implicit keep-alive of
            // reference_internal) to avoid immortality hazards like #22515.
            internal::ref_cycle<0, 1>(), py_rvp::reference, py::arg("name"),
            py::arg("type"), py::arg("size"),
            py::arg("random_type") = std::nullopt,
            doc.System.DeclareInputPort.doc)
        // Not part of System; SystemBase method promoted in bindings.
        .def(
            "EvalAbstractInput",
            [](const System<T>* self, const Context<T>& arg1, int arg2) {
              return self->EvalAbstractInput(arg1, arg2);
            },
            py::arg("context"), py::arg("port_index"), py_rvp::reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 2>(), doc.SystemBase.EvalAbstractInput.doc)
        // TODO(jwnimmer-tri) Use DefClone here, once it has support for
        // docstrings and overload resolution.
        .def(
            "Clone", [](const System<T>* self) { return self->Clone(); },
            doc.System.Clone.doc_0args)
        .def("__copy__", [](const System<T>* self) { return self->Clone(); })
        .def("__deepcopy__", [](const System<T>* self, py::dict /* memo */) {
          return self->Clone();
        });
    return system_cls;
  }

  template <typename PyClass>
  static void DefineSystemScalarConversions(PyClass* system_cls) {
    PyClass& cls = *system_cls;
    cls  // BR
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
        .def("get_system_scalar_converter",
            &System<T>::get_system_scalar_converter, py_rvp::reference_internal,
            doc.System.get_system_scalar_converter.doc);
    auto def_to_scalar_type = [&cls](auto dummy) {
      using U = decltype(dummy);
      AddTemplateMethod(
          cls, "ToScalarType",
          [](const System<T>& self) { return self.template ToScalarType<U>(); },
          GetPyParam<U>(), doc.System.ToScalarType.doc_0args);
      AddTemplateMethod(
          cls, "_HandlePostConstructionScalarConversion",
          [](System<T>& self, const System<U>& from) {
            LeafSystemPublic::HandlePostConstructionScalarConversion(
                from, &self);
          },
          GetPyParam<U>(),
          doc.System.HandlePostConstructionScalarConversion.doc);
    };
    type_visit(def_to_scalar_type, CommonScalarPack{});

    auto def_to_scalar_type_maybe = [&cls](auto dummy) {
      using U = decltype(dummy);
      AddTemplateMethod(cls, "ToScalarTypeMaybe",
          &System<T>::template ToScalarTypeMaybe<U>, GetPyParam<U>(),
          doc.System.ToScalarTypeMaybe.doc);
    };
    type_visit(def_to_scalar_type_maybe, CommonScalarPack{});
  }

  static void DefineLeafSystem(py::module m) {
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
            // Use a ref_cycle (rather than the implicit keep-alive of
            // reference_internal) to avoid immortality hazards like #22515.
            internal::ref_cycle<0, 1>(), py_rvp::reference, py::arg("name"),
            py::arg("model_value"), doc.LeafSystem.DeclareAbstractInputPort.doc)
        .def("DeclareAbstractParameter",
            &PyLeafSystem::DeclareAbstractParameter, py::arg("model_value"),
            doc.LeafSystem.DeclareAbstractParameter.doc)
        .def("DeclareNumericParameter", &PyLeafSystem::DeclareNumericParameter,
            py::arg("model_vector"), doc.LeafSystem.DeclareNumericParameter.doc)
        .def(
            "DeclareAbstractOutputPort",
            [](PyLeafSystem* self, const std::string& name, py::function alloc,
                std::function<void(py::object, py::object)> calc,
                const std::set<DependencyTicket>& prerequisites_of_calc)
                -> const OutputPort<T>& {
              return self->DeclareAbstractOutputPort(name,
                  MakeCppCompatibleAllocateCallback(std::move(alloc)),
                  MakeCppCompatibleCalcCallback(std::move(calc)),
                  prerequisites_of_calc);
            },
            // Use a ref_cycle (rather than the implicit keep-alive of
            // reference_internal) to avoid immortality hazards like #22515.
            internal::ref_cycle<0, 1>(), py_rvp::reference, py::arg("name"),
            py::arg("alloc"), py::arg("calc"),
            py::arg("prerequisites_of_calc") =
                std::set<DependencyTicket>{SystemBase::all_sources_ticket()},
            doc.LeafSystem.DeclareAbstractOutputPort
                .doc_4args_name_alloc_calc_prerequisites_of_calc)
        .def(
            "DeclareVectorInputPort",
            [](PyLeafSystem* self, std::string name,
                const BasicVector<T>& model_vector,
                std::optional<RandomDistribution> random_type)
                -> InputPort<T>& {
              return self->DeclareVectorInputPort(
                  name, model_vector, random_type);
            },
            // Use a ref_cycle (rather than the implicit keep-alive of
            // reference_internal) to avoid immortality hazards like #22515.
            internal::ref_cycle<0, 1>(), py_rvp::reference, py::arg("name"),
            py::arg("model_vector"), py::arg("random_type") = std::nullopt,
            doc.LeafSystem.DeclareVectorInputPort.doc_3args_model_vector)
        .def(
            "DeclareVectorInputPort",
            [](PyLeafSystem* self, std::string name, int size,
                std::optional<RandomDistribution> random_type)
                -> InputPort<T>& {
              return self->DeclareVectorInputPort(name, size, random_type);
            },
            // Use a ref_cycle (rather than the implicit keep-alive of
            // reference_internal) to avoid immortality hazards like #22515.
            internal::ref_cycle<0, 1>(), py_rvp::reference, py::arg("name"),
            py::arg("size"), py::arg("random_type") = std::nullopt,
            doc.LeafSystem.DeclareVectorInputPort.doc_3args_size)
        .def("DeclareVectorOutputPort",
            WrapCallbacks(
                [](PyLeafSystem* self, const std::string& name,
                    const BasicVector<T>& arg1, CalcVectorCallback arg2,
                    const std::set<DependencyTicket>& arg3)
                    -> const OutputPort<T>& {
                  return self->DeclareVectorOutputPort(name, arg1, arg2, arg3);
                }),
            // Use a ref_cycle (rather than the implicit keep-alive of
            // reference_internal) to avoid immortality hazards like #22515.
            internal::ref_cycle<0, 1>(), py_rvp::reference, py::arg("name"),
            py::arg("model_value"), py::arg("calc"),
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
            // Use a ref_cycle (rather than the implicit keep-alive of
            // reference_internal) to avoid immortality hazards like #22515.
            internal::ref_cycle<0, 1>(), py_rvp::reference, py::arg("name"),
            py::arg("size"), py::arg("calc"),
            py::arg("prerequisites_of_calc") =
                std::set<DependencyTicket>{SystemBase::all_sources_ticket()},
            doc.LeafSystem.DeclareVectorOutputPort.doc_4args_size)
        .def("DeclareStateOutputPort",
            py::overload_cast<std::variant<std::string, UseDefaultName>,
                ContinuousStateIndex>(
                &LeafSystemPublic::DeclareStateOutputPort),
            py::arg("name"), py::arg("state_index"),
            // Use a ref_cycle (rather than the implicit keep-alive of
            // reference_internal) to avoid immortality hazards like #22515.
            internal::ref_cycle<0, 1>(), py_rvp::reference,
            doc.LeafSystem.DeclareStateOutputPort.doc_continuous)
        .def("DeclareStateOutputPort",
            py::overload_cast<std::variant<std::string, UseDefaultName>,
                DiscreteStateIndex>(&LeafSystemPublic::DeclareStateOutputPort),
            py::arg("name"), py::arg("state_index"),
            // Use a ref_cycle (rather than the implicit keep-alive of
            // reference_internal) to avoid immortality hazards like #22515.
            internal::ref_cycle<0, 1>(), py_rvp::reference,
            doc.LeafSystem.DeclareStateOutputPort.doc_discrete)
        .def("DeclareStateOutputPort",
            py::overload_cast<std::variant<std::string, UseDefaultName>,
                AbstractStateIndex>(&LeafSystemPublic::DeclareStateOutputPort),
            py::arg("name"), py::arg("state_index"),
            // Use a ref_cycle (rather than the implicit keep-alive of
            // reference_internal) to avoid immortality hazards like #22515.
            internal::ref_cycle<0, 1>(), py_rvp::reference,
            doc.LeafSystem.DeclareStateOutputPort.doc_abstract)
        // TODO(russt): Implement the std::function variant of
        // LeafSystem::Declare*Event sugar methods if they are ever needed,
        // instead of implementing them here.
        .def("DeclareInitializationPublishEvent",
            WrapCallbacks([](PyLeafSystem* self,
                              EventCallback<const Context<T>&> publish) {
              self->DeclareInitializationEvent(PublishEvent<T>(
                  TriggerType::kInitialization,
                  [publish](const System<T>&, const Context<T>& context,
                      const PublishEvent<T>&) {
                    return publish(context).value_or(EventStatus::Succeeded());
                  }));
            }),
            py::arg("publish"),
            doc.LeafSystem.DeclareInitializationPublishEvent.doc)
        .def("DeclareInitializationDiscreteUpdateEvent",
            WrapCallbacks(
                [](PyLeafSystem* self,
                    EventCallback<const Context<T>&, DiscreteValues<T>*>
                        update) {
                  self->DeclareInitializationEvent(
                      DiscreteUpdateEvent<T>(TriggerType::kInitialization,
                          [update](const System<T>&, const Context<T>& context,
                              const DiscreteUpdateEvent<T>&,
                              DiscreteValues<T>* xd) {
                            return update(context, &*xd)
                                .value_or(EventStatus::Succeeded());
                          }));
                }),
            py::arg("update"),
            doc.LeafSystem.DeclareInitializationDiscreteUpdateEvent.doc)
        .def("DeclareInitializationUnrestrictedUpdateEvent",
            WrapCallbacks(
                [](PyLeafSystem* self,
                    EventCallback<const Context<T>&, State<T>*> update) {
                  self->DeclareInitializationEvent(
                      UnrestrictedUpdateEvent<T>(TriggerType::kInitialization,
                          [update](const System<T>&, const Context<T>& context,
                              const UnrestrictedUpdateEvent<T>&, State<T>* x) {
                            return update(context, &*x)
                                .value_or(EventStatus::Succeeded());
                          }));
                }),
            py::arg("update"),
            doc.LeafSystem.DeclareInitializationUnrestrictedUpdateEvent.doc)
        .def(
            "DeclareInitializationEvent",
            [](PyLeafSystem* self, const Event<T>& event) {
              self->DeclareInitializationEvent(event);
            },
            py::arg("event"), doc.LeafSystem.DeclareInitializationEvent.doc)
        .def("DeclarePeriodicPublishEvent",
            WrapCallbacks(
                [](PyLeafSystem* self, double period_sec, double offset_sec,
                    EventCallback<const Context<T>&> publish) {
                  self->DeclarePeriodicEvent(period_sec, offset_sec,
                      PublishEvent<T>(TriggerType::kPeriodic,
                          [publish](const System<T>&, const Context<T>& context,
                              const PublishEvent<T>&) {
                            return publish(context).value_or(
                                EventStatus::Succeeded());
                          }));
                }),
            py::arg("period_sec"), py::arg("offset_sec"), py::arg("publish"),
            doc.LeafSystem.DeclarePeriodicPublishEvent.doc)
        .def("DeclarePeriodicDiscreteUpdateEvent",
            WrapCallbacks(
                [](PyLeafSystem* self, double period_sec, double offset_sec,
                    EventCallback<const Context<T>&, DiscreteValues<T>*>
                        update) {
                  self->DeclarePeriodicEvent(period_sec, offset_sec,
                      DiscreteUpdateEvent<T>(TriggerType::kPeriodic,
                          [update](const System<T>&, const Context<T>& context,
                              const DiscreteUpdateEvent<T>&,
                              DiscreteValues<T>* xd) {
                            return update(context, &*xd)
                                .value_or(EventStatus::Succeeded());
                          }));
                }),
            py::arg("period_sec"), py::arg("offset_sec"), py::arg("update"),
            doc.LeafSystem.DeclarePeriodicDiscreteUpdateEvent.doc)
        .def("DeclarePeriodicUnrestrictedUpdateEvent",
            WrapCallbacks(
                [](PyLeafSystem* self, double period_sec, double offset_sec,
                    EventCallback<const Context<T>&, State<T>*> update) {
                  self->DeclarePeriodicEvent(period_sec, offset_sec,
                      UnrestrictedUpdateEvent<T>(TriggerType::kPeriodic,
                          [update](const System<T>&, const Context<T>& context,
                              const UnrestrictedUpdateEvent<T>&, State<T>* x) {
                            return update(context, &*x)
                                .value_or(EventStatus::Succeeded());
                          }));
                }),
            py::arg("period_sec"), py::arg("offset_sec"), py::arg("update"),
            doc.LeafSystem.DeclarePeriodicUnrestrictedUpdateEvent.doc)
        .def(
            "DeclarePeriodicEvent",
            [](PyLeafSystem* self, double period_sec, double offset_sec,
                const Event<T>& event) {
              self->DeclarePeriodicEvent(period_sec, offset_sec, event);
            },
            py::arg("period_sec"), py::arg("offset_sec"), py::arg("event"),
            doc.LeafSystem.DeclarePeriodicEvent.doc)
        .def("DeclarePerStepPublishEvent",
            WrapCallbacks([](PyLeafSystem* self,
                              EventCallback<const Context<T>&> publish) {
              self->DeclarePerStepEvent(PublishEvent<T>(TriggerType::kPerStep,
                  [publish](const System<T>&, const Context<T>& context,
                      const PublishEvent<T>&) {
                    return publish(context).value_or(EventStatus::Succeeded());
                  }));
            }),
            py::arg("publish"), doc.LeafSystem.DeclarePerStepPublishEvent.doc)
        .def("DeclarePerStepDiscreteUpdateEvent",
            WrapCallbacks(
                [](PyLeafSystem* self,
                    EventCallback<const Context<T>&, DiscreteValues<T>*>
                        update) {
                  self->DeclarePerStepEvent(
                      DiscreteUpdateEvent<T>(TriggerType::kPerStep,
                          [update](const System<T>&, const Context<T>& context,
                              const DiscreteUpdateEvent<T>&,
                              DiscreteValues<T>* xd) {
                            return update(context, &*xd)
                                .value_or(EventStatus::Succeeded());
                          }));
                }),
            py::arg("update"),
            doc.LeafSystem.DeclarePerStepDiscreteUpdateEvent.doc)
        .def("DeclarePerStepUnrestrictedUpdateEvent",
            WrapCallbacks(
                [](PyLeafSystem* self,
                    EventCallback<const Context<T>&, State<T>*> update) {
                  self->DeclarePerStepEvent(
                      UnrestrictedUpdateEvent<T>(TriggerType::kPerStep,
                          [update](const System<T>&, const Context<T>& context,
                              const UnrestrictedUpdateEvent<T>&, State<T>* x) {
                            return update(context, &*x)
                                .value_or(EventStatus::Succeeded());
                          }));
                }),
            py::arg("update"),
            doc.LeafSystem.DeclarePerStepUnrestrictedUpdateEvent.doc)
        .def(
            "DeclarePerStepEvent",
            [](PyLeafSystem* self, const Event<T>& event) {
              self->DeclarePerStepEvent(event);
            },
            py::arg("event"), doc.LeafSystem.DeclarePerStepEvent.doc)
        .def("DeclareForcedPublishEvent",
            WrapCallbacks([](PyLeafSystem* self,
                              EventCallback<const Context<T>&> publish) {
              self->get_mutable_forced_publish_events().AddEvent(
                  PublishEvent<T>(TriggerType::kForced,
                      [publish](const System<T>&, const Context<T>& context,
                          const PublishEvent<T>&) {
                        return publish(context).value_or(
                            EventStatus::Succeeded());
                      }));
            }),
            py::arg("publish"), doc.LeafSystem.DeclareForcedPublishEvent.doc)
        .def("DeclareForcedDiscreteUpdateEvent",
            WrapCallbacks(
                [](PyLeafSystem* self,
                    EventCallback<const Context<T>&, DiscreteValues<T>*>
                        update) {
                  self->get_mutable_forced_discrete_update_events().AddEvent(
                      DiscreteUpdateEvent<T>(TriggerType::kForced,
                          [update](const System<T>&, const Context<T>& context,
                              const DiscreteUpdateEvent<T>&,
                              DiscreteValues<T>* xd) {
                            return update(context, &*xd)
                                .value_or(EventStatus::Succeeded());
                          }));
                }),
            py::arg("update"),
            doc.LeafSystem.DeclareForcedDiscreteUpdateEvent.doc)
        .def("DeclareForcedUnrestrictedUpdateEvent",
            WrapCallbacks(
                [](PyLeafSystem* self,
                    EventCallback<const Context<T>&, State<T>*> update) {
                  self->get_mutable_forced_unrestricted_update_events()
                      .AddEvent(UnrestrictedUpdateEvent<T>(TriggerType::kForced,
                          [update](const System<T>&, const Context<T>& context,
                              const UnrestrictedUpdateEvent<T>&, State<T>* x) {
                            return update(context, &*x)
                                .value_or(EventStatus::Succeeded());
                          }));
                }),
            py::arg("update"),
            doc.LeafSystem.DeclareForcedUnrestrictedUpdateEvent.doc)
        .def("MakeWitnessFunction",
            WrapCallbacks([](PyLeafSystem* self, const std::string& description,
                              const WitnessFunctionDirection& direction_type,
                              std::function<std::optional<T>(const Context<T>&)>
                                  calc) -> std::unique_ptr<WitnessFunction<T>> {
              return self->MakeWitnessFunction(description, direction_type,
                  [calc](const Context<T>& context) -> T {
                    const std::optional<T> result = calc(context);
                    if (!result.has_value()) {
                      // Give a good error message in case the user forgot to
                      // return anything.
                      throw py::type_error(
                          "The MakeWitnessFunction() calc callback must return "
                          "a floating point value, not NoneType.");
                    }
                    return *result;
                  });
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
        // Abstract state.
        .def("DeclareAbstractState",
            py::overload_cast<const AbstractValue&>(
                &LeafSystemPublic::DeclareAbstractState),
            py::arg("model_value"), doc.LeafSystem.DeclareAbstractState.doc);
  }

  static void DefineDiagram(py::module m) {
    DefineTemplateClassWithDefault<Diagram<T>, PyDiagram, System<T>>(
        m, "Diagram", GetPyParam<T>(), doc.Diagram.doc)
        .def(py::init<>(), doc.Diagram.ctor.doc_0args)
        .def(
            "connection_map",
            [](Diagram<T>* self) {
              // N.B. This code is duplicated with DiagramBuilder's same-named
              // function. Keep the two copies in sync. The detailed unit test
              // is written against this copy of this function, not the
              // DiagramBuilder one.
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
                py::object output_port_index_py =
                    py::cast(output_locator.second);

                py::tuple output_locator_py(2);
                output_locator_py[0] = output_system_py;
                output_locator_py[1] = output_port_index_py;

                out[input_locator_py] = output_locator_py;
              }
              return out;
            },
            doc.Diagram.connection_map.doc)
        .def(
            "GetInputPortLocators",
            [](Diagram<T>* self, InputPortIndex port_index) {
              py::list out;
              py::object self_py = py::cast(self, py_rvp::reference);
              for (auto& locator : self->GetInputPortLocators(port_index)) {
                // Keep alive, ownership: `system_py` keeps `self` alive.
                py::object system_py = py::cast(
                    locator.first, py_rvp::reference_internal, self_py);
                py::object port_index_py = py::cast(locator.second);

                py::tuple locator_py(2);
                locator_py[0] = system_py;
                locator_py[1] = port_index_py;
                out.append(locator_py);
              }
              return out;
            },
            py::arg("port_index"), doc.Diagram.GetInputPortLocators.doc)
        .def(
            "get_output_port_locator",
            [](Diagram<T>* self, OutputPortIndex port_index) {
              py::object self_py = py::cast(self, py_rvp::reference);
              const auto& locator = self->get_output_port_locator(port_index);
              // Keep alive, ownership: `system_py` keeps `self` alive.
              py::object system_py =
                  py::cast(locator.first, py_rvp::reference_internal, self_py);
              py::object port_index_py = py::cast(locator.second);

              py::tuple locator_py(2);
              locator_py[0] = system_py;
              locator_py[1] = port_index_py;
              return locator_py;
            },
            py::arg("port_index"), doc.Diagram.get_output_port_locator.doc)
        .def("GetMutableSubsystemState",
            overload_cast_explicit<State<T>&, const System<T>&, Context<T>*>(
                &Diagram<T>::GetMutableSubsystemState),
            py_rvp::reference,
            // Keep alive, ownership: `return` keeps `Context` alive.
            py::keep_alive<0, 3>(),
            doc.Diagram.GetMutableSubsystemState.doc_2args_subsystem_context)
        .def("HasSubsystemNamed", &Diagram<T>::HasSubsystemNamed,
            py::arg("name"), doc.Diagram.HasSubsystemNamed.doc)
        .def("GetSubsystemByName", &Diagram<T>::GetSubsystemByName,
            py::arg("name"), py_rvp::reference_internal,
            doc.Diagram.GetSubsystemByName.doc)
        .def("GetSystems", &Diagram<T>::GetSystems, py_rvp::reference_internal,
            doc.Diagram.GetSystems.doc)
        .def("AreConnected", &Diagram<T>::AreConnected, py::arg("output"),
            py::arg("input"), doc.Diagram.AreConnected.doc);
  }

  static void DefineVectorSystem(py::module m) {
    {
      // N.B. This will effectively allow derived classes of `VectorSystem` to
      // override `LeafSystem` methods, disrespecting `final`-ity.
      // This could be changed (see https://stackoverflow.com/a/2425785), but
      // meh, we're already abusing Python and C++ enough.
      auto cls = DefineTemplateClassWithDefault<VectorSystem<T>, PyVectorSystem,
          LeafSystem<T>>(
          m, "VectorSystem", GetPyParam<T>(), doc.VectorSystem.doc);
      cls  // BR
          .def(py::init([](int input_size, int output_size,
                            std::optional<bool> direct_feedthrough) {
            return new PyVectorSystem(
                input_size, output_size, direct_feedthrough);
          }),
              py::arg("input_size"), py::arg("output_size"),
              py::arg("direct_feedthrough"), doc.VectorSystem.ctor.doc_3args);
    }
  }

  static void DefineWrappedSystem(py::module m) {
    using Class = WrappedSystem<T>;
    auto cls = DefineTemplateClassWithDefault<Class, Diagram<T>>(m,
        "_WrappedSystem", GetPyParam<T>(),
        "Wrapper that enables scalar-conversion of Python leaf systems.");
    cls  // BR
        .def("unwrap", &Class::unwrap, py_rvp::reference_internal,
            "Returns the underlying system.");
  }

  template <typename PyClass>
  static void DefineSystemVisitor(py::module m, PyClass* system_cls) {
    // TODO(eric.cousineau): Bind virtual methods once we provide a function
    // wrapper to convert `Map<Derived>*` arguments.
    // N.B. This could be mitigated by using `EigenPtr` in public interfaces in
    // upstream code.

    DefineTemplateClassWithDefault<SystemVisitor<T>, PySystemVisitor>(
        m, "SystemVisitor", GetPyParam<T>(), doc.SystemVisitor.doc)
        .def(py::init())
        .def("VisitSystem", &SystemVisitor<T>::VisitSystem, py::arg("system"),
            doc.SystemVisitor.VisitSystem.doc)
        .def("VisitDiagram", &SystemVisitor<T>::VisitDiagram,
            py::arg("diagram"), doc.SystemVisitor.VisitDiagram.doc);

    system_cls->def(
        "Accept",
        [](const System<T>* self, SystemVisitor<T>* v) { self->Accept(v); },
        py::arg("v"), doc.System.Accept.doc);
  }
};

template <typename... Packs>
py::tuple GetPyParamList(type_pack<Packs...> = {}) {
  return py::make_tuple(GetPyParam(Packs{})...);
}

void DoScalarIndependentDefinitions(py::module m) {
  {
    using Class = SystemBase;
    constexpr auto& cls_doc = doc.SystemBase;
    // TODO(eric.cousineau): Bind remaining methods.
    py::class_<Class> cls(m, "SystemBase", cls_doc.doc);
    {
      using Nested = SystemBase::GraphvizFragment;
      constexpr auto& nested_doc = doc.SystemBase.GraphvizFragment;
      py::class_<Nested>(cls, "GraphvizFragment", nested_doc.doc)
          .def_readwrite(
              "input_ports", &Nested::input_ports, nested_doc.input_ports.doc)
          .def_readwrite("output_ports", &Nested::output_ports,
              nested_doc.output_ports.doc)
          .def_readwrite(
              "fragments", &Nested::fragments, nested_doc.fragments.doc);
    }
    {
      // GraphvizFragmentParams
      using Nested = SystemBasePublic::GraphvizFragmentParams;
      constexpr auto& nested_doc = doc.SystemBase.GraphvizFragmentParams;
      py::class_<Nested>(cls, "GraphvizFragmentParams", nested_doc.doc)
          .def_readwrite(
              "max_depth", &Nested::max_depth, nested_doc.max_depth.doc)
          .def_readwrite("options", &Nested::options, nested_doc.options.doc)
          .def_readwrite("node_id", &Nested::node_id, nested_doc.node_id.doc)
          .def_readwrite("header_lines", &Nested::header_lines,
              nested_doc.header_lines.doc);
    }
    cls  // BR
        .def("GetSystemName", &Class::GetSystemName, cls_doc.GetSystemName.doc)
        .def("GetSystemPathname", &Class::GetSystemPathname,
            cls_doc.GetSystemPathname.doc)
        .def("GetSystemType", &Class::GetSystemType, cls_doc.GetSystemType.doc)
        .def("get_name", &Class::get_name, cls_doc.get_name.doc)
        .def(
            "set_name", &Class::set_name, py::arg("name"), cls_doc.set_name.doc)
        // Graphviz methods.
        .def("GetGraphvizString", &Class::GetGraphvizString,
            py::arg("max_depth") = py::none(), py::arg("options") = py::dict(),
            cls_doc.GetGraphvizString.doc)
        .def("GetGraphvizFragment", &Class::GetGraphvizFragment,
            py::arg("max_depth") = py::none(), py::arg("options") = py::dict(),
            cls_doc.GetGraphvizFragment.doc)
        .def("DoGetGraphvizFragment", &SystemBasePublic::DoGetGraphvizFragment,
            cls_doc.DoGetGraphvizFragment.doc)
        // Topology.
        .def("num_input_ports", &Class::num_input_ports,
            cls_doc.num_input_ports.doc)
        .def("num_output_ports", &Class::num_output_ports,
            cls_doc.num_output_ports.doc)
        // States.
        .def("num_continuous_states", &Class::num_continuous_states,
            cls_doc.num_continuous_states.doc)
        .def("num_discrete_state_groups", &Class::num_discrete_state_groups,
            cls_doc.num_discrete_state_groups.doc)
        .def("num_abstract_states", &Class::num_abstract_states,
            cls_doc.num_abstract_states.doc)
        .def("implicit_time_derivatives_residual_size",
            &Class::implicit_time_derivatives_residual_size,
            cls_doc.implicit_time_derivatives_residual_size.doc)
        .def("ValidateContext",
            overload_cast_explicit<void, const ContextBase&>(
                &Class::ValidateContext),
            py::arg("context"), cls_doc.ValidateContext.doc)
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
            py_rvp::reference_internal, cls_doc.get_cache_entry.doc);
    // N.B. Since this method has template overloads, we must specify the types
    // `overload_cast_explicit`; we must also specify Class. We do not use
    // `static_cast<>` to avoid accidental type mixing.
    // clang-format off
    auto DeclareCacheEntry = overload_cast_explicit<
            CacheEntry&, std::string, ValueProducer, std::set<DependencyTicket>
        >.operator()<Class>(&SystemBasePublic::DeclareCacheEntry);
    // clang-format on
    cls  // BR
        .def("DeclareCacheEntry", DeclareCacheEntry, py_rvp::reference_internal,
            py::arg("description"), py::arg("value_producer"),
            py::arg("prerequisites_of_calc") =
                std::set<DependencyTicket>{Class::all_sources_ticket()},
            doc.SystemBase.DeclareCacheEntry
                .doc_3args_description_value_producer_prerequisites_of_calc);
  }
}

template <typename PyClass>
void DefineSystemScalarConverter(PyClass* cls) {
  auto& converter = *cls;
  {
    converter  // BR
        .def(py::init())
        .def("__copy__",
            [](const SystemScalarConverter& in) -> SystemScalarConverter {
              return in;
            });
    // Bind templated instantiations.
    auto converter_methods = [converter](auto pack) {
      constexpr auto& cls_doc =
          pydrake_doc_systems_framework.drake.systems.SystemScalarConverter;
      using Pack = decltype(pack);
      using T = typename Pack::template type_at<0>;
      using U = typename Pack::template type_at<1>;
      AddTemplateMethod(converter, "IsConvertible",
          &SystemScalarConverter::IsConvertible<T, U>, GetPyParam<T, U>(),
          cls_doc.IsConvertible.doc);
      using system_scalar_converter_internal::AddPydrakeConverterFunction;
      // N.B. The "_AddConstructor" method is called by scalar_conversion.py
      // to register a constructor, similar to MaybeAddConstructor in C++.
      using ConverterFunction = std::function<System<T>*(const System<U>&)>;
      AddTemplateMethod(
          converter, "_AddConstructor",
          [](SystemScalarConverter* self,
              py::function python_converter_function) {
            AddPydrakeConverterFunction(self,
                ConverterFunction{
                    [python_converter_function](const System<U>& system_u_cpp) {
                      py::gil_scoped_acquire guard;
                      // Call the Python converter function.
                      py::object system_u_py =
                          py::cast(system_u_cpp, py_rvp::reference_internal);
                      py::object system_t_py =
                          python_converter_function(system_u_py);
                      DRAKE_THROW_UNLESS(!system_t_py.is_none());
                      // Cast its result to a shared_ptr.
                      std::shared_ptr<System<T>> system_t_cpp =
                          make_shared_ptr_from_py_object<System<T>>(
                              std::move(system_t_py));
                      // Wrap the result in a Diagram so we have a unique_ptr
                      // instead of a shared_ptr.
                      std::unique_ptr<System<T>> result =
                          std::make_unique<WrappedSystem<T>>(
                              std::move(system_t_cpp));
                      // Our contract is to return an owned raw pointer. Our
                      // caller will wrap the unique_ptr back around it.
                      return result.release();
                    }});
          },
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
}

}  // namespace

void DefineFrameworkPySystems(py::module m) {
  DoScalarIndependentDefinitions(m);

  // Declare (but don't define) to resolve a dependency cycle.
  py::class_<SystemScalarConverter> cls_system_scalar_converter(
      m, "SystemScalarConverter");
  auto cls_system_double = Impl<double>::DefineSystem(m);
  auto cls_system_autodiff = Impl<AutoDiffXd>::DefineSystem(m);
  auto cls_system_expression = Impl<Expression>::DefineSystem(m);

  // Do templated instantiations of system types.
  auto bind_common_scalar_types = [&](auto dummy) {
    using T = decltype(dummy);
    py::class_<System<T>, SystemBase, typename Impl<T>::PySystem>* cls_system{};
    if constexpr (std::is_same_v<T, double>) {
      cls_system = &cls_system_double;
    } else if constexpr (std::is_same_v<T, AutoDiffXd>) {
      cls_system = &cls_system_autodiff;
    } else {
      static_assert(std::is_same_v<T, Expression>);
      cls_system = &cls_system_expression;
    }
    Impl<T>::DefineSystemScalarConversions(cls_system);
    Impl<T>::DefineLeafSystem(m);
    Impl<T>::DefineDiagram(m);
    Impl<T>::DefineVectorSystem(m);
    Impl<T>::DefineWrappedSystem(m);
    Impl<T>::DefineSystemVisitor(m, cls_system);
  };
  type_visit(bind_common_scalar_types, CommonScalarPack{});

  DefineSystemScalarConverter(&cls_system_scalar_converter);
}

}  // namespace pydrake
}  // namespace drake
