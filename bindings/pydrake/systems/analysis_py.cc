#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/bindings/generated_docstrings/systems_analysis.h"
#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/scope_exit.h"
#include "drake/systems/analysis/batch_eval.h"
#include "drake/systems/analysis/discrete_time_approximation.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/monte_carlo.h"
#include "drake/systems/analysis/region_of_attraction.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_config.h"
#include "drake/systems/analysis/simulator_config_functions.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/analysis/simulator_python_internal.h"

namespace drake {
namespace pydrake {

namespace {
// Checks for Ctrl-C (and other signals) and invokes the Python handler,
// but only when called on the main interpreter thread. For details, see:
// https://docs.python.org/3/c-api/exceptions.html#c.PyErr_CheckSignals
// https://pybind11.readthedocs.io/en/stable/faq.html#how-can-i-properly-handle-ctrl-c-in-long-running-functions
void ThrowIfPythonHasPendingSignals() {
  py::gil_scoped_acquire guard;
  if (PyErr_CheckSignals() != 0) {
    throw py::error_already_set();
  }
}
}  // namespace

PYBIND11_MODULE(analysis, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::analysis;
  constexpr auto& doc = pydrake_doc_systems_analysis.drake.systems;

  m.doc() = "Bindings for the analysis portion of the Systems framework.";

  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.primitives");
  py::module::import("pydrake.solvers");
  py::module::import("pydrake.trajectories");

  {
    using Class = SimulatorConfig;
    constexpr auto& cls_doc = doc.SimulatorConfig;
    py::class_<Class> cls(m, "SimulatorConfig", cls_doc.doc);
    cls  // BR
        .def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = SimulatorStatus;
    constexpr auto& cls_doc = doc.SimulatorStatus;
    py::class_<Class> cls(m, "SimulatorStatus", cls_doc.doc);

    using Enum = Class::ReturnReason;
    constexpr auto& enum_doc = cls_doc.ReturnReason;
    py::enum_<Class::ReturnReason>(cls, "ReturnReason", enum_doc.doc)
        .value("kReachedBoundaryTime", Enum::kReachedBoundaryTime,
            enum_doc.kReachedBoundaryTime.doc)
        .value("kReachedTerminationCondition",
            Enum::kReachedTerminationCondition,
            enum_doc.kReachedTerminationCondition.doc)
        .value("kEventHandlerFailed", Enum::kEventHandlerFailed,
            enum_doc.kEventHandlerFailed.doc);

    cls  // BR
         // TODO(eric.cousineau): Bind setter methods.
        .def("FormatMessage", &Class::FormatMessage, cls_doc.FormatMessage.doc)
        .def("succeeded", &Class::succeeded, cls_doc.succeeded.doc)
        .def("boundary_time", &Class::boundary_time, cls_doc.boundary_time.doc)
        .def("return_time", &Class::return_time, cls_doc.return_time.doc)
        .def("reason", &Class::reason, cls_doc.reason.doc)
        .def("system", &Class::system, py_rvp::reference, cls_doc.system.doc)
        .def("message", &Class::message, cls_doc.message.doc)
        .def("IsIdenticalStatus", &Class::IsIdenticalStatus, py::arg("other"),
            cls_doc.IsIdenticalStatus.doc);
  }

  {
    constexpr auto& cls_doc = doc.InitializeParams;
    using Class = InitializeParams;
    py::class_<Class> cls(m, "InitializeParams", cls_doc.doc);
    cls  // BR
        .def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  auto bind_scalar_types = [&m](auto dummy) {
    using T = decltype(dummy);

    m.def("BatchEvalUniquePeriodicDiscreteUpdate",
        &BatchEvalUniquePeriodicDiscreteUpdate<T>, py::arg("system"),
        py::arg("context"), py::arg("times"), py::arg("states"),
        py::arg("inputs"), py::arg("num_time_steps") = 1,
        py::arg("input_port_index") =
            InputPortSelection::kUseFirstInputIfItExists,
        py::arg("parallelize") = Parallelism::Max(),
        py::call_guard<py::gil_scoped_release>(),
        doc.BatchEvalUniquePeriodicDiscreteUpdate.doc);

    m.def("BatchEvalTimeDerivatives", &BatchEvalTimeDerivatives<T>,
        py::arg("system"), py::arg("context"), py::arg("times"),
        py::arg("states"), py::arg("inputs"),
        py::arg("input_port_index") =
            InputPortSelection::kUseFirstInputIfItExists,
        py::arg("parallelize") = Parallelism::Max(),
        py::call_guard<py::gil_scoped_release>(),
        doc.BatchEvalTimeDerivatives.doc);

    {
      using Class = IntegratorBase<T>;
      constexpr auto& cls_doc = doc.IntegratorBase;
      DefineTemplateClassWithDefault<Class>(
          m, "IntegratorBase", GetPyParam<T>(), cls_doc.doc)
          .def("set_fixed_step_mode", &Class::set_fixed_step_mode,
              py::arg("flag"), cls_doc.set_fixed_step_mode.doc)
          .def("get_fixed_step_mode", &Class::get_fixed_step_mode,
              cls_doc.get_fixed_step_mode.doc)
          .def("set_target_accuracy", &Class::set_target_accuracy,
              py::arg("accuracy"), cls_doc.set_target_accuracy.doc)
          .def("get_target_accuracy", &Class::get_target_accuracy,
              cls_doc.get_target_accuracy.doc)
          .def("request_initial_step_size_target",
              &Class::request_initial_step_size_target, py::arg("step_size"),
              cls_doc.request_initial_step_size_target.doc)
          .def("get_initial_step_size_target",
              &Class::get_initial_step_size_target,
              cls_doc.get_initial_step_size_target.doc)
          .def("set_maximum_step_size", &Class::set_maximum_step_size,
              py::arg("max_step_size"), cls_doc.set_maximum_step_size.doc)
          .def("get_maximum_step_size", &Class::get_maximum_step_size,
              cls_doc.get_maximum_step_size.doc)
          .def("set_requested_minimum_step_size",
              &Class::set_requested_minimum_step_size, py::arg("min_step_size"),
              cls_doc.set_requested_minimum_step_size.doc)
          .def("get_requested_minimum_step_size",
              &Class::get_requested_minimum_step_size,
              cls_doc.get_requested_minimum_step_size.doc)
          .def("set_throw_on_minimum_step_size_violation",
              &Class::set_throw_on_minimum_step_size_violation,
              py::arg("throws"),
              cls_doc.set_throw_on_minimum_step_size_violation.doc)
          .def("get_throw_on_minimum_step_size_violation",
              &Class::get_throw_on_minimum_step_size_violation,
              cls_doc.get_throw_on_minimum_step_size_violation.doc)
          .def("Reset", &Class::Reset, cls_doc.Reset.doc)
          .def("Initialize", &Class::Initialize, cls_doc.Initialize.doc)
          .def("StartDenseIntegration", &Class::StartDenseIntegration,
              cls_doc.StartDenseIntegration.doc)
          .def("get_dense_output", &Class::get_dense_output,
              py_rvp::reference_internal, cls_doc.get_dense_output.doc)
          .def(
              "StopDenseIntegration",
              [](Class* self) -> trajectories::PiecewisePolynomial<T>* {
                // Having abandoned the old RobotLocomotion pybind11 branch
                // with special handling of std::unique_ptr<>, this binding's
                // return value path started deleting the C++ object and
                // returning a dead non-null pointer. To avoid that, we
                // instead explicitly unwrap the pointer here and rely on the
                // take_ownership return value policy. The take_ownership
                // policy would be the default policy in this case, but it
                // seems safer and more clear to apply it explicitly.
                std::unique_ptr<trajectories::PiecewisePolynomial<T>> result =
                    self->StopDenseIntegration();
                return result.release();
              },
              py_rvp::take_ownership, cls_doc.StopDenseIntegration.doc)
          .def("ResetStatistics", &Class::ResetStatistics,
              cls_doc.ResetStatistics.doc)
          .def("get_num_substep_failures", &Class::get_num_substep_failures,
              cls_doc.get_num_substep_failures.doc)
          .def("get_num_step_shrinkages_from_substep_failures",
              &Class::get_num_step_shrinkages_from_substep_failures,
              cls_doc.get_num_step_shrinkages_from_substep_failures.doc)
          .def("get_num_step_shrinkages_from_error_control",
              &Class::get_num_step_shrinkages_from_error_control,
              cls_doc.get_num_step_shrinkages_from_error_control.doc)
          .def("get_num_derivative_evaluations",
              &Class::get_num_derivative_evaluations,
              cls_doc.get_num_derivative_evaluations.doc)
          .def("get_actual_initial_step_size_taken",
              &Class::get_actual_initial_step_size_taken,
              cls_doc.get_actual_initial_step_size_taken.doc)
          .def("get_smallest_adapted_step_size_taken",
              &Class::get_smallest_adapted_step_size_taken,
              cls_doc.get_smallest_adapted_step_size_taken.doc)
          .def("get_largest_step_size_taken",
              &Class::get_largest_step_size_taken,
              cls_doc.get_largest_step_size_taken.doc)
          .def("get_num_steps_taken", &Class::get_num_steps_taken,
              cls_doc.get_num_steps_taken.doc)
          // N.B. While `context` is not directly owned by this system, we
          // would still like our accessors to keep it alive (e.g. a user calls
          // `simulator.get_integrator().get_context()`.
          .def("get_context", &Class::get_context,
              // Keep alive, transitive: `return` keeps `self` alive.
              py::keep_alive<0, 1>(), cls_doc.get_context.doc)
          .def("get_mutable_context", &Class::get_mutable_context,
              // Keep alive, transitive: `return` keeps `self` alive.
              py::keep_alive<0, 1>(), cls_doc.get_mutable_context.doc)
          .def("reset_context",
              py::overload_cast<Context<T>*>(&Class::reset_context),
              py::arg("context"),
              // Keep alive, reference: `context` keeps `self` alive.
              py::keep_alive<2, 1>(), cls_doc.reset_context.doc);
    }

    DefineTemplateClassWithDefault<RungeKutta2Integrator<T>, IntegratorBase<T>>(
        m, "RungeKutta2Integrator", GetPyParam<T>(),
        doc.RungeKutta2Integrator.doc)
        .def(py::init<const System<T>&, const T&, Context<T>*>(),
            py::arg("system"), py::arg("max_step_size"),
            py::arg("context") = nullptr,
            // Keep alive, reference: `self` keeps `system` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `context` alive.
            py::keep_alive<1, 4>(), doc.RungeKutta2Integrator.ctor.doc);

    {
      m.def("DiscreteTimeApproximation",
          overload_cast_explicit<std::unique_ptr<LinearSystem<T>>,
              const LinearSystem<T>&, double>(&DiscreteTimeApproximation),
          py::arg("linear_system"), py::arg("time_period"),
          doc.DiscreteTimeApproximation.doc_2args_constLinearSystem_double);

      m.def("DiscreteTimeApproximation",
          overload_cast_explicit<std::unique_ptr<AffineSystem<T>>,
              const AffineSystem<T>&, double>(&DiscreteTimeApproximation),
          py::arg("affine_system"), py::arg("time_period"),
          doc.DiscreteTimeApproximation.doc_2args_constAffineSystem_double);

      m.def(
          "DiscreteTimeApproximation",
          [](const System<T>& system, double time_period,
              const SimulatorConfig& integrator_config) {
            return DiscreteTimeApproximation(
                // The lifetime of `system` is managed by the keep_alive
                // below, not the C++ shared_ptr.
                make_unowned_shared_ptr_from_raw(&system), time_period,
                integrator_config);
          },
          py::arg("system"), py::arg("time_period"),
          py::arg("integrator_config") = SimulatorConfig(),
          // Keep alive, reference: `result` keeps `system` alive.
          py::keep_alive<0, 1>(),
          doc.DiscreteTimeApproximation
              .doc_3args_constSystem_double_SimulatorConfig);
    }
  };
  type_visit(bind_scalar_types, CommonScalarPack{});

  auto bind_nonsymbolic_scalar_types = [&m](auto dummy) {
    using T = decltype(dummy);

    DefineTemplateClassWithDefault<RungeKutta3Integrator<T>, IntegratorBase<T>>(
        m, "RungeKutta3Integrator", GetPyParam<T>(),
        doc.RungeKutta3Integrator.doc)
        .def(py::init<const System<T>&, Context<T>*>(), py::arg("system"),
            py::arg("context") = nullptr,
            // Keep alive, reference: `self` keeps `system` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `context` alive.
            py::keep_alive<1, 3>(), doc.RungeKutta3Integrator.ctor.doc);

    // See equivalent note about EventCallback in `framework_py_systems.cc`.
    using MonitorCallback =
        std::function<std::optional<EventStatus>(const Context<T>&)>;

    auto cls = DefineTemplateClassWithDefault<Simulator<T>>(
        m, "Simulator", GetPyParam<T>(), doc.Simulator.doc);
    cls  // BR
        .def(py::init([](const System<T>& system, py::object py_context) {
          // Handle the two cases for context ownership explicitly:
          // 1. If py_context is None, create a new context and take ownership.
          // 2. If py_context is provided, use the existing Python wrapper
          //    directly (it already owns the C++ object).
          if (py_context.is_none()) {
            std::unique_ptr<Context<T>> context_ptr =
                system.CreateDefaultContext();
            // Use take_ownership because we just created this context and need
            // Python to own it. The unique_ptr is released, leaving the raw
            // pointer with no owner until take_ownership establishes Python
            // ownership.
            py_context =
                py::cast(context_ptr.release(), py_rvp::take_ownership);
          }
          return Simulator<T>::MakeWithSharedContext(
              system, make_shared_ptr_from_py_object<Context<T>>(py_context));
        }),
            py::arg("system"), py::arg("context") = py::none(),
            // Keep alive, reference: `self` keeps `system` alive.
            py::keep_alive<1, 2>(),
            []() {
              std::string new_doc = doc.Simulator.ctor.doc;
              new_doc += R"""(

(Python only) The Simulator's Context, whether provided as a constructor
argument or allocated internally, will have a lifetime managed by Python
reference counting. Note, however, that the simulator logically "owns" the
context; it will modify the context in most of its methods. Therefore, sharing
a Context object among Simulators will likely lead to incorrect results.
)""";
              return new_doc;
            }()
                .c_str())
        .def("Initialize", &Simulator<T>::Initialize,
            doc.Simulator.Initialize.doc,
            py::arg("params") = InitializeParams{})
        .def(
            "AdvanceTo",
            [](Simulator<T>* self, const T& boundary_time, bool interruptible) {
              if (!interruptible) {
                return self->AdvanceTo(boundary_time);
              }
              // Enable the interrupt monitor.
              using systems::internal::SimulatorPythonInternal;
              SimulatorPythonInternal<T>::set_python_monitor(
                  self, &ThrowIfPythonHasPendingSignals);
              ScopeExit guard([self]() {
                SimulatorPythonInternal<T>::set_python_monitor(self, nullptr);
              });
              return self->AdvanceTo(boundary_time);
            },
            py::arg("boundary_time"), py::arg("interruptible") = true,
            // This is a long-running function that might sleep; for both
            // reasons, we must release the GIL.
            py::call_guard<py::gil_scoped_release>(),
            // Amend the docstring with the additional parameter.
            []() {
              std::string new_doc = doc.Simulator.AdvanceTo.doc;
              auto found = new_doc.find("\nReturns");
              DRAKE_DEMAND(found != std::string::npos);
              new_doc.insert(found + 1, R"""(
Parameter ``interruptible``:
    When True, the simulator will check for ``KeyboardInterrupt``
    signals (Ctrl-C) during the call to AdvanceTo(). When False,
    the AdvanceTo() may or may not be interruptible, depending on
    what systems and/or monitors have been added to the simulator.
    The check has a very minor runtime performance cost, so can be
    disabled by passing ``False``. This is a Python-only parameter
    (not available in the C++ API).
)""");
              return new_doc;
            }()
                .c_str())
        .def("AdvancePendingEvents", &Simulator<T>::AdvancePendingEvents,
            doc.Simulator.AdvancePendingEvents.doc)
        .def("set_monitor",
            WrapCallbacks([](Simulator<T>* self, MonitorCallback monitor) {
              self->set_monitor([monitor](const Context<T>& context) {
                return monitor(context).value_or(EventStatus::DidNothing());
              });
            }),
            py::arg("monitor"), doc.Simulator.set_monitor.doc)
        .def("clear_monitor", &Simulator<T>::clear_monitor,
            doc.Simulator.clear_monitor.doc)
        .def("get_monitor", &Simulator<T>::get_monitor,
            doc.Simulator.get_monitor.doc)
        .def("get_context", &Simulator<T>::get_context,
            py_rvp::reference_internal, doc.Simulator.get_context.doc)
        .def("get_integrator", &Simulator<T>::get_integrator,
            py_rvp::reference_internal, doc.Simulator.get_integrator.doc)
        .def("get_mutable_integrator", &Simulator<T>::get_mutable_integrator,
            py_rvp::reference_internal,
            doc.Simulator.get_mutable_integrator.doc)
        .def("get_mutable_context", &Simulator<T>::get_mutable_context,
            py_rvp::reference_internal, doc.Simulator.get_mutable_context.doc)
        .def("has_context", &Simulator<T>::has_context,
            doc.Simulator.has_context.doc)
        .def(
            "reset_context",
            [](Simulator<T>* self, Context<T>* context) {
              auto py_context = py::cast(context);
              self->reset_context_from_shared(
                  make_shared_ptr_from_py_object<Context<T>>(py_context));
            },
            py::arg("context"), doc.Simulator.reset_context.doc)
        .def("set_target_realtime_rate",
            &Simulator<T>::set_target_realtime_rate, py::arg("realtime_rate"),
            doc.Simulator.set_target_realtime_rate.doc)
        .def("get_target_realtime_rate",
            &Simulator<T>::get_target_realtime_rate,
            doc.Simulator.get_target_realtime_rate.doc)
        .def("get_actual_realtime_rate",
            &Simulator<T>::get_actual_realtime_rate,
            doc.Simulator.get_actual_realtime_rate.doc)
        .def("ResetStatistics", &Simulator<T>::ResetStatistics,
            doc.Simulator.ResetStatistics.doc)
        .def("get_num_publishes", &Simulator<T>::get_num_publishes,
            doc.Simulator.get_num_publishes.doc)
        .def("get_num_steps_taken", &Simulator<T>::get_num_steps_taken,
            doc.Simulator.get_num_steps_taken.doc)
        .def("get_num_discrete_updates",
            &Simulator<T>::get_num_discrete_updates,
            doc.Simulator.get_num_discrete_updates.doc)
        .def("get_num_unrestricted_updates",
            &Simulator<T>::get_num_unrestricted_updates,
            doc.Simulator.get_num_unrestricted_updates.doc)
        .def("get_system", &Simulator<T>::get_system, py_rvp::reference,
            doc.Simulator.get_system.doc);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    // delete with publish_every_time_step 2026-06-01
    cls.def("set_publish_every_time_step",
           WrapDeprecated(
               doc.Simulator.set_publish_every_time_step.doc_deprecated,
               &Simulator<T>::set_publish_every_time_step),
           py::arg("publish"),
           doc.Simulator.set_publish_every_time_step.doc_deprecated)
        .def("set_publish_at_initialization",
            WrapDeprecated(
                doc.Simulator.set_publish_at_initialization.doc_deprecated,
                &Simulator<T>::set_publish_at_initialization),
            py::arg("publish"),
            doc.Simulator.set_publish_at_initialization.doc_deprecated);
    // delete till here
#pragma GCC diagnostic pop
    m  // BR
        .def("ApplySimulatorConfig",
            py::overload_cast<const SimulatorConfig&,
                drake::systems::Simulator<T>*>(&ApplySimulatorConfig<T>),
            py::arg("config"), py::arg("simulator"),
            doc.ApplySimulatorConfig.doc_config_sim)
        .def("ExtractSimulatorConfig", &ExtractSimulatorConfig<T>,
            py::arg("simulator"), doc.ExtractSimulatorConfig.doc);
  };
  type_visit(bind_nonsymbolic_scalar_types, NonSymbolicScalarPack{});

  // Simulator Flags
  m  // BR
      .def(
          "ResetIntegratorFromFlags",
          [](Simulator<double>* simulator, const std::string& scheme,
              const double& max_step_size) {
            IntegratorBase<double>& result =
                ResetIntegratorFromFlags(simulator, scheme, max_step_size);
            return &result;
          },
          py::arg("simulator"), py::arg("scheme"), py::arg("max_step_size"),
          py_rvp::reference,
          // Keep alive, reference: `return` keeps `simulator` alive.
          py::keep_alive<0, 1>(), doc.ResetIntegratorFromFlags.doc)
      .def(
          "ResetIntegratorFromFlags",
          [](Simulator<AutoDiffXd>* simulator, const std::string& scheme,
              const AutoDiffXd& max_step_size) {
            IntegratorBase<AutoDiffXd>& result =
                ResetIntegratorFromFlags(simulator, scheme, max_step_size);
            return &result;
          },
          py::arg("simulator"), py::arg("scheme"), py::arg("max_step_size"),
          py_rvp::reference,
          // Keep alive, reference: `return` keeps `simulator` alive.
          py::keep_alive<0, 1>(), doc.ResetIntegratorFromFlags.doc)
      .def("GetIntegrationSchemes", &GetIntegrationSchemes,
          doc.GetIntegrationSchemes.doc);

  // Print Simulator Statistics
  m  // BR
      .def("PrintSimulatorStatistics", &PrintSimulatorStatistics<double>,
          doc.PrintSimulatorStatistics.doc)
      .def("PrintSimulatorStatistics", &PrintSimulatorStatistics<AutoDiffXd>,
          doc.PrintSimulatorStatistics.doc);

  // Monte Carlo Testing
  {
    // Like RandomSimulatorFactory but returning a Python object instead of C++.
    using PyRandomSimulatorFactory =
        std::function<py::object(RandomGenerator*)>;
    auto make_cpp_compatible_factory =
        [](PyRandomSimulatorFactory factory_py) -> RandomSimulatorFactory {
      return [factory_py = std::move(factory_py)](RandomGenerator* generator) {
        py::gil_scoped_acquire guard;
        py::object make_simulator_result = factory_py(generator);
        DRAKE_THROW_UNLESS(!make_simulator_result.is_none());
        return make_shared_ptr_from_py_object<Simulator<double>>(
            std::move(make_simulator_result));
      };
    };

    // Like ScalarSystemFunction but with optional<> added to the return type.
    // (This leads to better error messages in case the user forgot to return.)
    using PyScalarSystemFunction = std::function<std::optional<double>(
        const System<double>* system, const Context<double>* context)>;
    auto make_cpp_compatible_output =
        [](PyScalarSystemFunction output_py) -> ScalarSystemFunction {
      return [output_py = std::move(output_py)](
                 const System<double>& system, const Context<double>& context) {
        py::gil_scoped_acquire guard;
        std::optional<double> scalar_system_function_output =
            output_py(&system, &context);
        DRAKE_THROW_UNLESS(scalar_system_function_output.has_value());
        return *scalar_system_function_output;
      };
    };

    m.def(
        "RandomSimulation",
        [&make_cpp_compatible_factory, &make_cpp_compatible_output](
            PyRandomSimulatorFactory make_simulator,
            PyScalarSystemFunction output, double final_time,
            RandomGenerator* generator) -> double {
          return RandomSimulation(
              make_cpp_compatible_factory(std::move(make_simulator)),
              make_cpp_compatible_output(std::move(output)), final_time,
              generator);
        },
        py::arg("make_simulator"), py::arg("output"), py::arg("final_time"),
        py::arg("generator"), doc.analysis.RandomSimulation.doc);

    py::class_<RandomSimulationResult>(
        m, "RandomSimulationResult", doc.analysis.RandomSimulationResult.doc)
        .def_readwrite("output", &RandomSimulationResult::output,
            doc.analysis.RandomSimulationResult.output.doc)
        .def_readwrite("generator_snapshot",
            &RandomSimulationResult::generator_snapshot,
            doc.analysis.RandomSimulationResult.generator_snapshot.doc);

    // Note: This hard-codes `parallelism` to be off, since parallel execution
    // of Python systems on multiple threads was thought to be unsupported. It's
    // possible that with `py::call_guard<py::gil_scoped_release>` it would
    // actually be fine, so we could revisit that decision at some point.
    m.def(
        "MonteCarloSimulation",
        [&make_cpp_compatible_factory, &make_cpp_compatible_output](
            PyRandomSimulatorFactory make_simulator,
            PyScalarSystemFunction output, double final_time, int num_samples,
            RandomGenerator* generator) -> std::vector<RandomSimulationResult> {
          return MonteCarloSimulation(
              make_cpp_compatible_factory(std::move(make_simulator)),
              make_cpp_compatible_output(std::move(output)), final_time,
              num_samples, generator, /* parallelism = */ Parallelism::None());
        },
        py::arg("make_simulator"), py::arg("output"), py::arg("final_time"),
        py::arg("num_samples"), py::arg("generator"),
        doc.analysis.MonteCarloSimulation.doc);
  }

  {
    using Class = RegionOfAttractionOptions;
    constexpr auto& cls_doc = doc.analysis.RegionOfAttractionOptions;
    py::class_<Class, std::shared_ptr<Class>> cls(
        m, "RegionOfAttractionOptions", cls_doc.doc);
    cls.def(py::init<>(), cls_doc.ctor.doc)
        // TODO(jeremy.nimmer): replace the def_readwrite with
        // DefAttributesUsingSerialize when we fix binding a
        // VectorX<symbolic::Variable> state_variables to a numpy array of
        // objects.
        .def_readwrite("lyapunov_candidate",
            &RegionOfAttractionOptions::lyapunov_candidate,
            cls_doc.lyapunov_candidate.doc)
        .def_readwrite("state_variables",
            &RegionOfAttractionOptions::state_variables,
            // dtype = object arrays must be copied, and cannot be referenced.
            py_rvp::copy, cls_doc.state_variables.doc)
        .def_readwrite("use_implicit_dynamics",
            &RegionOfAttractionOptions::use_implicit_dynamics,
            cls_doc.use_implicit_dynamics.doc)
        .def_readwrite("solver_id", &RegionOfAttractionOptions::solver_id,
            cls_doc.solver_id.doc)
        .def_readwrite("solver_options",
            &RegionOfAttractionOptions::solver_options,
            cls_doc.solver_options.doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);

    m.def("RegionOfAttraction", &RegionOfAttraction, py::arg("system"),
        py::arg("context"), py::arg("options") = RegionOfAttractionOptions(),
        doc.analysis.RegionOfAttraction.doc);
  }
}

}  // namespace pydrake
}  // namespace drake
