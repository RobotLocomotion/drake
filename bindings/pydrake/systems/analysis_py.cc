#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/analysis/integrator_base.h"
#include "drake/systems/analysis/monte_carlo.h"
#include "drake/systems/analysis/region_of_attraction.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/runge_kutta3_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_config_functions.h"
#include "drake/systems/analysis/simulator_print_stats.h"

using std::unique_ptr;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(analysis, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;

  m.doc() = "Bindings for the analysis portion of the Systems framework.";

  py::module::import("pydrake.systems.framework");

  {
    constexpr auto& doc = pydrake_doc.drake.systems;
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
    constexpr auto& cls_doc = pydrake_doc.drake.systems.InitializeParams;
    using Class = InitializeParams;
    py::class_<Class>(m, "InitializeParams", cls_doc.doc)
        .def(ParamInit<Class>())
        .def_readwrite("suppress_initialization_events",
            &Class::suppress_initialization_events,
            cls_doc.suppress_initialization_events.doc)
        .def("__repr__", [](const Class& self) {
          return py::str("InitializeParams(suppress_initialization_events={})")
              .format(self.suppress_initialization_events);
        });
  }

  auto bind_scalar_types = [m](auto dummy) {
    constexpr auto& doc = pydrake_doc.drake.systems;
    using T = decltype(dummy);
    DefineTemplateClassWithDefault<IntegratorBase<T>>(
        m, "IntegratorBase", GetPyParam<T>(), doc.IntegratorBase.doc)
        .def("set_fixed_step_mode", &IntegratorBase<T>::set_fixed_step_mode,
            doc.IntegratorBase.set_fixed_step_mode.doc)
        .def("get_fixed_step_mode", &IntegratorBase<T>::get_fixed_step_mode,
            doc.IntegratorBase.get_fixed_step_mode.doc)
        .def("set_target_accuracy", &IntegratorBase<T>::set_target_accuracy,
            doc.IntegratorBase.set_target_accuracy.doc)
        .def("get_target_accuracy", &IntegratorBase<T>::get_target_accuracy,
            doc.IntegratorBase.get_target_accuracy.doc)
        .def("set_maximum_step_size", &IntegratorBase<T>::set_maximum_step_size,
            doc.IntegratorBase.set_maximum_step_size.doc)
        .def("get_maximum_step_size", &IntegratorBase<T>::get_maximum_step_size,
            doc.IntegratorBase.get_maximum_step_size.doc)
        .def("set_requested_minimum_step_size",
            &IntegratorBase<T>::set_requested_minimum_step_size,
            doc.IntegratorBase.set_requested_minimum_step_size.doc)
        .def("get_requested_minimum_step_size",
            &IntegratorBase<T>::get_requested_minimum_step_size,
            doc.IntegratorBase.get_requested_minimum_step_size.doc)
        .def("set_throw_on_minimum_step_size_violation",
            &IntegratorBase<T>::set_throw_on_minimum_step_size_violation,
            doc.IntegratorBase.set_throw_on_minimum_step_size_violation.doc)
        .def("get_throw_on_minimum_step_size_violation",
            &IntegratorBase<T>::get_throw_on_minimum_step_size_violation,
            doc.IntegratorBase.get_throw_on_minimum_step_size_violation.doc)
        .def("StartDenseIntegration", &IntegratorBase<T>::StartDenseIntegration,
            doc.IntegratorBase.StartDenseIntegration.doc)
        .def("get_dense_output", &IntegratorBase<T>::get_dense_output,
            py_rvp::reference_internal, doc.IntegratorBase.get_dense_output.doc)
        .def("StopDenseIntegration", &IntegratorBase<T>::StopDenseIntegration,
            doc.IntegratorBase.StopDenseIntegration.doc);

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
  };
  type_visit(bind_scalar_types, CommonScalarPack{});

  auto bind_nonsymbolic_scalar_types = [m](auto dummy) {
    constexpr auto& doc = pydrake_doc.drake.systems;
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

    auto cls = DefineTemplateClassWithDefault<Simulator<T>>(
        m, "Simulator", GetPyParam<T>(), doc.Simulator.doc);
    cls  // BR
        .def(py::init<const System<T>&, unique_ptr<Context<T>>>(),
            py::arg("system"), py::arg("context") = nullptr,
            // Keep alive, reference: `self` keeps `system` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, ownership: `context` keeps `self` alive.
            py::keep_alive<3, 1>(), doc.Simulator.ctor.doc)
        .def("Initialize", &Simulator<T>::Initialize,
            doc.Simulator.Initialize.doc,
            py::arg("params") = InitializeParams{})
        .def("AdvanceTo", &Simulator<T>::AdvanceTo, py::arg("boundary_time"),
            doc.Simulator.AdvanceTo.doc)
        .def("AdvancePendingEvents", &Simulator<T>::AdvancePendingEvents,
            doc.Simulator.AdvancePendingEvents.doc)
        .def("set_monitor", WrapCallbacks(&Simulator<T>::set_monitor),
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
        .def("reset_context", &Simulator<T>::reset_context, py::arg("context"),
            // Keep alive, ownership: `context` keeps `self` alive.
            py::keep_alive<2, 1>(), doc.Simulator.reset_context.doc)
        // TODO(eric.cousineau): Bind `release_context` once some form of the
        // PR RobotLocomotion/pybind11#33 lands. Presently, it fails.
        .def("set_publish_every_time_step",
            &Simulator<T>::set_publish_every_time_step,
            doc.Simulator.set_publish_every_time_step.doc)
        .def("set_target_realtime_rate",
            &Simulator<T>::set_target_realtime_rate,
            doc.Simulator.set_target_realtime_rate.doc)
        .def("get_target_realtime_rate",
            &Simulator<T>::get_target_realtime_rate,
            doc.Simulator.get_target_realtime_rate.doc)
        .def("get_actual_realtime_rate",
            &Simulator<T>::get_actual_realtime_rate,
            doc.Simulator.get_actual_realtime_rate.doc)
        .def("ResetStatistics", &Simulator<T>::ResetStatistics,
            doc.Simulator.ResetStatistics.doc)
        .def("get_system", &Simulator<T>::get_system, py_rvp::reference,
            doc.Simulator.get_system.doc);
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
          py::keep_alive<0, 1>(),
          pydrake_doc.drake.systems.ResetIntegratorFromFlags.doc)
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
          py::keep_alive<0, 1>(),
          pydrake_doc.drake.systems.ResetIntegratorFromFlags.doc)
      .def("GetIntegrationSchemes", &GetIntegrationSchemes,
          pydrake_doc.drake.systems.GetIntegrationSchemes.doc);
  // Print Simulator Statistics
  m  // BR
      .def("PrintSimulatorStatistics", &PrintSimulatorStatistics<double>,
          pydrake_doc.drake.systems.PrintSimulatorStatistics.doc)
      .def("PrintSimulatorStatistics", &PrintSimulatorStatistics<AutoDiffXd>,
          pydrake_doc.drake.systems.PrintSimulatorStatistics.doc);

  // Monte Carlo Testing
  {
    // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
    using namespace drake::systems::analysis;
    constexpr auto& doc = pydrake_doc.drake.systems.analysis;

    m.def("RandomSimulation",
        WrapCallbacks([](const SimulatorFactory make_simulator,
                          const ScalarSystemFunction& output, double final_time,
                          RandomGenerator* generator) -> double {
          return RandomSimulation(
              make_simulator, output, final_time, generator);
        }),
        py::arg("make_simulator"), py::arg("output"), py::arg("final_time"),
        py::arg("generator"), doc.RandomSimulation.doc);

    py::class_<RandomSimulationResult>(
        m, "RandomSimulationResult", doc.RandomSimulationResult.doc)
        .def_readwrite("output", &RandomSimulationResult::output,
            doc.RandomSimulationResult.output.doc)
        .def_readwrite("generator_snapshot",
            &RandomSimulationResult::generator_snapshot,
            doc.RandomSimulationResult.generator_snapshot.doc);

    // Note: parallel simulation must be disabled in the binding via
    // num_parallel_executions=kNoConcurrency, since parallel execution of
    // Python systems in multiple threads is not supported.
    m.def("MonteCarloSimulation",
        WrapCallbacks([](const SimulatorFactory make_simulator,
                          const ScalarSystemFunction& output, double final_time,
                          int num_samples, RandomGenerator* generator)
                          -> std::vector<RandomSimulationResult> {
          return MonteCarloSimulation(make_simulator, output, final_time,
              num_samples, generator, kNoConcurrency);
        }),
        py::arg("make_simulator"), py::arg("output"), py::arg("final_time"),
        py::arg("num_samples"), py::arg("generator"),
        doc.MonteCarloSimulation.doc);

    py::class_<RegionOfAttractionOptions>(
        m, "RegionOfAttractionOptions", doc.RegionOfAttractionOptions.doc)
        .def(py::init<>(), doc.RegionOfAttractionOptions.ctor.doc)
        .def_readwrite("lyapunov_candidate",
            &RegionOfAttractionOptions::lyapunov_candidate,
            doc.RegionOfAttractionOptions.lyapunov_candidate.doc)
        .def_readwrite("state_variables",
            &RegionOfAttractionOptions::state_variables,
            doc.RegionOfAttractionOptions.state_variables.doc)
        .def("__repr__", [](const RegionOfAttractionOptions& self) {
          return py::str(
              "RegionOfAttractionOptions("
              "lyapunov_candidate={}, "
              "state_variables={})")
              .format(self.lyapunov_candidate, self.state_variables);
        });

    m.def("RegionOfAttraction", &RegionOfAttraction, py::arg("system"),
        py::arg("context"), py::arg("options") = RegionOfAttractionOptions(),
        doc.RegionOfAttraction.doc);
  }
}

}  // namespace pydrake
}  // namespace drake
