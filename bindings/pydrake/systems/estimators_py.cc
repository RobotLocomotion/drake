#include "drake/bindings/generated_docstrings/systems_estimators.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/estimators/kalman_filter.h"
#include "drake/systems/estimators/luenberger_observer.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(estimators, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::estimators;
  constexpr auto& doc = pydrake_doc_systems_estimators.drake.systems.estimators;

  using drake::systems::Context;
  using drake::systems::LeafSystem;
  using drake::systems::System;

  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.primitives");

  auto bind_common_scalar_types = [&m, &doc](auto dummy) {
    using T = decltype(dummy);

    DefineTemplateClassWithDefault<LuenbergerObserver<T>, LeafSystem<T>>(
        m, "LuenbergerObserver", GetPyParam<T>(), doc.LuenbergerObserver.doc)
        .def(py::init<const System<T>&, const Context<T>&,
                 const Eigen::Ref<const Eigen::MatrixXd>&>(),
            py::arg("observed_system"), py::arg("observed_system_context"),
            py::arg("observer_gain"),
            // Keep alive, reference: `self` keeps `observed_system` alive.
            py::keep_alive<1, 2>(), doc.LuenbergerObserver.ctor.doc)
        .def("get_observed_system_input_input_port",
            &LuenbergerObserver<T>::get_observed_system_input_input_port,
            py_rvp::reference_internal,
            doc.LuenbergerObserver.get_observed_system_input_input_port.doc)
        .def("get_observed_system_output_input_port",
            &LuenbergerObserver<T>::get_observed_system_output_input_port,
            py_rvp::reference_internal,
            doc.LuenbergerObserver.get_observed_system_output_input_port.doc)
        .def("get_estimated_state_output_port",
            &LuenbergerObserver<T>::get_estimated_state_output_port,
            py_rvp::reference_internal,
            doc.LuenbergerObserver.get_estimated_state_output_port.doc)
        .def("observer_gain", &LuenbergerObserver<T>::observer_gain,
            py_rvp::reference_internal,
            doc.LuenbergerObserver.observer_gain.doc)
        .def("L", &LuenbergerObserver<T>::L, py_rvp::reference_internal,
            doc.LuenbergerObserver.L.doc);
  };
  type_visit(bind_common_scalar_types, CommonScalarPack{});

  {
    using drake::systems::LinearSystem;

    m.def("SteadyStateKalmanFilter",
        py::overload_cast<const Eigen::Ref<const Eigen::MatrixXd>&,
            const Eigen::Ref<const Eigen::MatrixXd>&,
            const Eigen::Ref<const Eigen::MatrixXd>&,
            const Eigen::Ref<const Eigen::MatrixXd>&>(&SteadyStateKalmanFilter),
        py::arg("A"), py::arg("C"), py::arg("W"), py::arg("V"),
        doc.SteadyStateKalmanFilter.doc_ACWV);

    m.def("DiscreteTimeSteadyStateKalmanFilter",
        &DiscreteTimeSteadyStateKalmanFilter, py::arg("A"), py::arg("C"),
        py::arg("W"), py::arg("V"),
        doc.DiscreteTimeSteadyStateKalmanFilter.doc);

    m.def(
        "SteadyStateKalmanFilter",
        [](const LinearSystem<double>& system,
            const Eigen::Ref<const Eigen::MatrixXd>& W,
            const Eigen::Ref<const Eigen::MatrixXd>& V) {
          return SteadyStateKalmanFilter(
              // The lifetime of `system` is managed by the keep_alive below,
              // not the C++ shared_ptr.
              make_unowned_shared_ptr_from_raw(&system), W, V);
        },
        py::arg("system"), py::arg("W"), py::arg("V"),
        // Keep alive, reference: `result` keeps `system` alive.
        py::keep_alive<0, 1>(), doc.SteadyStateKalmanFilter.doc_linear_system);

    m.def(
        "SteadyStateKalmanFilter",
        [](const System<double>& system, const Context<double>& context,
            const Eigen::Ref<const Eigen::MatrixXd>& W,
            const Eigen::Ref<const Eigen::MatrixXd>& V) {
          return SteadyStateKalmanFilter(
              // The lifetime of `system` is managed by the keep_alive below,
              // not the C++ shared_ptr.
              make_unowned_shared_ptr_from_raw(&system), context, W, V);
        },
        py::arg("system"), py::arg("context"), py::arg("W"), py::arg("V"),
        // Keep alive, reference: `result` keeps `system` alive.
        py::keep_alive<0, 1>(), doc.SteadyStateKalmanFilter.doc_system);
  }
}

}  // namespace pydrake
}  // namespace drake
