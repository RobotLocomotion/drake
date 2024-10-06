#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/estimators/kalman_filter.h"
#include "drake/systems/estimators/luenberger_observer.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(estimators, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::estimators;
  constexpr auto& doc = pydrake_doc.drake.systems.estimators;

  using drake::systems::Context;
  using drake::systems::LeafSystem;
  using drake::systems::System;

  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.primitives");

  {
    using Class = LuenbergerObserver<double>;
    constexpr auto& cls_doc = doc.LuenbergerObserver;
    py::class_<Class, LeafSystem<double>>(m, "LuenbergerObserver", cls_doc.doc)
        .def(py::init<const System<double>&, const Context<double>&,
                 const Eigen::Ref<const Eigen::MatrixXd>&>(),
            py::arg("observed_system"), py::arg("observed_system_context"),
            py::arg("observer_gain"), cls_doc.ctor.doc)
        .def("get_observed_system_input_input_port",
            &Class::get_observed_system_input_input_port,
            py_rvp::reference_internal,
            cls_doc.get_observed_system_input_input_port.doc)
        .def("get_observed_system_output_input_port",
            &Class::get_observed_system_output_input_port,
            py_rvp::reference_internal,
            cls_doc.get_observed_system_output_input_port.doc)
        .def("get_estimated_state_output_port",
            &Class::get_estimated_state_output_port, py_rvp::reference_internal,
            cls_doc.get_estimated_state_output_port.doc)
        .def("observer_gain", &Class::observer_gain, py_rvp::reference_internal,
            cls_doc.observer_gain.doc)
        .def("L", &Class::L, py_rvp::reference_internal, cls_doc.L.doc);
  }

  m.def("SteadyStateKalmanFilter",
      py::overload_cast<const Eigen::Ref<const Eigen::MatrixXd>&,
          const Eigen::Ref<const Eigen::MatrixXd>&,
          const Eigen::Ref<const Eigen::MatrixXd>&,
          const Eigen::Ref<const Eigen::MatrixXd>&>(&SteadyStateKalmanFilter),
      py::arg("A"), py::arg("C"), py::arg("W"), py::arg("V"),
      doc.SteadyStateKalmanFilter.doc_ACWV);

  m.def("SteadyStateKalmanFilter",
      py::overload_cast<std::unique_ptr<systems::LinearSystem<double>>,
          const Eigen::Ref<const Eigen::MatrixXd>&,
          const Eigen::Ref<const Eigen::MatrixXd>&>(&SteadyStateKalmanFilter),
      py::arg("system"), py::arg("W"), py::arg("V"),
      doc.SteadyStateKalmanFilter.doc_linear_system);

  m.def("SteadyStateKalmanFilter",
      py::overload_cast<std::unique_ptr<System<double>>,
          std::unique_ptr<Context<double>>,
          const Eigen::Ref<const Eigen::MatrixXd>&,
          const Eigen::Ref<const Eigen::MatrixXd>&>(&SteadyStateKalmanFilter),
      py::arg("system"), py::arg("context"), py::arg("W"), py::arg("V"),
      doc.SteadyStateKalmanFilter.doc_system);
}

}  // namespace pydrake
}  // namespace drake
