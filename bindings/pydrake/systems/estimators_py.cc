#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/estimators/extended_kalman_filter.h"
#include "drake/systems/estimators/gaussian_state_observer.h"
#include "drake/systems/estimators/kalman_filter.h"
#include "drake/systems/estimators/luenberger_observer.h"
#include "drake/systems/estimators/unscented_kalman_filter.h"

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

    DefineTemplateClassWithDefault<GaussianStateObserver<T>, LeafSystem<T>>(m,
        "GaussianStateObserver", GetPyParam<T>(), doc.GaussianStateObserver.doc)
        .def("get_observed_system_input_input_port",
            &GaussianStateObserver<T>::get_observed_system_input_input_port,
            py_rvp::reference_internal,
            doc.GaussianStateObserver.get_observed_system_input_input_port.doc)
        .def("get_observed_system_output_input_port",
            &GaussianStateObserver<T>::get_observed_system_output_input_port,
            py_rvp::reference_internal,
            doc.GaussianStateObserver.get_observed_system_output_input_port.doc)
        .def("get_estimated_state_output_port",
            &GaussianStateObserver<T>::get_estimated_state_output_port,
            py_rvp::reference_internal,
            doc.GaussianStateObserver.get_estimated_state_output_port.doc)
        .def("SetStateEstimateAndCovariance",
            &GaussianStateObserver<T>::SetStateEstimateAndCovariance,
            py::arg("context"), py::arg("state_estimate"),
            py::arg("state_covariance"),
            doc.GaussianStateObserver.SetStateEstimateAndCovariance.doc)
        .def("GetStateEstimate", &GaussianStateObserver<T>::GetStateEstimate,
            py::arg("context"), doc.GaussianStateObserver.GetStateEstimate.doc)
        .def("GetStateCovariance",
            &GaussianStateObserver<T>::GetStateCovariance, py::arg("context"),
            doc.GaussianStateObserver.GetStateCovariance.doc);
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

  {
    using Class = ExtendedKalmanFilterOptions;
    constexpr auto& cls_doc = doc.ExtendedKalmanFilterOptions;
    py::class_<ExtendedKalmanFilterOptions> cls(
        m, "ExtendedKalmanFilterOptions", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc)
        .def_readwrite("initial_state_estimate", &Class::initial_state_estimate,
            cls_doc.initial_state_estimate.doc)
        .def_readwrite("initial_state_covariance",
            &Class::initial_state_covariance,
            cls_doc.initial_state_covariance.doc)
        .def_readwrite("actuation_input_port_index",
            &Class::actuation_input_port_index,
            cls_doc.actuation_input_port_index.doc)
        .def_readwrite("measurement_output_port_index",
            &Class::measurement_output_port_index,
            cls_doc.measurement_output_port_index.doc)
        .def_readwrite("process_noise_input_port_index",
            &Class::process_noise_input_port_index,
            cls_doc.process_noise_input_port_index.doc)
        .def_readwrite("measurement_noise_input_port_index",
            &Class::measurement_noise_input_port_index,
            cls_doc.measurement_noise_input_port_index.doc)
        .def_readwrite("use_square_root_method", &Class::use_square_root_method,
            cls_doc.use_square_root_method.doc)
        .def_readwrite("discrete_measurement_time_period",
            &Class::discrete_measurement_time_period,
            cls_doc.discrete_measurement_time_period.doc)
        .def_readwrite("discrete_measurement_time_offset",
            &Class::discrete_measurement_time_offset,
            cls_doc.discrete_measurement_time_offset.doc);

    m.def(
        "ExtendedKalmanFilter",
        [](const System<AutoDiffXd>& observed_system,
            const Context<AutoDiffXd>& observed_system_context,
            const Eigen::Ref<const Eigen::MatrixXd>& W,
            const Eigen::Ref<const Eigen::MatrixXd>& V,
            const ExtendedKalmanFilterOptions& options) {
          return ExtendedKalmanFilter(
              // The lifetime of `observed_system` is managed by the keep_alive
              // below, not the C++ shared_ptr.
              make_unowned_shared_ptr_from_raw(&observed_system),
              observed_system_context, W, V, options);
        },
        py::arg("observed_system"), py::arg("observed_system_context"),
        py::arg("W"), py::arg("V"),
        py::arg("options") = ExtendedKalmanFilterOptions(),
        // Keep alive, reference: `result` keeps `observed_system` alive.
        py::keep_alive<0, 1>(), doc.ExtendedKalmanFilter.doc_System_AutoDiffXd);

    m.def("ExtendedKalmanFilter",
        py::overload_cast<const System<double>&, const Context<double>&,
            const Eigen::Ref<const Eigen::MatrixXd>&,
            const Eigen::Ref<const Eigen::MatrixXd>&,
            const ExtendedKalmanFilterOptions&>(&ExtendedKalmanFilter),
        py::arg("observed_system"), py::arg("observed_system_context"),
        py::arg("W"), py::arg("V"),
        py::arg("options") = ExtendedKalmanFilterOptions(),
        doc.ExtendedKalmanFilter.doc_System_double);
  }

  {
    using Class = UnscentedKalmanFilterOptions;
    constexpr auto& cls_doc = doc.UnscentedKalmanFilterOptions;
    py::class_<UnscentedKalmanFilterOptions> cls(
        m, "UnscentedKalmanFilterOptions", cls_doc.doc);
    {
      using Nested = Class::UnscentedTransformParameters;
      constexpr auto& nested_doc = cls_doc.UnscentedTransformParameters;
      py::class_<Nested> nested(
          cls, "UnscentedTransformParameters", nested_doc.doc);
      nested  // BR
          .def(py::init<double, double,
                   std::variant<double, std::function<double(int)>>>(),
              py::arg("alpha") = 1.0, py::arg("beta") = 2.0,
              py::arg("kappa") = 0.0)
          .def_readwrite("alpha", &Nested::alpha, nested_doc.alpha.doc)
          .def_readwrite("beta", &Nested::beta, nested_doc.beta.doc)
          .def_readwrite("kappa", &Nested::kappa, nested_doc.kappa.doc);
    }
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc)
        .def_readwrite("initial_state_estimate", &Class::initial_state_estimate,
            cls_doc.initial_state_estimate.doc)
        .def_readwrite("initial_state_covariance",
            &Class::initial_state_covariance,
            cls_doc.initial_state_covariance.doc)
        .def_readwrite("actuation_input_port_index",
            &Class::actuation_input_port_index,
            cls_doc.actuation_input_port_index.doc)
        .def_readwrite("measurement_output_port_index",
            &Class::measurement_output_port_index,
            cls_doc.measurement_output_port_index.doc)
        .def_readwrite("process_noise_input_port_index",
            &Class::process_noise_input_port_index,
            cls_doc.process_noise_input_port_index.doc)
        .def_readwrite("measurement_noise_input_port_index",
            &Class::measurement_noise_input_port_index,
            cls_doc.measurement_noise_input_port_index.doc)
        .def_readwrite("unscented_transform_parameters",
            &Class::unscented_transform_parameters,
            cls_doc.unscented_transform_parameters.doc)
        .def_readwrite("discrete_measurement_time_period",
            &Class::discrete_measurement_time_period,
            cls_doc.discrete_measurement_time_period.doc)
        .def_readwrite("discrete_measurement_time_offset",
            &Class::discrete_measurement_time_offset,
            cls_doc.discrete_measurement_time_offset.doc);

    m.def(
        "UnscentedKalmanFilter",
        [](const System<double>& observed_system,
            const Context<double>& observed_system_context,
            const Eigen::Ref<const Eigen::MatrixXd>& W,
            const Eigen::Ref<const Eigen::MatrixXd>& V,
            const UnscentedKalmanFilterOptions& options) {
          return UnscentedKalmanFilter(
              // The lifetime of `observed_system` is managed by the keep_alive
              // below, not the C++ shared_ptr.
              make_unowned_shared_ptr_from_raw(&observed_system),
              observed_system_context, W, V, options);
        },
        py::arg("observed_system"), py::arg("observed_system_context"),
        py::arg("W"), py::arg("V"),
        py::arg("options") = UnscentedKalmanFilterOptions(),
        // Keep alive, reference: `result` keeps `observed_system` alive.
        py::keep_alive<0, 1>(), doc.UnscentedKalmanFilter.doc);
  }
}

}  // namespace pydrake
}  // namespace drake
