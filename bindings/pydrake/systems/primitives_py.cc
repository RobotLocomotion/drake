#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/bindings/pydrake/util/cpp_template_pybind.h"
#include "drake/bindings/pydrake/util/drake_optional_pybind.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/affine_system.h"
#include "drake/systems/primitives/barycentric_system.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/primitives/gain.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/random_source.h"
#include "drake/systems/primitives/saturation.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/primitives/wrap_to_system.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(primitives, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;

  m.doc() = "Bindings for the primitives portion of the Systems framework.";
  auto& doc = pydrake_doc.drake.systems;

  py::module::import("pydrake.systems.framework");

  auto bind_common_scalar_types = [m](auto dummy) {
    using T = decltype(dummy);
    auto& doc_lambda = pydrake_doc.drake.systems;

    DefineTemplateClassWithDefault<Adder<T>, LeafSystem<T>>(
        m, "Adder", GetPyParam<T>(), doc_lambda.Adder.doc)
        .def(py::init<int, int>(), py::arg("num_inputs"), py::arg("size"),
        doc_lambda.Adder.ctor.doc);

    DefineTemplateClassWithDefault<AffineSystem<T>, LeafSystem<T>>(
        m, "AffineSystem", GetPyParam<T>(), doc_lambda.AffineSystem.doc)
        .def(py::init<const Eigen::Ref<const Eigen::MatrixXd>&,
                      const Eigen::Ref<const Eigen::MatrixXd>&,
                      const Eigen::Ref<const Eigen::VectorXd>&,
                      const Eigen::Ref<const Eigen::MatrixXd>&,
                      const Eigen::Ref<const Eigen::MatrixXd>&,
                      const Eigen::Ref<const Eigen::VectorXd>&, double>(),
             py::arg("A"), py::arg("B"), py::arg("f0"), py::arg("C"),
             py::arg("D"), py::arg("y0"), py::arg("time_period") = 0.0,
             doc_lambda.AffineSystem.ctor.doc_3)
        // TODO(eric.cousineau): Fix these to return references instead of
        // copies.
        .def("A", overload_cast_explicit<const Eigen::MatrixXd&>(
            &AffineSystem<T>::A), doc_lambda.AffineSystem.A.doc)
        .def("B", overload_cast_explicit<const Eigen::MatrixXd&>(
            &AffineSystem<T>::B), doc_lambda.AffineSystem.B.doc)
        .def("f0", overload_cast_explicit<const Eigen::VectorXd&>(
            &AffineSystem<T>::f0), doc_lambda.AffineSystem.f0.doc)
        .def("C", overload_cast_explicit<const Eigen::MatrixXd&>(
            &AffineSystem<T>::C), doc_lambda.AffineSystem.C.doc)
        .def("D", overload_cast_explicit<const Eigen::MatrixXd&>(
            &AffineSystem<T>::D), doc_lambda.AffineSystem.D.doc)
        .def("y0", overload_cast_explicit<const Eigen::VectorXd&>(
            &AffineSystem<T>::y0), doc_lambda.AffineSystem.y0.doc)
        .def("time_period", &AffineSystem<T>::time_period,
            doc_lambda.TimeVaryingAffineSystem.time_period.doc);

    DefineTemplateClassWithDefault<ConstantValueSource<T>, LeafSystem<T>>(
        m, "ConstantValueSource", GetPyParam<T>(),
            doc_lambda.ConstantValueSource.doc);

    DefineTemplateClassWithDefault<ConstantVectorSource<T>, LeafSystem<T>>(
        m, "ConstantVectorSource", GetPyParam<T>(),
            doc_lambda.ConstantValueSource.doc)
        .def(py::init<VectorX<T>>(),
             doc_lambda.ConstantValueSource.ctor.doc_3);

    DefineTemplateClassWithDefault<Demultiplexer<T>, LeafSystem<T>>(
        m, "Demultiplexer", GetPyParam<T>(), doc_lambda.Demultiplexer.doc)
        .def(py::init<int, int>(),
             py::arg("size"),
             py::arg("output_ports_sizes") = 1,
             doc_lambda.Demultiplexer.ctor.doc_3);

    DefineTemplateClassWithDefault<Gain<T>, LeafSystem<T>>(
        m, "Gain", GetPyParam<T>(), doc_lambda.Gain.doc)
        .def(py::init<double, int>(), py::arg("k"), py::arg("size"),
             doc_lambda.Gain.ctor.doc_3)
        .def(py::init<const Eigen::Ref<const Eigen::VectorXd>&>(),
             py::arg("k"), doc_lambda.Gain.ctor.doc_4);

    DefineTemplateClassWithDefault<Integrator<T>, LeafSystem<T>>(
        m, "Integrator", GetPyParam<T>(), doc_lambda.Integrator.doc)
        .def(py::init<int>(), doc_lambda.Integrator.ctor.doc_3);

    DefineTemplateClassWithDefault<LinearSystem<T>, AffineSystem<T>>(
        m, "LinearSystem", GetPyParam<T>(), doc_lambda.LinearSystem.doc)
        .def(py::init<const Eigen::Ref<const Eigen::MatrixXd>&,
                      const Eigen::Ref<const Eigen::MatrixXd>&,
                      const Eigen::Ref<const Eigen::MatrixXd>&,
                      const Eigen::Ref<const Eigen::MatrixXd>&, double>(),
             py::arg("A"), py::arg("B"), py::arg("C"), py::arg("D"),
             py::arg("time_period") = 0.0, doc_lambda.LinearSystem.ctor.doc_3);

    DefineTemplateClassWithDefault<MatrixGain<T>, LinearSystem<T>>(
        m, "MatrixGain", GetPyParam<T>(), doc_lambda.MatrixGain.doc)
        .def(py::init<const Eigen::Ref<const Eigen::MatrixXd>&>(),
             py::arg("D"), doc_lambda.MatrixGain.ctor.doc_3);

    DefineTemplateClassWithDefault<Multiplexer<T>, LeafSystem<T>>(
        m, "Multiplexer", GetPyParam<T>(), doc_lambda.Multiplexer.doc)
        .def(py::init<int>(), py::arg("num_scalar_inputs"),
             doc_lambda.Multiplexer.ctor.doc_3)
        .def(py::init<std::vector<int>>(), py::arg("input_sizes"),
             doc_lambda.Multiplexer.ctor.doc_4)
        .def(py::init<const BasicVector<T>&>(), py::arg("model_vector"),
             doc_lambda.Multiplexer.ctor.doc_5);

    DefineTemplateClassWithDefault<PassThrough<T>, LeafSystem<T>>(
        m, "PassThrough", GetPyParam<T>(), doc_lambda.PassThrough.doc)
        .def(py::init<int>(), doc_lambda.PassThrough.ctor.doc_3)
        .def(py::init<const AbstractValue&>(),
             doc_lambda.PassThrough.ctor.doc_4);

    DefineTemplateClassWithDefault<Saturation<T>, LeafSystem<T>>(
        m, "Saturation", GetPyParam<T>(), doc_lambda.Saturation.doc)
        .def(py::init<const VectorX<T>&, const VectorX<T>&>(), py::arg
      ("min_value"), py::arg("max_value"), doc_lambda.Saturation.ctor.doc_3);

    DefineTemplateClassWithDefault<SignalLogger<T>, LeafSystem<T>>(
        m, "SignalLogger", GetPyParam<T>(), doc_lambda.SignalLogger.doc)
        .def(py::init<int, int>(), py::arg("input_size"),
             py::arg("batch_allocation_size") = 1000,
             doc_lambda.SignalLogger.ctor.doc_3)
        .def("sample_times", &SignalLogger<T>::sample_times,
             doc_lambda.SignalLogger.sample_times.doc)
        .def("data", &SignalLogger<T>::data, doc_lambda.SignalLogger.data.doc)
        .def("reset", &SignalLogger<T>::reset,
             doc_lambda.SignalLogger.reset.doc);

    DefineTemplateClassWithDefault<WrapToSystem<T>, LeafSystem<T>>(
        m, "WrapToSystem", GetPyParam<T>(), doc_lambda.WrapToSystem.doc)
        .def(py::init<int>(), doc_lambda.WrapToSystem.ctor.doc_3)
        .def("set_interval", &WrapToSystem<T>::set_interval,
             doc_lambda.WrapToSystem.set_interval.doc);

    DefineTemplateClassWithDefault<ZeroOrderHold<T>, LeafSystem<T>>(
        m, "ZeroOrderHold", GetPyParam<T>(), doc_lambda.ZeroOrderHold.doc)
        .def(py::init<double, int>(), doc_lambda.ZeroOrderHold.ctor.doc_3);
  };
  type_visit(bind_common_scalar_types, pysystems::CommonScalarPack{});

  py::class_<BarycentricMeshSystem<double>, LeafSystem<double>>(
      m, "BarycentricMeshSystem", doc.BarycentricMeshSystem.doc)
      .def(py::init<math::BarycentricMesh<double>,
                    const Eigen::Ref<const MatrixX<double>>&>(),
         doc.BarycentricMeshSystem.ctor.doc_3)
      .def("get_mesh", &BarycentricMeshSystem<double>::get_mesh,
         doc.BarycentricMeshSystem.get_mesh.doc)
      .def("get_output_values",
           &BarycentricMeshSystem<double>::get_output_values,
         doc.BarycentricMeshSystem.get_output_values.doc);

  // Docs for typedef not being parsed.
  py::class_<UniformRandomSource, LeafSystem<double>>(m, "UniformRandomSource")
      .def(py::init<int, double>(), py::arg("num_outputs"),
           py::arg("sampling_interval_sec"));

  // Docs for typedef not being parsed.
  py::class_<GaussianRandomSource, LeafSystem<double>>(m,
                                                       "GaussianRandomSource")
      .def(py::init<int, double>(), py::arg("num_outputs"),
           py::arg("sampling_interval_sec"));

  // Docs for typedef not being parsed.
  py::class_<ExponentialRandomSource, LeafSystem<double>>(
      m, "ExponentialRandomSource")
      .def(py::init<int, double>(), py::arg("num_outputs"),
           py::arg("sampling_interval_sec"));

  py::class_<TrajectorySource<double>, LeafSystem<double>>(
        m, "TrajectorySource", doc.TrajectorySource.doc)
        .def(py::init<const trajectories::Trajectory<double>&, int, bool>(),
          py::arg("trajectory"),
          py::arg("output_derivative_order") = 0,
          py::arg("zero_derivatives_beyond_limits") = true,
          doc.TrajectorySource.ctor.doc_3);

  m.def("AddRandomInputs", &AddRandomInputs, py::arg("sampling_interval_sec"),
        py::arg("builder"), doc.AddRandomInputs.doc);

  m.def("Linearize", &Linearize, py::arg("system"), py::arg("context"),
        py::arg("input_port_index") = systems::kUseFirstInputIfItExists,
        py::arg("output_port_index") = systems::kUseFirstOutputIfItExists,
        py::arg("equilibrium_check_tolerance") = 1e-6,
        doc.Linearize.doc);

  m.def("FirstOrderTaylorApproximation", &FirstOrderTaylorApproximation,
        py::arg("system"), py::arg("context"),
        py::arg("input_port_index") = systems::kUseFirstInputIfItExists,
        py::arg("output_port_index") = systems::kUseFirstOutputIfItExists,
        doc.FirstOrderTaylorApproximation.doc);

  m.def("ControllabilityMatrix", &ControllabilityMatrix,
    doc.ControllabilityMatrix.doc);

  m.def("IsControllable", &IsControllable, py::arg("sys"),
        py::arg("threshold") = nullopt,
        doc.IsControllable.doc);

  m.def("ObservabilityMatrix", &ObservabilityMatrix,
    doc.ObservabilityMatrix.doc);

  m.def("IsObservable", &IsObservable, py::arg("sys"),
        py::arg("threshold") = nullopt, doc.IsObservable.doc);

  m.def("LogOutput", &LogOutput<double>, py::arg("src"), py::arg("builder"),
        // Keep alive, ownership: `return` keeps `builder` alive.
        py::keep_alive<0, 2>(),
        // TODO(eric.cousineau): Figure out why this is necessary (#9398).
        py_reference, doc.LogOutput.doc);

  // TODO(eric.cousineau): Add more systems as needed.
}

}  // namespace pydrake
}  // namespace drake
