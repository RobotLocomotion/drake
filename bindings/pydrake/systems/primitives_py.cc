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
#include "drake/systems/primitives/first_order_low_pass_filter.h"
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

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(primitives, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;

  m.doc() = "Bindings for the primitives portion of the Systems framework.";
  constexpr auto& doc = pydrake_doc.drake.systems;

  py::module::import("pydrake.systems.framework");
  // N.B. Capturing `&doc` should not be required; workaround per #9600.
  auto bind_common_scalar_types = [m, &doc](auto dummy) {
    using T = decltype(dummy);

    DefineTemplateClassWithDefault<Adder<T>, LeafSystem<T>>(
        m, "Adder", GetPyParam<T>(), doc.Adder.doc)
        .def(py::init<int, int>(), py::arg("num_inputs"), py::arg("size"),
            doc.Adder.ctor.doc);

    DefineTemplateClassWithDefault<AffineSystem<T>, LeafSystem<T>>(
        m, "AffineSystem", GetPyParam<T>(), doc.AffineSystem.doc)
        .def(py::init<const Eigen::Ref<const MatrixXd>&,
                 const Eigen::Ref<const MatrixXd>&,
                 const Eigen::Ref<const VectorXd>&,
                 const Eigen::Ref<const MatrixXd>&,
                 const Eigen::Ref<const MatrixXd>&,
                 const Eigen::Ref<const VectorXd>&, double>(),
            py::arg("A"), py::arg("B"), py::arg("f0"), py::arg("C"),
            py::arg("D"), py::arg("y0"), py::arg("time_period") = 0.0,
            doc.AffineSystem.ctor.doc_7args)
        // TODO(eric.cousineau): Fix these to return references instead of
        // copies.
        .def("A", overload_cast_explicit<const MatrixXd&>(&AffineSystem<T>::A),
            doc.AffineSystem.A.doc_0args)
        .def("B", overload_cast_explicit<const MatrixXd&>(&AffineSystem<T>::B),
            doc.AffineSystem.B.doc)
        .def("f0",
            overload_cast_explicit<const VectorXd&>(&AffineSystem<T>::f0),
            doc.AffineSystem.f0.doc)
        .def("C", overload_cast_explicit<const MatrixXd&>(&AffineSystem<T>::C),
            doc.AffineSystem.C.doc)
        .def("D", overload_cast_explicit<const MatrixXd&>(&AffineSystem<T>::D),
            doc.AffineSystem.D.doc)
        .def("y0",
            overload_cast_explicit<const VectorXd&>(&AffineSystem<T>::y0),
            doc.AffineSystem.y0.doc)
        .def("time_period", &AffineSystem<T>::time_period,
            doc.TimeVaryingAffineSystem.time_period.doc);

    DefineTemplateClassWithDefault<ConstantValueSource<T>, LeafSystem<T>>(
        m, "ConstantValueSource", GetPyParam<T>(), doc.ConstantValueSource.doc)
        .def(py::init<const AbstractValue&>(), py::arg("value"),
            doc.ConstantValueSource.ctor.doc);

    DefineTemplateClassWithDefault<ConstantVectorSource<T>, LeafSystem<T>>(
        m, "ConstantVectorSource", GetPyParam<T>(), doc.ConstantValueSource.doc)
        .def(py::init<VectorX<T>>(), py::arg("source_value"),
            doc.ConstantValueSource.ctor.doc);

    DefineTemplateClassWithDefault<Demultiplexer<T>, LeafSystem<T>>(
        m, "Demultiplexer", GetPyParam<T>(), doc.Demultiplexer.doc)
        .def(py::init<int, int>(), py::arg("size"),
            py::arg("output_ports_sizes") = 1, doc.Demultiplexer.ctor.doc);

    DefineTemplateClassWithDefault<                  // BR
        FirstOrderLowPassFilter<T>, LeafSystem<T>>(  //
        m, "FirstOrderLowPassFilter", GetPyParam<T>(),
        doc.FirstOrderLowPassFilter.doc)
        .def(py::init<double, int>(), py::arg("time_constant"),
            py::arg("size") = 1, doc.FirstOrderLowPassFilter.ctor.doc_2args)
        .def(py::init<const VectorX<double>&>(), py::arg("time_constants"),
            doc.FirstOrderLowPassFilter.ctor.doc_1args)
        .def("get_time_constant",
            &FirstOrderLowPassFilter<T>::get_time_constant,
            doc.FirstOrderLowPassFilter.get_time_constant.doc)
        .def("get_time_constants_vector",
            &FirstOrderLowPassFilter<T>::get_time_constants_vector,
            doc.FirstOrderLowPassFilter.get_time_constants_vector.doc)
        .def("set_initial_output_value",
            &FirstOrderLowPassFilter<T>::set_initial_output_value,
            doc.FirstOrderLowPassFilter.set_initial_output_value.doc);

    DefineTemplateClassWithDefault<Gain<T>, LeafSystem<T>>(
        m, "Gain", GetPyParam<T>(), doc.Gain.doc)
        .def(py::init<double, int>(), py::arg("k"), py::arg("size"),
            doc.Gain.ctor.doc_2args)
        .def(py::init<const Eigen::Ref<const VectorXd>&>(), py::arg("k"),
            doc.Gain.ctor.doc_1args);

    DefineTemplateClassWithDefault<Integrator<T>, LeafSystem<T>>(
        m, "Integrator", GetPyParam<T>(), doc.Integrator.doc)
        .def(py::init<int>(), doc.Integrator.ctor.doc);

    DefineTemplateClassWithDefault<LinearSystem<T>, AffineSystem<T>>(
        m, "LinearSystem", GetPyParam<T>(), doc.LinearSystem.doc)
        .def(py::init<const Eigen::Ref<const MatrixXd>&,
                 const Eigen::Ref<const MatrixXd>&,
                 const Eigen::Ref<const MatrixXd>&,
                 const Eigen::Ref<const MatrixXd>&, double>(),
            py::arg("A"), py::arg("B"), py::arg("C"), py::arg("D"),
            py::arg("time_period") = 0.0, doc.LinearSystem.ctor.doc_5args);

    DefineTemplateClassWithDefault<MatrixGain<T>, LinearSystem<T>>(
        m, "MatrixGain", GetPyParam<T>(), doc.MatrixGain.doc)
        .def(py::init<const Eigen::Ref<const MatrixXd>&>(), py::arg("D"),
            doc.MatrixGain.ctor.doc_1args_D);

    DefineTemplateClassWithDefault<Multiplexer<T>, LeafSystem<T>>(
        m, "Multiplexer", GetPyParam<T>(), doc.Multiplexer.doc)
        .def(py::init<int>(), py::arg("num_scalar_inputs"),
            doc.Multiplexer.ctor.doc_1args_num_scalar_inputs)
        .def(py::init<std::vector<int>>(), py::arg("input_sizes"),
            doc.Multiplexer.ctor.doc_1args_input_sizes)
        .def(py::init<const BasicVector<T>&>(), py::arg("model_vector"),
            doc.Multiplexer.ctor.doc_1args_model_vector);

    DefineTemplateClassWithDefault<PassThrough<T>, LeafSystem<T>>(
        m, "PassThrough", GetPyParam<T>(), doc.PassThrough.doc)
        .def(py::init<int>(), doc.PassThrough.ctor.doc_1args_vector_size)
        .def(py::init<const AbstractValue&>(),
            doc.PassThrough.ctor.doc_1args_abstract_model_value);

    DefineTemplateClassWithDefault<Saturation<T>, LeafSystem<T>>(
        m, "Saturation", GetPyParam<T>(), doc.Saturation.doc)
        .def(py::init<const VectorX<T>&, const VectorX<T>&>(),
            py::arg("min_value"), py::arg("max_value"),
            doc.Saturation.ctor.doc_2args);

    DefineTemplateClassWithDefault<SignalLogger<T>, LeafSystem<T>>(
        m, "SignalLogger", GetPyParam<T>(), doc.SignalLogger.doc)
        .def(py::init<int, int>(), py::arg("input_size"),
            py::arg("batch_allocation_size") = 1000, doc.SignalLogger.ctor.doc)
        .def("sample_times", &SignalLogger<T>::sample_times,
            doc.SignalLogger.sample_times.doc)
        .def("data", &SignalLogger<T>::data, doc.SignalLogger.data.doc)
        .def("reset", &SignalLogger<T>::reset, doc.SignalLogger.reset.doc);

    DefineTemplateClassWithDefault<WrapToSystem<T>, LeafSystem<T>>(
        m, "WrapToSystem", GetPyParam<T>(), doc.WrapToSystem.doc)
        .def(py::init<int>(), doc.WrapToSystem.ctor.doc)
        .def("set_interval", &WrapToSystem<T>::set_interval,
            doc.WrapToSystem.set_interval.doc);

    DefineTemplateClassWithDefault<ZeroOrderHold<T>, LeafSystem<T>>(
        m, "ZeroOrderHold", GetPyParam<T>(), doc.ZeroOrderHold.doc)
        .def(py::init<double, int>(), py::arg("period_sec"),
            py::arg("vector_size"),
            doc.ZeroOrderHold.ctor.doc_2args_period_sec_vector_size)
        .def(py::init<double, const AbstractValue&>(), py::arg("period_sec"),
            py::arg("abstract_model_value"),
            doc.ZeroOrderHold.ctor.doc_2args_period_sec_abstract_model_value);
  };
  type_visit(bind_common_scalar_types, pysystems::CommonScalarPack{});

  py::class_<BarycentricMeshSystem<double>, LeafSystem<double>>(
      m, "BarycentricMeshSystem", doc.BarycentricMeshSystem.doc)
      .def(py::init<math::BarycentricMesh<double>,
               const Eigen::Ref<const MatrixX<double>>&>(),
          doc.BarycentricMeshSystem.ctor.doc)
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
  py::class_<GaussianRandomSource, LeafSystem<double>>(
      m, "GaussianRandomSource")
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
          py::arg("trajectory"), py::arg("output_derivative_order") = 0,
          py::arg("zero_derivatives_beyond_limits") = true,
          doc.TrajectorySource.ctor.doc);

  m.def("AddRandomInputs", &AddRandomInputs, py::arg("sampling_interval_sec"),
      py::arg("builder"), doc.AddRandomInputs.doc);

  m.def("Linearize", &Linearize, py::arg("system"), py::arg("context"),
      py::arg("input_port_index") = systems::kUseFirstInputIfItExists,
      py::arg("output_port_index") = systems::kUseFirstOutputIfItExists,
      py::arg("equilibrium_check_tolerance") = 1e-6, doc.Linearize.doc);

  m.def("FirstOrderTaylorApproximation", &FirstOrderTaylorApproximation,
      py::arg("system"), py::arg("context"),
      py::arg("input_port_index") = systems::kUseFirstInputIfItExists,
      py::arg("output_port_index") = systems::kUseFirstOutputIfItExists,
      doc.FirstOrderTaylorApproximation.doc);

  m.def("ControllabilityMatrix", &ControllabilityMatrix,
      doc.ControllabilityMatrix.doc);

  m.def("IsControllable", &IsControllable, py::arg("sys"),
      py::arg("threshold") = nullopt, doc.IsControllable.doc);

  m.def(
      "ObservabilityMatrix", &ObservabilityMatrix, doc.ObservabilityMatrix.doc);

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
