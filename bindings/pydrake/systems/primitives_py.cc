#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/util/drake_optional_pybind.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/affine_system.h"
#include "drake/systems/primitives/barycentric_system.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/integrator.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/pass_through.h"
#include "drake/systems/primitives/saturation.h"
#include "drake/systems/primitives/signal_logger.h"
#include "drake/systems/primitives/wrap_to_system.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(primitives, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;

  m.doc() = "Bindings for the primitives portion of the Systems framework.";

  py::module::import("pydrake.systems.framework");

  using T = double;

  py::class_<Adder<T>, LeafSystem<T>>(m, "Adder").def(py::init<int, int>());

  py::class_<AffineSystem<T>, LeafSystem<T>>(m, "AffineSystem")
      .def(py::init<const Eigen::Ref<const Eigen::MatrixXd>&,
                    const Eigen::Ref<const Eigen::MatrixXd>&,
                    const Eigen::Ref<const Eigen::VectorXd>&,
                    const Eigen::Ref<const Eigen::MatrixXd>&,
                    const Eigen::Ref<const Eigen::MatrixXd>&,
                    const Eigen::Ref<const Eigen::VectorXd>&, double>(),
           py::arg("A"), py::arg("B"), py::arg("f0"), py::arg("C"),
           py::arg("D"), py::arg("y0"), py::arg("time_period") = 0.0)
      // TODO(eric.cousineau): Fix these to return references instead of copies.
      .def("A",
           overload_cast_explicit<const Eigen::MatrixXd&>(&AffineSystem<T>::A))
      .def("B",
           overload_cast_explicit<const Eigen::MatrixXd&>(&AffineSystem<T>::B))
      .def("f0",
           overload_cast_explicit<const Eigen::VectorXd&>(&AffineSystem<T>::f0))
      .def("C",
           overload_cast_explicit<const Eigen::MatrixXd&>(&AffineSystem<T>::C))
      .def("D",
           overload_cast_explicit<const Eigen::MatrixXd&>(&AffineSystem<T>::D))
      .def("y0",
           overload_cast_explicit<const Eigen::VectorXd&>(&AffineSystem<T>::y0))
      .def("time_period", &AffineSystem<T>::time_period);

  py::class_<BarycentricMeshSystem<T>, LeafSystem<T>>(m,
                                                      "BarycentricMeshSystem")
      .def(py::init<math::BarycentricMesh<T>,
                    const Eigen::Ref<const MatrixX<T>>&>())
      .def("get_mesh", &BarycentricMeshSystem<T>::get_mesh)
      .def("get_output_values", &BarycentricMeshSystem<T>::get_output_values);

  py::class_<ConstantValueSource<T>, LeafSystem<T>>(m, "ConstantValueSource");

  py::class_<ConstantVectorSource<T>, LeafSystem<T>>(m, "ConstantVectorSource")
      .def(py::init<VectorX<T>>());

  py::class_<Integrator<T>, LeafSystem<T>>(m, "Integrator")
      .def(py::init<int>());

  py::class_<LinearSystem<T>, AffineSystem<T>>(m, "LinearSystem")
      .def(py::init<const Eigen::Ref<const Eigen::MatrixXd>&,
                    const Eigen::Ref<const Eigen::MatrixXd>&,
                    const Eigen::Ref<const Eigen::MatrixXd>&,
                    const Eigen::Ref<const Eigen::MatrixXd>&, double>(),
           py::arg("A"), py::arg("B"), py::arg("C"), py::arg("D"),
           py::arg("time_period") = 0.0);

  m.def("Linearize", &Linearize, py::arg("system"), py::arg("context"),
        py::arg("input_port_index") = systems::kUseFirstInputIfItExists,
        py::arg("output_port_index") = systems::kUseFirstOutputIfItExists,
        py::arg("equilibrium_check_tolerance") = 1e-6);

  m.def("FirstOrderTaylorApproximation", &FirstOrderTaylorApproximation,
        py::arg("system"), py::arg("context"),
        py::arg("input_port_index") = systems::kUseFirstInputIfItExists,
        py::arg("output_port_index") = systems::kUseFirstOutputIfItExists);

  m.def("ControllabilityMatrix", &ControllabilityMatrix);

  m.def("IsControllable", &IsControllable, py::arg("sys"),
        py::arg("threshold") = nullopt);

  m.def("ObservabilityMatrix", &ObservabilityMatrix);

  m.def("IsObservable", &IsObservable, py::arg("sys"),
        py::arg("threshold") = nullopt);

  py::class_<PassThrough<T>, LeafSystem<T>>(m, "PassThrough")
      .def(py::init<int>())
      .def(py::init<const AbstractValue&>());

  py::class_<Saturation<T>, LeafSystem<T>>(m, "Saturation")
      .def(py::init<const VectorX<T>&, const VectorX<T>&>(), py::arg
    ("min_value"), py::arg("max_value"));

  py::class_<SignalLogger<T>, LeafSystem<T>>(m, "SignalLogger")
      .def(py::init<int>())
      .def(py::init<int, int>())
      .def("sample_times", &SignalLogger<T>::sample_times)
      .def("data", &SignalLogger<T>::data)
      .def("reset", &SignalLogger<T>::reset);

  py::class_<WrapToSystem<T>, LeafSystem<T>>(m, "WrapToSystem")
      .def(py::init<int>())
      .def("set_interval", &WrapToSystem<T>::set_interval);

  py::class_<ZeroOrderHold<T>, LeafSystem<T>>(m, "ZeroOrderHold")
      .def(py::init<double, int>());

  py::class_<Multiplexer<T>, LeafSystem<T>>(m, "Multiplexer")
      .def(py::init<int>(), py::arg("num_scalar_inputs"))
      .def(py::init<std::vector<int>>(), py::arg("input_sizes"))
      .def(py::init<const BasicVector<T>&>(), py::arg("model_vector"));

  // TODO(eric.cousineau): Add more systems as needed.
}

}  // namespace pydrake
}  // namespace drake
