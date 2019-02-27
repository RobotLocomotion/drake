#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/examples/quadrotor/quadrotor_plant.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(quadrotor, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::examples::quadrotor;
  constexpr auto& doc = pydrake_doc.drake.examples.quadrotor;

  m.doc() = "Bindings for the Quadrotor example.";

  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.primitives");

  // TODO(eric.cousineau): At present, we only bind doubles.
  // In the future, we will bind more scalar types, and enable scalar
  // conversion. Issue #7660.
  using T = double;

  py::class_<QuadrotorPlant<T>, LeafSystem<T>>(
      m, "QuadrotorPlant", doc.QuadrotorPlant.doc)
      .def(py::init<>(), doc.QuadrotorPlant.ctor.doc)
      .def(py::init<double, double, const Eigen::Matrix3d&, double, double>(),
          py::arg("m_arg"), py::arg("L_arg"), py::arg("I_arg"),
          py::arg("kF_arg"), py::arg("kM_arg"), doc.QuadrotorPlant.ctor.doc)
      .def("m", &QuadrotorPlant<T>::m, doc.QuadrotorPlant.m.doc)
      .def("g", &QuadrotorPlant<T>::g, doc.QuadrotorPlant.g.doc)
      .def("RegisterGeometry", &QuadrotorPlant<T>::RegisterGeometry,
          py::arg("scene_graph"), doc.QuadrotorPlant.RegisterGeometry.doc)
      .def("get_geometry_pose_output_port",
          &QuadrotorPlant<T>::get_geometry_pose_output_port,
          py_reference_internal,
          doc.QuadrotorPlant.get_geometry_pose_output_port.doc)
      .def("source_id", &QuadrotorPlant<T>::source_id,
          doc.QuadrotorPlant.source_id.doc);

  m.def("StabilizingLQRController", &StabilizingLQRController,
      py::arg("quadrotor_plant"), py::arg("nominal_position"),
      doc.StabilizingLQRController.doc);
}

}  // namespace pydrake
}  // namespace drake
