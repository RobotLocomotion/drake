#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/examples/quadrotor/quadrotor_geometry.h"
#include "drake/examples/quadrotor/quadrotor_plant.h"
#include "drake/systems/primitives/affine_system.h"

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
      .def("g", &QuadrotorPlant<T>::g, doc.QuadrotorPlant.g.doc);

  py::class_<QuadrotorGeometry, LeafSystem<double>>(
      m, "QuadrotorGeometry", doc.QuadrotorGeometry.doc)
      .def("get_frame_id", &QuadrotorGeometry::get_frame_id,
          doc.QuadrotorGeometry.get_frame_id.doc)
      .def_static("AddToBuilder", &QuadrotorGeometry::AddToBuilder,
          py::arg("builder"), py::arg("quadrotor_state_port"),
          py::arg("scene_graph"), py::return_value_policy::reference,
          // Keep alive, ownership: `return` keeps `builder` alive.
          py::keep_alive<0, 1>(), doc.QuadrotorGeometry.AddToBuilder.doc);

  m.def("StabilizingLQRController", &StabilizingLQRController,
      py::arg("quadrotor_plant"), py::arg("nominal_position"),
      doc.StabilizingLQRController.doc);
}

}  // namespace pydrake
}  // namespace drake
