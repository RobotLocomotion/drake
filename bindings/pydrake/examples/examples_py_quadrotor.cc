#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/examples/examples_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/examples/quadrotor/quadrotor_geometry.h"
#include "drake/examples/quadrotor/quadrotor_plant.h"
#include "drake/systems/primitives/affine_system.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineExamplesQuadrotor(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::examples::quadrotor;
  constexpr auto& doc = pydrake_doc.drake.examples.quadrotor;

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
      .def("length", &QuadrotorPlant<T>::length, doc.QuadrotorPlant.length.doc)
      .def("force_constant", &QuadrotorPlant<T>::force_constant,
          doc.QuadrotorPlant.force_constant.doc)
      .def("moment_constant", &QuadrotorPlant<T>::moment_constant,
          doc.QuadrotorPlant.moment_constant.doc)
      .def("inertia", &QuadrotorPlant<T>::inertia, py_rvp::reference_internal,
          doc.QuadrotorPlant.inertia.doc);

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

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
