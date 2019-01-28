#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(inverse_kinematics, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody;

  m.doc() = "InverseKinematics module";

  py::module::import("pydrake.solvers.mathematicalprogram");
  py::module::import("pydrake.math");

  {
    using Class = InverseKinematics;
    constexpr auto& cls_doc = doc.InverseKinematics;
    py::class_<Class>(m, "InverseKinematics", cls_doc.doc)
        .def(py::init<const MultibodyPlant<double>&>(), py::arg("plant"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),  // BR
            cls_doc.ctor.doc_1args)
        .def(py::init<const MultibodyPlant<double>&,
                 systems::Context<double>*>(),
            py::arg("plant"), py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),  // BR
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 3>(),  // BR
            cls_doc.ctor.doc_2args)
        .def("AddPositionConstraint", &Class::AddPositionConstraint,
            py::arg("frameB"), py::arg("p_BQ"), py::arg("frameA"),
            py::arg("p_AQ_lower"), py::arg("p_AQ_upper"),
            cls_doc.AddPositionConstraint.doc)
        .def("AddOrientationConstraint", &Class::AddOrientationConstraint,
            py::arg("frameAbar"), py::arg("R_AbarA"), py::arg("frameBbar"),
            py::arg("R_BbarB"), py::arg("theta_bound"),
            cls_doc.AddOrientationConstraint.doc)
        .def("AddGazeTargetConstraint", &Class::AddGazeTargetConstraint,
            py::arg("frameA"), py::arg("p_AS"), py::arg("n_A"),
            py::arg("frameB"), py::arg("p_BT"), py::arg("cone_half_angle"),
            cls_doc.AddGazeTargetConstraint.doc)
        .def("AddAngleBetweenVectorsConstraint",
            &Class::AddAngleBetweenVectorsConstraint, py::arg("frameA"),
            py::arg("na_A"), py::arg("frameB"), py::arg("nb_B"),
            py::arg("angle_lower"), py::arg("angle_upper"),
            cls_doc.AddAngleBetweenVectorsConstraint.doc)
        .def("AddMinimumDistanceConstraint",
            &Class::AddMinimumDistanceConstraint, py::arg("minimal_distance"),
            cls_doc.AddMinimumDistanceConstraint.doc)
        .def("q", &Class::q, cls_doc.q.doc)
        .def("prog", &Class::prog, py_reference_internal, cls_doc.prog.doc)
        .def("get_mutable_prog", &Class::get_mutable_prog,
            py_reference_internal, cls_doc.get_mutable_prog.doc);
  }
}

}  // namespace pydrake
}  // namespace drake
