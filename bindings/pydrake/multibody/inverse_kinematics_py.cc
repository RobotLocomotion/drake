#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(inverse_kinematics, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;

  using multibody_plant::MultibodyPlant;

  m.doc() = "InverseKinematics module";

  py::module::import("pydrake.solvers.mathematicalprogram");
  py::module::import("pydrake.math");

  py::class_<InverseKinematics> ik_cls(m, "InverseKinematics");
  ik_cls.def(py::init<const MultibodyPlant<double>&>(), py::arg("plant"))
      .def("AddPositionConstraint", &InverseKinematics::AddPositionConstraint,
           py::arg("frameB"), py::arg("p_BQ"), py::arg("frameA"),
           py::arg("p_AQ_lower"), py::arg("p_AQ_upper"))
      .def("AddOrientationConstraint",
           &InverseKinematics::AddOrientationConstraint, py::arg("frameAbar"),
           py::arg("R_AbarA"), py::arg("frameBbar"), py::arg("R_BbarB"),
           py::arg("theta_bound"))
      .def("AddGazeTargetConstraint",
           &InverseKinematics::AddGazeTargetConstraint, py::arg("frameA"),
           py::arg("p_AS"), py::arg("n_A"), py::arg("frameB"), py::arg("p_BT"),
           py::arg("cone_half_angle"))
      .def("AddAngleBetweenVectorsConstraint",
           &InverseKinematics::AddAngleBetweenVectorsConstraint,
           py::arg("frameA"), py::arg("na_A"), py::arg("frameB"),
           py::arg("nb_B"), py::arg("angle_lower"), py::arg("angle_upper"))
      .def("q", &InverseKinematics::q)
      .def("prog", &InverseKinematics::prog, py_reference_internal)
      .def("get_mutable_prog", &InverseKinematics::get_mutable_prog,
           py_reference_internal);
}

}  // namespace pydrake
}  // namespace drake
