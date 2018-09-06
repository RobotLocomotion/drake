#include "pybind11/eigen.h"
#include "pybind11/eval.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"

namespace drake {
namespace pydrake {

using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;
using multibody::InverseKinematics;
using multibody::multibody_plant::MultibodyPlant;
using multibody::Frame;

void init_module(py::module m) {
  m.doc() = "InverseKinematics module";

  py::object variable = py::module::import("pydrake.symbolic").attr("Variable");

  py::class_<InverseKinematics> ik_cls(m, "InverseKinematics");
  ik_cls.def(py::init<const MultibodyPlant<double>&>(), py::arg("plant"))
      .def("AddPositionConstraint",
           static_cast<Binding<Constraint> (InverseKinematics::*)(
               const Frame<double>&, const Eigen::Ref<const Eigen::Vector3d>&,
               const Frame<double>&, const Eigen::Ref<const Eigen::Vector3d>&,
               const Eigen::Ref<const Eigen::Vector3d>&)>(
               &InverseKinematics::AddPositionConstraint),
           py::arg("frameB"), py::arg("p_BQ"), py::arg("frameA"),
           py::arg("p_AQ_lower"), py::arg("p_AQ_upper"))
      .def("AddOrientationConstraint",
           static_cast<Binding<Constraint> (InverseKinematics::*)(
               const Frame<double>&, const Frame<double>&, double)>(
               &InverseKinematics::AddOrientationConstraint),
           py::arg("frameA"), py::arg("frameB"), py::arg("theta_bound"))
      .def("prog", static_cast<const MathematicalProgram& (
                       InverseKinematics::*)() const>(&InverseKinematics::prog))
      .def("get_mutable_prog",
           static_cast<MathematicalProgram* (InverseKinematics::*)() const>(
               &InverseKinematics::get_mutable_prog))
      .def("q", static_cast<const solvers::VectorXDecisionVariable& (
                    InverseKinematics::*)() const>(&InverseKinematics::q));
}

void init_all(py::module m) {
  py::dict vars = m.attr("__dict__");
  py::exec("import pydrake.solvers.mathematicalprogram \n", py::globals(),
           vars);
}

PYBIND11_MODULE(inverse_kinematics, m) {
  init_module(m);
  init_all(m);
}

}  // namespace pydrake
}  // namespace drake
