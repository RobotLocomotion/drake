#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>


#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/ik_options.h"

namespace py = pybind11;

PYBIND11_PLUGIN(_ik) {
  py::module m("_ik", "RigidBodyTree inverse kinematics");

  py::class_<RigidBodyConstraint>(m, "RigidBodyConstraint");

  py::class_<PostureConstraint, RigidBodyConstraint>(m, "PostureConstraint")
    .def(py::init<RigidBodyTree<double> *,
                  const Eigen::Vector2d& >(),
         py::arg("model"), py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan)
    .def("setJointLimits", (void (PostureConstraint::*)(
      const Eigen::VectorXi&,
      const Eigen::VectorXd&,
      const Eigen::VectorXd&)) &PostureConstraint::setJointLimits)
    ;
  py::class_<WorldPositionConstraint, RigidBodyConstraint>(m, "WorldPositionConstraint")
    .def(py::init<RigidBodyTree<double>*, 
                  int, 
                  const Eigen::Matrix3Xd&, 
                  Eigen::MatrixXd,
                  Eigen::MatrixXd,
                  const Eigen::Vector2d&>(),
         py::arg("model"),
         py::arg("body"),
         py::arg("pts"),
         py::arg("lb"),
         py::arg("ub"),
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan)
    ;

  py::class_<WorldEulerConstraint, RigidBodyConstraint>(m, "WorldEulerConstraint")
    .def(py::init<RigidBodyTree<double>*,
                  int,
                  const Eigen::Vector3d&,
                  const Eigen::Vector3d&,
                  const Eigen::Vector2d>(),
         py::arg("model"),
         py::arg("body"),
         py::arg("lb"),
         py::arg("ub"),
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan)
    ;

  py::class_<IKoptions>(m, "IKoptions")
    .def(py::init<RigidBodyTree<double> *>())
    ;

  m.def("InverseKin", (IKResults (*)(
      RigidBodyTree<double>*, 
      const Eigen::VectorXd&,
      const Eigen::VectorXd&,
      const std::vector<RigidBodyConstraint*>&,
      const IKoptions&)) 
    &inverseKinSimple);

  m.def("InverseKinPointwise", (IKResults (*)(
      RigidBodyTree<double>*, 
      const Eigen::VectorXd&,
      const Eigen::MatrixXd&,
      const Eigen::MatrixXd&,
      const std::vector<RigidBodyConstraint*>&,
      const IKoptions&)) 
    &inverseKinPointwiseSimple);

  m.def("InverseKinTraj", &inverseKinTrajSimple);

  py::class_<IKResults>(m, "IKResults")
    .def_readonly("q_sol", &IKResults::q_sol)
    .def_readonly("info", &IKResults::info)
    .def_readonly("infeasible_constraints", &IKResults::infeasible_constraints)
    ;

  return m.ptr();
}
