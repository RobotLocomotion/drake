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
    .def("setQ", &IKoptions::setQ)
    .def("getQ", &IKoptions::getQ)
    .def("setQa", &IKoptions::setQa)
    .def("getQa", &IKoptions::getQa)
    .def("setQv", &IKoptions::setQv)
    .def("getQv", &IKoptions::getQv)
    .def("setDebug", &IKoptions::setDebug)
    .def("getDebug", &IKoptions::getDebug)
    .def("setSequentialSeedFlag", &IKoptions::setSequentialSeedFlag)
    .def("getSequentialSeedFlag", &IKoptions::getSequentialSeedFlag)
    .def("setMajorOptimalityTolerance", &IKoptions::setMajorOptimalityTolerance)
    .def("getMajorOptimalityTolerance", &IKoptions::getMajorOptimalityTolerance)
    .def("setMajorFeasibilityTolerance", &IKoptions::setMajorFeasibilityTolerance)
    .def("getMajorFeasibilityTolerance", &IKoptions::getMajorFeasibilityTolerance)
    .def("setSuperbasicsLimit", &IKoptions::setSuperbasicsLimit)
    .def("getSuperbasicsLimit", &IKoptions::getSuperbasicsLimit)
    .def("setMajorIterationsLimit", &IKoptions::setMajorIterationsLimit)
    .def("getMajorIterationsLimit", &IKoptions::getMajorIterationsLimit)
    .def("setIterationsLimit", &IKoptions::setIterationsLimit)
    .def("getIterationsLimit", &IKoptions::getIterationsLimit)
    .def("setFixInitialState", &IKoptions::setFixInitialState)
    .def("getFixInitialState", &IKoptions::getFixInitialState)
    .def("setq0", &IKoptions::setq0)
    .def("getq0", &IKoptions::getq0)
    .def("setqd0", &IKoptions::setqd0)
    .def("getqd0", &IKoptions::getqd0)
    .def("setqdf", &IKoptions::setqdf)
    .def("getqdf", &IKoptions::getqdf)
    .def("setAdditionaltSamples", &IKoptions::setAdditionaltSamples)
    .def("getAdditionaltSamples", &IKoptions::getAdditionaltSamples)
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
