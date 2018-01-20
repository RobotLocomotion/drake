#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(_ik_py, m) {
  m.doc() = "RigidBodyTree inverse kinematics";

  py::class_<RigidBodyConstraint>(m, "RigidBodyConstraint");

  py::class_<PostureConstraint, RigidBodyConstraint>(m, "PostureConstraint")
    .def(py::init<RigidBodyTree<double> *,
                  const Eigen::Vector2d& >(),
         py::arg("model"),
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan)
    .def("setJointLimits",
         static_cast<void(PostureConstraint::*)(
             const Eigen::VectorXi&,
             const Eigen::VectorXd&,
             const Eigen::VectorXd&)>(
                 &PostureConstraint::setJointLimits));

  py::class_<WorldPositionConstraint, RigidBodyConstraint>(
    m, "WorldPositionConstraint")
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
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan);

  py::class_<RelativePositionConstraint, RigidBodyConstraint>(
    m, "RelativePositionConstraint")
    .def(py::init<RigidBodyTree<double>*,
                  const Eigen::Matrix3Xd&,
                  const Eigen::MatrixXd&,
                  const Eigen::MatrixXd&,
                  int,
                  int,
                  const Eigen::Matrix<double, 7, 1>&,
                  const Eigen::Vector2d&>(),
         py::arg("model"),
         py::arg("pts"),
         py::arg("lb"),
         py::arg("ub"),
         py::arg("bodyA_idx"),
         py::arg("bodyB_idx"),
         py::arg("bTbp"),
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan);

  py::class_<RelativeQuatConstraint, RigidBodyConstraint>(
    m, "RelativeQuatConstraint")
    .def(py::init<RigidBodyTree<double>*,
                  int,
                  int,
                  const Eigen::Vector4d&,
                  double,
                  const Eigen::Vector2d&>(),
         py::arg("model"),
         py::arg("bodyA_idx"),
         py::arg("bodyB_idx"),
         py::arg("quat_des"),
         py::arg("tol"),
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan);

  py::class_<WorldPositionInFrameConstraint, RigidBodyConstraint>(
    m, "WorldPositionInFrameConstraint")
    .def(py::init<RigidBodyTree<double>*,
                  int,
                  const Eigen::Matrix3Xd&,
                  const Eigen::Matrix4d&,
                  const Eigen::MatrixXd&,
                  const Eigen::MatrixXd&,
                  const Eigen::Vector2d&>(),
         py::arg("model"),
         py::arg("body"),
         py::arg("pts"),
         py::arg("T_world_to_frame"),
         py::arg("lb"),
         py::arg("ub"),
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan);

  py::class_<WorldGazeDirConstraint, RigidBodyConstraint>(
    m, "WorldGazeDirConstraint")
    .def(py::init<RigidBodyTree<double>*,
                  int,
                  const Eigen::Vector3d&,
                  const Eigen::Vector3d&,
                  double,
                  const Eigen::Vector2d&>(),
         py::arg("model"),
         py::arg("body"),
         py::arg("axis"),
         py::arg("dir"),
         py::arg("conethreshold"),
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan);

  py::class_<WorldGazeTargetConstraint, RigidBodyConstraint>(
    m, "WorldGazeTargetConstraint")
    .def(py::init<RigidBodyTree<double>*,
                  int,
                  const Eigen::Vector3d&,
                  const Eigen::Vector3d&,
                  const Eigen::Vector3d&,
                  double,
                  const Eigen::Vector2d&>(),
        py::arg("model"),
        py::arg("body"),
        py::arg("axis"),
        py::arg("target"),
        py::arg("gaze_origin"),
        py::arg("conethreshold"),
        py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan);

  py::class_<RelativeGazeDirConstraint, RigidBodyConstraint>(
    m, "RelativeGazeDirConstraint")
    .def(py::init<RigidBodyTree<double>*,
                  int,
                  int,
                  const Eigen::Vector3d&,
                  const Eigen::Vector3d&,
                  double,
                  const Eigen::Vector2d&>(),
         py::arg("model"),
         py::arg("bodyA_idx"),
         py::arg("bodyB_idx"),
         py::arg("axis"),
         py::arg("dir"),
         py::arg("conethreshold"),
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan);

  py::class_<MinDistanceConstraint, RigidBodyConstraint>(
    m, "MinDistanceConstraint")
    .def(py::init<RigidBodyTree<double>*,
                  double,
                  const std::vector<int>&,
                  const std::set<std::string>&,
                  const Eigen::Vector2d&>(),
         py::arg("model"),
         py::arg("min_distance"),
         py::arg("active_bodies_idx"),
         py::arg("active_group_names"),
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan);

  py::class_<WorldEulerConstraint, RigidBodyConstraint>(
    m, "WorldEulerConstraint")
    .def(py::init<RigidBodyTree<double>*,
                  int,
                  const Eigen::Vector3d&,
                  const Eigen::Vector3d&,
                  const Eigen::Vector2d>(),
         py::arg("model"),
         py::arg("body"),
         py::arg("lb"),
         py::arg("ub"),
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan);

  py::class_<WorldQuatConstraint, RigidBodyConstraint>(
    m, "WorldQuatConstraint")
    .def(py::init<RigidBodyTree<double>*,
                  int,
                  const Eigen::Vector4d&,
                  double,
                  const Eigen::Vector2d>(),
         py::arg("model"),
         py::arg("body"),
         py::arg("quat_des"),
         py::arg("tol"),
         py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan);

  py::class_<QuasiStaticConstraint, RigidBodyConstraint>(
    m, "QuasiStaticConstraint")
    .def("__init__",
         [](QuasiStaticConstraint& instance,
            RigidBodyTree<double>* model,
            const Eigen::Vector2d& tspan) {
            new (&instance) QuasiStaticConstraint(model, tspan);
          },
          py::arg("model"),
          py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan)
    .def(py::init<RigidBodyTree<double>*,
                  const Eigen::Vector2d&,
                  const std::set<int>& >())
    .def("setActive", &QuasiStaticConstraint::setActive)
    .def("bounds", &QuasiStaticConstraint::bounds)
    .def("setShrinkFactor", &QuasiStaticConstraint::setShrinkFactor)
    .def("addContact",
         static_cast<void(QuasiStaticConstraint::*)(
             std::vector<int>, const Eigen::Matrix3Xd&)>(
                 &QuasiStaticConstraint::addContact));

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
    .def("setMajorFeasibilityTolerance",
         &IKoptions::setMajorFeasibilityTolerance)
    .def("getMajorFeasibilityTolerance",
         &IKoptions::getMajorFeasibilityTolerance)
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
    .def("getAdditionaltSamples", &IKoptions::getAdditionaltSamples);

  m.def("InverseKin",
        static_cast<IKResults(*)(
            RigidBodyTree<double>*,
            const Eigen::VectorXd&,
            const Eigen::VectorXd&,
            const std::vector<RigidBodyConstraint*>&,
            const IKoptions&)>(
                &inverseKinSimple));

  m.def("InverseKinPointwise",
        static_cast<IKResults(*)(
            RigidBodyTree<double>*,
            const Eigen::VectorXd&,
            const Eigen::MatrixXd&,
            const Eigen::MatrixXd&,
            const std::vector<RigidBodyConstraint*>&,
            const IKoptions&)>(
                &inverseKinPointwiseSimple));

  m.def("InverseKinTraj", &inverseKinTrajSimple);

  py::class_<IKResults>(m, "IKResults")
    .def_readonly("q_sol", &IKResults::q_sol)
    .def_readonly("info", &IKResults::info)
    .def_readonly("infeasible_constraints", &IKResults::infeasible_constraints);
}

}  // namespace pydrake
}  // namespace drake
