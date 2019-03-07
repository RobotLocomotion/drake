#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/ik_options.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(ik, m) {
  m.doc() = "RigidBodyTree inverse kinematics";
  constexpr auto& doc = pydrake_doc;

  py::class_<RigidBodyConstraint>(
      m, "RigidBodyConstraint", doc.RigidBodyConstraint.doc);

  py::class_<SingleTimeKinematicConstraint, RigidBodyConstraint>(
      m, "SingleTimeKinematicConstraint")
      .def("eval",
          [](const SingleTimeKinematicConstraint& self, double t,
              KinematicsCache<double>& cache) {
            Eigen::VectorXd c;
            Eigen::MatrixXd dc;
            self.eval(&t, cache, c, dc);
            return std::pair<Eigen::VectorXd, Eigen::MatrixXd>(c, dc);
          },
          py::arg("t"), py::arg("cache"),
          doc.SingleTimeKinematicConstraint.eval.doc)
      .def("bounds",
          [](const SingleTimeKinematicConstraint& self, double t) {
            Eigen::VectorXd lb, ub;
            self.bounds(&t, lb, ub);
            return std::pair<Eigen::VectorXd, Eigen::VectorXd>(lb, ub);
          },
          py::arg("t"), doc.SingleTimeKinematicConstraint.bounds.doc);

  py::class_<PostureConstraint, RigidBodyConstraint>(
      m, "PostureConstraint", doc.PostureConstraint.doc)
      .def(py::init<RigidBodyTree<double>*, const Eigen::Vector2d&>(),
          py::arg("model"),
          py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
          doc.PostureConstraint.ctor.doc)
      .def("setJointLimits",
          static_cast<void (PostureConstraint::*)(  // NOLINT
              const Eigen::VectorXi&, const Eigen::VectorXd&,
              const Eigen::VectorXd&)>(&PostureConstraint::setJointLimits),
          doc.PostureConstraint.setJointLimits.doc);

  py::class_<WorldPositionConstraint, SingleTimeKinematicConstraint>(
      m, "_WorldPositionConstraint", doc.WorldPositionConstraint.doc)
      .def(py::init<RigidBodyTree<double>*, int, const Eigen::Matrix3Xd&,
               Eigen::MatrixXd, Eigen::MatrixXd, const Eigen::Vector2d&>(),
          py::arg("model"), py::arg("body"), py::arg("pts"), py::arg("lb"),
          py::arg("ub"),
          py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
          doc.WorldPositionConstraint.ctor.doc);

  py::class_<RelativePositionConstraint, SingleTimeKinematicConstraint>(
      m, "RelativePositionConstraint", doc.RelativePositionConstraint.doc)
      .def(py::init<RigidBodyTree<double>*, const Eigen::Matrix3Xd&,
               const Eigen::MatrixXd&, const Eigen::MatrixXd&, int, int,
               const Eigen::Matrix<double, 7, 1>&, const Eigen::Vector2d&>(),
          py::arg("model"), py::arg("pts"), py::arg("lb"), py::arg("ub"),
          py::arg("bodyA_idx"), py::arg("bodyB_idx"), py::arg("bTbp"),
          py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
          doc.RelativePositionConstraint.ctor.doc);

  py::class_<RelativeQuatConstraint, SingleTimeKinematicConstraint>(
      m, "RelativeQuatConstraint", doc.RelativeQuatConstraint.doc)
      .def(py::init<RigidBodyTree<double>*, int, int, const Eigen::Vector4d&,
               double, const Eigen::Vector2d&>(),
          py::arg("model"), py::arg("bodyA_idx"), py::arg("bodyB_idx"),
          py::arg("quat_des"), py::arg("tol"),
          py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
          doc.RelativeQuatConstraint.ctor.doc);

  py::class_<WorldPositionInFrameConstraint, SingleTimeKinematicConstraint>(m,
      "_WorldPositionInFrameConstraint", doc.WorldPositionInFrameConstraint.doc)
      .def(py::init<RigidBodyTree<double>*, int, const Eigen::Matrix3Xd&,
               const Eigen::Matrix4d&, const Eigen::MatrixXd&,
               const Eigen::MatrixXd&, const Eigen::Vector2d&>(),
          py::arg("model"), py::arg("body"), py::arg("pts"),
          py::arg("T_world_to_frame"), py::arg("lb"), py::arg("ub"),
          py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
          doc.WorldPositionInFrameConstraint.ctor.doc);

  py::class_<WorldGazeDirConstraint, SingleTimeKinematicConstraint>(
      m, "WorldGazeDirConstraint", doc.WorldGazeDirConstraint.doc)
      .def(py::init<RigidBodyTree<double>*, int, const Eigen::Vector3d&,
               const Eigen::Vector3d&, double, const Eigen::Vector2d&>(),
          py::arg("model"), py::arg("body"), py::arg("axis"), py::arg("dir"),
          py::arg("conethreshold"),
          py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
          doc.WorldGazeDirConstraint.ctor.doc);

  py::class_<WorldGazeTargetConstraint, SingleTimeKinematicConstraint>(
      m, "WorldGazeTargetConstraint", doc.WorldGazeTargetConstraint.doc)
      .def(py::init<RigidBodyTree<double>*, int, const Eigen::Vector3d&,
               const Eigen::Vector3d&, const Eigen::Vector3d&, double,
               const Eigen::Vector2d&>(),
          py::arg("model"), py::arg("body"), py::arg("axis"), py::arg("target"),
          py::arg("gaze_origin"), py::arg("conethreshold"),
          py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
          doc.WorldGazeTargetConstraint.ctor.doc);

  py::class_<RelativeGazeDirConstraint, SingleTimeKinematicConstraint>(
      m, "RelativeGazeDirConstraint", doc.RelativeGazeDirConstraint.doc)
      .def(py::init<RigidBodyTree<double>*, int, int, const Eigen::Vector3d&,
               const Eigen::Vector3d&, double, const Eigen::Vector2d&>(),
          py::arg("model"), py::arg("bodyA_idx"), py::arg("bodyB_idx"),
          py::arg("axis"), py::arg("dir"), py::arg("conethreshold"),
          py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
          doc.RelativeGazeDirConstraint.ctor.doc);

  py::class_<MinDistanceConstraint, SingleTimeKinematicConstraint>(
      m, "MinDistanceConstraint", doc.MinDistanceConstraint.doc)
      .def(py::init<RigidBodyTree<double>*, double, const std::vector<int>&,
               const std::set<std::string>&, const Eigen::Vector2d&>(),
          py::arg("model"), py::arg("min_distance"),
          py::arg("active_bodies_idx"), py::arg("active_group_names"),
          py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
          doc.MinDistanceConstraint.ctor.doc);

  py::class_<WorldEulerConstraint, SingleTimeKinematicConstraint>(
      m, "WorldEulerConstraint", doc.WorldEulerConstraint.doc)
      .def(py::init<RigidBodyTree<double>*, int, const Eigen::Vector3d&,
               const Eigen::Vector3d&, const Eigen::Vector2d>(),
          py::arg("model"), py::arg("body"), py::arg("lb"), py::arg("ub"),
          py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
          doc.WorldEulerConstraint.ctor.doc);

  py::class_<WorldQuatConstraint, SingleTimeKinematicConstraint>(
      m, "WorldQuatConstraint", doc.WorldQuatConstraint.doc)
      .def(py::init<RigidBodyTree<double>*, int, const Eigen::Vector4d&, double,
               const Eigen::Vector2d>(),
          py::arg("model"), py::arg("body"), py::arg("quat_des"),
          py::arg("tol"),
          py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
          doc.WorldQuatConstraint.ctor.doc);

  py::class_<QuasiStaticConstraint, RigidBodyConstraint>(
      m, "QuasiStaticConstraint", doc.QuasiStaticConstraint.doc)
      .def(py::init<RigidBodyTree<double>*, const Eigen::Vector2d&>(),
          py::arg("model"),
          py::arg("tspan") = DrakeRigidBodyConstraint::default_tspan,
          doc.QuasiStaticConstraint.ctor.doc)
      .def(py::init<RigidBodyTree<double>*, const Eigen::Vector2d&,
               const std::set<int>&>(),
          doc.QuasiStaticConstraint.ctor.doc)
      .def("setActive", &QuasiStaticConstraint::setActive,
          doc.QuasiStaticConstraint.setActive.doc)
      .def("bounds", &QuasiStaticConstraint::bounds,
          doc.QuasiStaticConstraint.bounds.doc)
      .def("setShrinkFactor", &QuasiStaticConstraint::setShrinkFactor,
          doc.QuasiStaticConstraint.setShrinkFactor.doc)
      .def("addContact",
          static_cast<void (QuasiStaticConstraint::*)(  // NOLINT
              std::vector<int>, const Eigen::Matrix3Xd&)>(
              &QuasiStaticConstraint::addContact),
          doc.QuasiStaticConstraint.addContact.doc);

  py::class_<IKoptions>(m, "IKoptions", doc.IKoptions.doc)
      .def(py::init<RigidBodyTree<double>*>(), doc.IKoptions.ctor.doc)
      .def("setQ", &IKoptions::setQ, doc.IKoptions.setQ.doc)
      .def("getQ", &IKoptions::getQ, doc.IKoptions.getQ.doc)
      .def("setQa", &IKoptions::setQa, doc.IKoptions.setQa.doc)
      .def("getQa", &IKoptions::getQa, doc.IKoptions.getQa.doc)
      .def("setQv", &IKoptions::setQv, doc.IKoptions.setQv.doc)
      .def("getQv", &IKoptions::getQv, doc.IKoptions.getQv.doc)
      .def("setDebug", &IKoptions::setDebug, doc.IKoptions.setDebug.doc)
      .def("getDebug", &IKoptions::getDebug, doc.IKoptions.getDebug.doc)
      .def("setSequentialSeedFlag", &IKoptions::setSequentialSeedFlag,
          doc.IKoptions.setSequentialSeedFlag.doc)
      .def("getSequentialSeedFlag", &IKoptions::getSequentialSeedFlag,
          doc.IKoptions.getSequentialSeedFlag.doc)
      .def("setMajorOptimalityTolerance",
          &IKoptions::setMajorOptimalityTolerance,
          doc.IKoptions.setMajorOptimalityTolerance.doc)
      .def("getMajorOptimalityTolerance",
          &IKoptions::getMajorOptimalityTolerance,
          doc.IKoptions.getMajorOptimalityTolerance.doc)
      .def("setMajorFeasibilityTolerance",
          &IKoptions::setMajorFeasibilityTolerance,
          doc.IKoptions.setMajorFeasibilityTolerance.doc)
      .def("getMajorFeasibilityTolerance",
          &IKoptions::getMajorFeasibilityTolerance,
          doc.IKoptions.getMajorFeasibilityTolerance.doc)
      .def("setSuperbasicsLimit", &IKoptions::setSuperbasicsLimit,
          doc.IKoptions.setSuperbasicsLimit.doc)
      .def("getSuperbasicsLimit", &IKoptions::getSuperbasicsLimit,
          doc.IKoptions.getSuperbasicsLimit.doc)
      .def("setMajorIterationsLimit", &IKoptions::setMajorIterationsLimit,
          doc.IKoptions.setMajorIterationsLimit.doc)
      .def("getMajorIterationsLimit", &IKoptions::getMajorIterationsLimit,
          doc.IKoptions.getMajorIterationsLimit.doc)
      .def("setIterationsLimit", &IKoptions::setIterationsLimit,
          doc.IKoptions.setIterationsLimit.doc)
      .def("getIterationsLimit", &IKoptions::getIterationsLimit,
          doc.IKoptions.getIterationsLimit.doc)
      .def("setFixInitialState", &IKoptions::setFixInitialState,
          doc.IKoptions.setFixInitialState.doc)
      .def("getFixInitialState", &IKoptions::getFixInitialState,
          doc.IKoptions.getFixInitialState.doc)
      .def("setq0", &IKoptions::setq0, doc.IKoptions.setq0.doc)
      .def("getq0", &IKoptions::getq0, doc.IKoptions.getq0.doc)
      .def("setqd0", &IKoptions::setqd0, doc.IKoptions.setqd0.doc)
      .def("getqd0", &IKoptions::getqd0, doc.IKoptions.getqd0.doc)
      .def("setqdf", &IKoptions::setqdf, doc.IKoptions.setqdf.doc)
      .def("getqdf", &IKoptions::getqdf, doc.IKoptions.getqdf.doc)
      .def("setAdditionaltSamples", &IKoptions::setAdditionaltSamples,
          doc.IKoptions.setAdditionaltSamples.doc)
      .def("getAdditionaltSamples", &IKoptions::getAdditionaltSamples,
          doc.IKoptions.getAdditionaltSamples.doc);

  m.def("InverseKin",
      static_cast<IKResults (*)(  // NOLINT
          RigidBodyTree<double>*, const Eigen::VectorXd&,
          const Eigen::VectorXd&, const std::vector<RigidBodyConstraint*>&,
          const IKoptions&)>(&inverseKinSimple),
      doc.inverseKin.doc);

  m.def("InverseKinPointwise",
      static_cast<IKResults (*)(RigidBodyTree<double>*, const Eigen::VectorXd&,
          const Eigen::MatrixXd&, const Eigen::MatrixXd&,
          const std::vector<RigidBodyConstraint*>&, const IKoptions&)>(
          &inverseKinPointwiseSimple),
      doc.inverseKinPointwise.doc);

  m.def("InverseKinTraj", &inverseKinTrajSimple, doc.inverseKinTraj.doc);

  py::class_<IKResults>(m, "IKResults", doc.IKResults.doc)
      .def_readonly("q_sol", &IKResults::q_sol, doc.IKResults.q_sol.doc)
      .def_readonly("info", &IKResults::info, doc.IKResults.info.doc)
      .def_readonly("infeasible_constraints",
          &IKResults::infeasible_constraints,
          doc.IKResults.infeasible_constraints.doc);

  ExecuteExtraPythonCode(m);
}

}  // namespace pydrake
}  // namespace drake
