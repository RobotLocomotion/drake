#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/optimization/centroidal_momentum_constraint.h"
#include "drake/multibody/optimization/quaternion_integration_constraint.h"
#include "drake/multibody/optimization/static_equilibrium_problem.h"
#include "drake/multibody/optimization/toppra.h"

namespace drake {
namespace pydrake {

namespace {
PYBIND11_MODULE(optimization, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody;

  m.doc() = "Optimization module for MultibodyPlant motion planning";

  py::module::import("pydrake.math");
  py::module::import("pydrake.multibody.plant");
  py::module::import("pydrake.solvers.mathematicalprogram");

  {
    using Class = CalcGridPointsOptions;
    constexpr auto& cls_doc = doc.CalcGridPointsOptions;
    py::class_<Class>(m, "CalcGridPointsOptions", cls_doc.doc)
        .def(ParamInit<Class>())
        .def_readwrite("max_err", &Class::max_err, cls_doc.max_err.doc)
        .def_readwrite("max_iter", &Class::max_iter, cls_doc.max_iter.doc)
        .def_readwrite("max_seg_length", &Class::max_seg_length,
            cls_doc.max_seg_length.doc)
        .def_readwrite(
            "min_points", &Class::min_points, cls_doc.min_points.doc);
  }

  {
    using Class = CentroidalMomentumConstraint;
    constexpr auto& cls_doc = doc.CentroidalMomentumConstraint;
    using Ptr = std::shared_ptr<Class>;
    py::class_<Class, solvers::Constraint, Ptr>(
        m, "CentroidalMomentumConstraint", cls_doc.doc)
        .def(py::init([](const MultibodyPlant<AutoDiffXd>* plant,
                          std::optional<std::vector<ModelInstanceIndex>>
                              model_instances,
                          systems::Context<AutoDiffXd>* plant_context,
                          bool angular_only) {
          return std::make_unique<Class>(
              plant, model_instances, plant_context, angular_only);
        }),
            py::arg("plant"), py::arg("model_instances"),
            py::arg("plant_context"), py::arg("angular_only"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 4>(), cls_doc.ctor.doc);
  }

  {
    py::class_<ContactWrench>(m, "ContactWrench", doc.ContactWrench.doc)
        .def_readonly("bodyA_index", &ContactWrench::bodyA_index,
            doc.ContactWrench.bodyA_index.doc)
        .def_readonly("bodyB_index", &ContactWrench::bodyB_index,
            doc.ContactWrench.bodyB_index.doc)
        .def_readonly(
            "p_WCb_W", &ContactWrench::p_WCb_W, doc.ContactWrench.p_WCb_W.doc)
        .def_readonly(
            "F_Cb_W", &ContactWrench::F_Cb_W, doc.ContactWrench.F_Cb_W.doc);
    AddValueInstantiation<ContactWrench>(m);
  }

  {
    using Class = QuaternionEulerIntegrationConstraint;
    constexpr auto& cls_doc = doc.QuaternionEulerIntegrationConstraint;
    using Ptr = std::shared_ptr<Class>;
    py::class_<Class, solvers::Constraint, Ptr>(
        m, "QuaternionEulerIntegrationConstraint", cls_doc.doc)
        .def(py::init<bool>(), py::arg("allow_quaternion_negation"),
            cls_doc.ctor.doc);
  }

  {
    using Class = StaticEquilibriumProblem;
    constexpr auto& cls_doc = doc.StaticEquilibriumProblem;
    py::class_<Class>(m, "StaticEquilibriumProblem", cls_doc.doc)
        .def(py::init<const MultibodyPlant<AutoDiffXd>*,
                 systems::Context<AutoDiffXd>*,
                 const std::set<
                     std::pair<geometry::GeometryId, geometry::GeometryId>>&>(),
            py::arg("plant"), py::arg("context"),
            py::arg("ignored_collision_pairs"),
            // Keep alive, reference: `self` keeps `plant` and `context` alive.
            py::keep_alive<1, 2>(), py::keep_alive<1, 3>(), cls_doc.ctor.doc)
        .def("get_mutable_prog", &Class::get_mutable_prog,
            py_rvp::reference_internal, cls_doc.get_mutable_prog.doc)
        .def("prog", &Class::prog, py_rvp::reference_internal, cls_doc.prog.doc)
        .def("q_vars", &Class::q_vars, cls_doc.q_vars.doc)
        .def("u_vars", &Class::u_vars, cls_doc.u_vars.doc)
        .def("GetContactWrenchSolution", &Class::GetContactWrenchSolution,
            py::arg("result"), cls_doc.GetContactWrenchSolution.doc)
        .def("UpdateComplementarityTolerance",
            &Class::UpdateComplementarityTolerance, py::arg("tol"),
            cls_doc.UpdateComplementarityTolerance.doc);
  }

  {
    using Class = ToppraDiscretization;
    constexpr auto& cls_doc = doc.ToppraDiscretization;
    py::enum_<Class>(m, "ToppraDiscretization", cls_doc.doc)
        .value("kCollocation", Class::kCollocation, cls_doc.kCollocation.doc)
        .value("kInterpolation", Class::kInterpolation,
            cls_doc.kInterpolation.doc);
  }

  {
    using Class = Toppra;
    constexpr auto& cls_doc = doc.Toppra;
    py::class_<Class>(m, "Toppra", cls_doc.doc)
        .def(py::init<const Trajectory<double>&, const MultibodyPlant<double>&,
                 const Eigen::Ref<const Eigen::VectorXd>&>(),
            py::arg("path"), py::arg("plant"), py::arg("gridpoints"),
            cls_doc.ctor.doc)
        .def_static("CalcGridPoints", &Class::CalcGridPoints, py::arg("path"),
            py::arg("options"), cls_doc.CalcGridPoints.doc)
        .def("SolvePathParameterization", &Class::SolvePathParameterization,
            cls_doc.SolvePathParameterization.doc)
        .def("AddJointVelocityLimit", &Class::AddJointVelocityLimit,
            py::arg("lower_limit"), py::arg("upper_limit"),
            cls_doc.AddJointVelocityLimit.doc)
        .def("AddJointAccelerationLimit", &Class::AddJointAccelerationLimit,
            py::arg("lower_limit"), py::arg("upper_limit"),
            py::arg("discretization") = ToppraDiscretization::kInterpolation,
            cls_doc.AddJointAccelerationLimit.doc)
        .def("AddJointTorqueLimit", &Class::AddJointTorqueLimit,
            py::arg("lower_limit"), py::arg("upper_limit"),
            py::arg("discretization") = ToppraDiscretization::kInterpolation,
            cls_doc.AddJointTorqueLimit.doc)
        .def("AddFrameVelocityLimit", &Class::AddFrameVelocityLimit,
            py::arg("constraint_frame"), py::arg("lower_limit"),
            py::arg("upper_limit"), cls_doc.AddFrameVelocityLimit.doc)
        .def("AddFrameTranslationalSpeedLimit",
            &Class::AddFrameTranslationalSpeedLimit,
            py::arg("constraint_frame"), py::arg("upper_limit"),
            cls_doc.AddFrameTranslationalSpeedLimit.doc)
        .def("AddFrameAccelerationLimit", &Class::AddFrameAccelerationLimit,
            py::arg("constraint_frame"), py::arg("lower_limit"),
            py::arg("upper_limit"),
            py::arg("discretization") = ToppraDiscretization::kInterpolation,
            cls_doc.AddFrameAccelerationLimit.doc);
  }
}
}  // namespace
}  // namespace pydrake
}  // namespace drake
