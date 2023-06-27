#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/common/sorted_pair_pybind.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/optimization/centroidal_momentum_constraint.h"
#include "drake/multibody/optimization/quaternion_integration_constraint.h"
#include "drake/multibody/optimization/spatial_velocity_constraint.h"
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
  py::module::import("pydrake.solvers");

  {
    using Class = CalcGridPointsOptions;
    constexpr auto& cls_doc = doc.CalcGridPointsOptions;
    py::class_<Class> cls(m, "CalcGridPointsOptions", cls_doc.doc);
    cls  // BR
        .def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
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
    using Class = ContactWrenchFromForceInWorldFrameEvaluator;
    constexpr auto& cls_doc = doc.ContactWrenchFromForceInWorldFrameEvaluator;
    using Ptr = std::shared_ptr<Class>;
    py::class_<Class, solvers::EvaluatorBase, Ptr>(
        m, "ContactWrenchFromForceInWorldFrameEvaluator", cls_doc.doc)
        .def(py::init<const MultibodyPlant<AutoDiffXd>*,
                 systems::Context<AutoDiffXd>*,
                 const SortedPair<geometry::GeometryId>&>(),
            py::arg("plant"), py::arg("context"), py::arg("geometry_id_pair"),
            cls_doc.ctor.doc);
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
            cls_doc.ctor.doc)
        .def("allow_quaternion_negation", &Class::allow_quaternion_negation,
            cls_doc.allow_quaternion_negation.doc)
        .def("ComposeVariable", &Class::ComposeVariable<double>,
            py::arg("quat1"), py::arg("quat2"), py::arg("angular_vel"),
            py::arg("h"), cls_doc.ComposeVariable.doc)
        .def("ComposeVariable", &Class::ComposeVariable<symbolic::Variable>,
            py::arg("quat1"), py::arg("quat2"), py::arg("angular_vel"),
            py::arg("h"), cls_doc.ComposeVariable.doc)
        .def("ComposeVariable", &Class::ComposeVariable<symbolic::Expression>,
            py::arg("quat1"), py::arg("quat2"), py::arg("angular_vel"),
            py::arg("h"), cls_doc.ComposeVariable.doc);
  }

  {
    using Class = SpatialVelocityConstraint;
    constexpr auto& cls_doc = doc.SpatialVelocityConstraint;
    using Ptr = std::shared_ptr<Class>;
    py::class_<Class, solvers::Constraint, Ptr> cls(
        m, "SpatialVelocityConstraint", cls_doc.doc);
    cls.def(py::init([](const MultibodyPlant<AutoDiffXd>* plant,
                         const Frame<AutoDiffXd>& frameA,
                         const Eigen::Ref<const Eigen::Vector3d>& v_AC_lower,
                         const Eigen::Ref<const Eigen::Vector3d>& v_AC_upper,
                         const Frame<AutoDiffXd>& frameB,
                         const Eigen::Ref<const Eigen::Vector3d>& p_BCo,
                         systems::Context<AutoDiffXd>* plant_context,
                         const std::optional<
                             SpatialVelocityConstraint::AngularVelocityBounds>&
                             w_AC_bounds) {
      return std::make_unique<Class>(plant, frameA, v_AC_lower, v_AC_upper,
          frameB, p_BCo, plant_context, w_AC_bounds);
    }),
        py::arg("plant"), py::arg("frameA"), py::arg("v_AC_lower"),
        py::arg("v_AC_upper"), py::arg("frameB"), py::arg("p_BCo"),
        py::arg("plant_context"), py::arg("w_AC_bounds") = std::nullopt,
        // Keep alive, reference: `self` keeps `plant` alive.
        py::keep_alive<1, 2>(),
        // Keep alive, reference: `self` keeps `plant_context` alive.
        py::keep_alive<1, 8>(), cls_doc.ctor.doc);

    using Avb = SpatialVelocityConstraint::AngularVelocityBounds;
    constexpr auto& avb_doc =
        doc.SpatialVelocityConstraint.AngularVelocityBounds;
    py::class_<SpatialVelocityConstraint::AngularVelocityBounds>(
        cls, "AngularVelocityBounds", avb_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc)
        .def_readwrite("magnitude_lower", &Avb::magnitude_lower,
            avb_doc.magnitude_lower.doc)
        .def_readwrite("magnitude_upper", &Avb::magnitude_upper,
            avb_doc.magnitude_upper.doc)
        .def_readwrite("reference_direction", &Avb::reference_direction,
            avb_doc.reference_direction.doc)
        .def_readwrite(
            "theta_bound", &Avb::theta_bound, avb_doc.theta_bound.doc);
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
            py::overload_cast<const Frame<double>&, const double&>(
                &Class::AddFrameTranslationalSpeedLimit),
            py::arg("constraint_frame"), py::arg("upper_limit"),
            cls_doc.AddFrameTranslationalSpeedLimit.doc_const)
        .def("AddFrameTranslationalSpeedLimit",
            py::overload_cast<const Frame<double>&, const Trajectory<double>&>(
                &Class::AddFrameTranslationalSpeedLimit),
            py::arg("constraint_frame"), py::arg("upper_limit"),
            cls_doc.AddFrameTranslationalSpeedLimit.doc_trajectory)
        .def("AddFrameAccelerationLimit",
            py::overload_cast<const Frame<double>&,
                const Eigen::Ref<const Vector6d>&,
                const Eigen::Ref<const Vector6d>&, ToppraDiscretization>(
                &Class::AddFrameAccelerationLimit),
            py::arg("constraint_frame"), py::arg("lower_limit"),
            py::arg("upper_limit"),
            py::arg("discretization") = ToppraDiscretization::kInterpolation,
            cls_doc.AddFrameAccelerationLimit.doc_const)
        .def("AddFrameAccelerationLimit",
            py::overload_cast<const Frame<double>&, const Trajectory<double>&,
                const Trajectory<double>&, ToppraDiscretization>(
                &Class::AddFrameAccelerationLimit),
            py::arg("constraint_frame"), py::arg("lower_limit"),
            py::arg("upper_limit"),
            py::arg("discretization") = ToppraDiscretization::kInterpolation,
            cls_doc.AddFrameAccelerationLimit.doc_trajectory);
  }
}
}  // namespace
}  // namespace pydrake
}  // namespace drake
