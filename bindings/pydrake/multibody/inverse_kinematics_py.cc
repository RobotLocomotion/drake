#include "drake/bindings/pydrake/multibody/inverse_kinematics_py.h"

#include <memory>
#include <string>
#include <vector>

#include "drake/bindings/generated_docstrings/multibody_inverse_kinematics.h"
#include "drake/bindings/generated_docstrings/solvers.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/ref_cycle_pybind.h"
#include "drake/bindings/pydrake/common/sorted_pair_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/inverse_kinematics/add_multibody_plant_constraints.h"
#include "drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h"
#include "drake/multibody/inverse_kinematics/angle_between_vectors_cost.h"
#include "drake/multibody/inverse_kinematics/com_in_polyhedron_constraint.h"
#include "drake/multibody/inverse_kinematics/com_position_constraint.h"
#include "drake/multibody/inverse_kinematics/distance_constraint.h"
#include "drake/multibody/inverse_kinematics/gaze_target_constraint.h"
#include "drake/multibody/inverse_kinematics/global_inverse_kinematics.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.h"
#include "drake/multibody/inverse_kinematics/minimum_distance_upper_bound_constraint.h"
#include "drake/multibody/inverse_kinematics/orientation_constraint.h"
#include "drake/multibody/inverse_kinematics/orientation_cost.h"
#include "drake/multibody/inverse_kinematics/point_to_line_distance_constraint.h"
#include "drake/multibody/inverse_kinematics/point_to_point_distance_constraint.h"
#include "drake/multibody/inverse_kinematics/polyhedron_constraint.h"
#include "drake/multibody/inverse_kinematics/position_constraint.h"
#include "drake/multibody/inverse_kinematics/position_cost.h"
#include "drake/multibody/inverse_kinematics/unit_quaternion_constraint.h"

namespace drake {
namespace pydrake {
namespace {
// This is used in the
// MinimumDistanceLowerBoundConstraint/MinimumDistanceUpperBoundConstraint to
// convert a penalty function in python to a penalty function in C++.
using CppPenaltyFunction = std::function<void(double, double*, double*)>;
using PyPenaltyFunction = std::function<py::tuple(double, bool)>;
CppPenaltyFunction UnwrapPyPenaltyFunction(PyPenaltyFunction penalty_function) {
  if (penalty_function) {
    return [penalty_function](double x, double* penalty, double* dpenalty) {
      const bool compute_grad = dpenalty != nullptr;
      py::tuple penalty_tuple = penalty_function(x, compute_grad);
      *penalty = py::cast<double>(penalty_tuple[0]);
      if (compute_grad) {
        *dpenalty = py::cast<double>(penalty_tuple[1]);
      }
    };
  } else {
    return solvers::MinimumValuePenaltyFunction{};
  }
}

using solvers::Constraint;

PYDRAKE_MODULE(inverse_kinematics, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc =
      pydrake_doc_multibody_inverse_kinematics.drake.multibody;
  constexpr auto& constraint_doc = pydrake_doc_solvers.drake.solvers.Constraint;

  m.doc() = "InverseKinematics module";

  py::module_::import_("pydrake.math");
  py::module_::import_("pydrake.multibody.plant");
  py::module_::import_("pydrake.planning");
  py::module_::import_("pydrake.solvers");

  {
    using Class = InverseKinematics;
    constexpr auto& cls_doc = doc.InverseKinematics;
    py::class_<Class> cls(
        m, "InverseKinematics", py::dynamic_attr(), cls_doc.doc);
    cls  // BR
        .def(py::init<const MultibodyPlant<double>&, bool>(), py::arg("plant"),
            py::arg("with_joint_limits") = true,
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),  // BR
            cls_doc.ctor.doc_2args)
        .def(py::init<const MultibodyPlant<double>&, systems::Context<double>*,
                 bool>(),
            py::arg("plant"), py::arg("plant_context"),
            py::arg("with_joint_limits") = true,
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),  // BR
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 3>(),  // BR
            cls_doc.ctor.doc_3args)
        .def(
            "AddPositionConstraint",
            [](Class* self, const Frame<double>& frameB,
                const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
                const Frame<double>& frameA,
                const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
                const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper) {
              return self->AddPositionConstraint(
                  frameB, p_BQ, frameA, p_AQ_lower, p_AQ_upper);
            },
            py::arg("frameB"), py::arg("p_BQ"), py::arg("frameA"),
            py::arg("p_AQ_lower"), py::arg("p_AQ_upper"),
            cls_doc.AddPositionConstraint.doc_5args)
        .def(
            "AddPositionConstraint",
            [](Class* self, const Frame<double>& frameB,
                const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
                const Frame<double>& frameAbar,
                const std::optional<math::RigidTransformd>& X_AbarA,
                const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
                const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper) {
              return self->AddPositionConstraint(
                  frameB, p_BQ, frameAbar, X_AbarA, p_AQ_lower, p_AQ_upper);
            },
            py::arg("frameB"), py::arg("p_BQ"), py::arg("frameAbar"),
            py::arg("X_AbarA"), py::arg("p_AQ_lower"), py::arg("p_AQ_upper"),
            cls_doc.AddPositionConstraint.doc_6args)
        .def("AddPositionCost", &Class::AddPositionCost, py::arg("frameA"),
            py::arg("p_AP"), py::arg("frameB"), py::arg("p_BQ"), py::arg("C"),
            cls_doc.AddPositionCost.doc)
        .def("AddOrientationConstraint", &Class::AddOrientationConstraint,
            py::arg("frameAbar"), py::arg("R_AbarA"), py::arg("frameBbar"),
            py::arg("R_BbarB"), py::arg("theta_bound"),
            cls_doc.AddOrientationConstraint.doc)
        .def("AddOrientationCost", &Class::AddOrientationCost,
            py::arg("frameAbar"), py::arg("R_AbarA"), py::arg("frameBbar"),
            py::arg("R_BbarB"), py::arg("c"), cls_doc.AddOrientationCost.doc)
        .def("AddGazeTargetConstraint", &Class::AddGazeTargetConstraint,
            py::arg("frameA"), py::arg("p_AS"), py::arg("n_A"),
            py::arg("frameB"), py::arg("p_BT"), py::arg("cone_half_angle"),
            cls_doc.AddGazeTargetConstraint.doc)
        .def("AddAngleBetweenVectorsConstraint",
            &Class::AddAngleBetweenVectorsConstraint, py::arg("frameA"),
            py::arg("na_A"), py::arg("frameB"), py::arg("nb_B"),
            py::arg("angle_lower"), py::arg("angle_upper"),
            cls_doc.AddAngleBetweenVectorsConstraint.doc)
        .def("AddAngleBetweenVectorsCost", &Class::AddAngleBetweenVectorsCost,
            py::arg("frameA"), py::arg("na_A"), py::arg("frameB"),
            py::arg("nb_B"), py::arg("c"),
            cls_doc.AddAngleBetweenVectorsCost.doc)
        .def("AddMinimumDistanceLowerBoundConstraint",
            &Class::AddMinimumDistanceLowerBoundConstraint, py::arg("bound"),
            py::arg("influence_distance_offset") = 0.01,
            cls_doc.AddMinimumDistanceLowerBoundConstraint.doc)
        .def("AddMinimumDistanceUpperBoundConstraint",
            &Class::AddMinimumDistanceUpperBoundConstraint, py::arg("bound"),
            py::arg("influence_distance_offset"),
            cls_doc.AddMinimumDistanceUpperBoundConstraint.doc)
#ifdef PYDRAKE_USE_PYBIND11  // XXX porting
        .def("AddDistanceConstraint", &Class::AddDistanceConstraint,
            py::arg("geometry_pair"), py::arg("distance_lower"),
            py::arg("distance_upper"), cls_doc.AddDistanceConstraint.doc)
#endif  // XXX porting
        .def("AddPointToLineDistanceConstraint",
            &Class::AddPointToLineDistanceConstraint, py::arg("frame_point"),
            py::arg("p_B1P"), py::arg("frame_line"), py::arg("p_B2Q"),
            py::arg("n_B2"), py::arg("distance_lower"),
            py::arg("distance_upper"),
            cls_doc.AddPointToLineDistanceConstraint.doc)
        .def("AddPointToPointDistanceConstraint",
            &Class::AddPointToPointDistanceConstraint, py::arg("frame1"),
            py::arg("p_B1P1"), py::arg("frame2"), py::arg("p_B2P2"),
            py::arg("distance_lower"), py::arg("distance_upper"),
            cls_doc.AddPointToPointDistanceConstraint.doc)
        .def("AddPolyhedronConstraint", &Class::AddPolyhedronConstraint,
            py::arg("frameF"), py::arg("frameG"), py::arg("p_GP"), py::arg("A"),
            py::arg("b"), cls_doc.AddPolyhedronConstraint.doc)
        .def("q", &Class::q, cls_doc.q.doc)
        .def("prog", &Class::prog, internal::ref_cycle<0, 1>(),
            py_rvp::reference, cls_doc.prog.doc)
        .def("get_mutable_prog", &Class::get_mutable_prog,
            py_rvp::reference_internal, cls_doc.get_mutable_prog.doc)
        .def("context", &Class::context, py_rvp::reference_internal,
            cls_doc.context.doc)
        .def("get_mutable_context", &Class::get_mutable_context,
            py_rvp::reference_internal, cls_doc.get_mutable_context.doc);
  }

  m.def(
      "AddMultibodyPlantConstraints",
      [](py::object plant, const solvers::VectorXDecisionVariable& q,
          solvers::MathematicalProgram* prog,
          systems::Context<double>* plant_context) {
        return AddMultibodyPlantConstraints(
            make_shared_ptr_from_py_object<MultibodyPlant<double>>(plant), q,
            prog, plant_context);
      },
      py::arg("plant"), py::arg("q"), py::arg("prog"),
      py::arg("plant_context") = py::none(),
      doc.AddMultibodyPlantConstraints.doc);

  {
    using Class = AngleBetweenVectorsConstraint;
    constexpr auto& cls_doc = doc.AngleBetweenVectorsConstraint;
    py::class_<Class, Constraint
#ifdef PYDRAKE_USE_PYBIND11
,
std::shared_ptr<Class>
#endif
>(
        m, "AngleBetweenVectorsConstraint", cls_doc.doc)
        .def(py::init<const MultibodyPlant<double>*, const Frame<double>&,
                 const Eigen::Ref<const Eigen::Vector3d>&, const Frame<double>&,
                 const Eigen::Ref<const Eigen::Vector3d>&, double, double,
                 systems::Context<double>*>(),
            py::arg("plant"), py::arg("frameA"), py::arg("a_A"),
            py::arg("frameB"), py::arg("b_B"), py::arg("angle_lower"),
            py::arg("angle_upper"), py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 9>(), cls_doc.ctor.doc_double)
        .def(py::init<const MultibodyPlant<AutoDiffXd>*,
                 const Frame<AutoDiffXd>&,
                 const Eigen::Ref<const Eigen::Vector3d>&,
                 const Frame<AutoDiffXd>&,
                 const Eigen::Ref<const Eigen::Vector3d>&, double, double,
                 systems::Context<AutoDiffXd>*>(),
            py::arg("plant"), py::arg("frameA"), py::arg("a_A"),
            py::arg("frameB"), py::arg("b_B"), py::arg("angle_lower"),
            py::arg("angle_upper"), py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 9>(), cls_doc.ctor.doc_autodiff);
  }
  {
    using Class = AngleBetweenVectorsCost;
    constexpr auto& cls_doc = doc.AngleBetweenVectorsCost;
    py::class_<Class, solvers::Cost
#ifdef PYDRAKE_USE_PYBIND11
,
std::shared_ptr<Class>
#endif
>(
        m, "AngleBetweenVectorsCost", cls_doc.doc)
        .def(py::init<const MultibodyPlant<double>*, const Frame<double>&,
                 const Eigen::Ref<const Eigen::Vector3d>&, const Frame<double>&,
                 const Eigen::Ref<const Eigen::Vector3d>&, double,
                 systems::Context<double>*>(),
            py::arg("plant"), py::arg("frameA"), py::arg("a_A"),
            py::arg("frameB"), py::arg("b_B"), py::arg("c"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_double)
        .def(py::init<const MultibodyPlant<AutoDiffXd>*,
                 const Frame<AutoDiffXd>&,
                 const Eigen::Ref<const Eigen::Vector3d>&,
                 const Frame<AutoDiffXd>&,
                 const Eigen::Ref<const Eigen::Vector3d>&, double,
                 systems::Context<AutoDiffXd>*>(),
            py::arg("plant"), py::arg("frameA"), py::arg("a_A"),
            py::arg("frameB"), py::arg("b_B"), py::arg("c"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_autodiff);
  }
  {
    using Class = PointToPointDistanceConstraint;
    constexpr auto& cls_doc = doc.PointToPointDistanceConstraint;
    py::class_<Class, Constraint
#ifdef PYDRAKE_USE_PYBIND11
,
std::shared_ptr<Class>
#endif
>(
        m, "PointToPointDistanceConstraint", cls_doc.doc)
        .def(py::init<const multibody::MultibodyPlant<double>* const,
                 const multibody::Frame<double>&,
                 const Eigen::Ref<const Eigen::Vector3d>&,
                 const multibody::Frame<double>&,
                 const Eigen::Ref<const Eigen::Vector3d>&, double, double,
                 systems::Context<double>*>(),
            py::arg("plant"), py::arg("frame1"), py::arg("p_B1P1"),
            py::arg("frame2"), py::arg("p_B2P2"), py::arg("distance_lower"),
            py::arg("distance_upper"), py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 9>(), cls_doc.ctor.doc_double)
        .def(py::init<const multibody::MultibodyPlant<AutoDiffXd>* const,
                 const multibody::Frame<AutoDiffXd>&,
                 const Eigen::Ref<const Eigen::Vector3d>&,
                 const multibody::Frame<AutoDiffXd>&,
                 const Eigen::Ref<const Eigen::Vector3d>&, double, double,
                 systems::Context<AutoDiffXd>*>(),
            py::arg("plant"), py::arg("frame1"), py::arg("p_B1P1"),
            py::arg("frame2"), py::arg("p_B2P2"), py::arg("distance_lower"),
            py::arg("distance_upper"), py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 9>(), cls_doc.ctor.doc_autodiff);
  }
  {
    using Class = PointToLineDistanceConstraint;
    constexpr auto& cls_doc = doc.PointToLineDistanceConstraint;
    py::class_<Class, Constraint
#ifdef PYDRAKE_USE_PYBIND11
,
std::shared_ptr<Class>
#endif
>(
        m, "PointToLineDistanceConstraint", cls_doc.doc)
        .def(py::init<const multibody::MultibodyPlant<double>* const,
                 const multibody::Frame<double>&,
                 const Eigen::Ref<const Eigen::Vector3d>&,
                 const multibody::Frame<double>&,
                 const Eigen::Ref<const Eigen::Vector3d>&,
                 const Eigen::Ref<const Eigen::Vector3d>&, double, double,
                 systems::Context<double>*>(),
            py::arg("plant"), py::arg("frame_point"), py::arg("p_B1P"),
            py::arg("frame_line"), py::arg("p_B2Q"), py::arg("n_B2"),
            py::arg("distance_lower"), py::arg("distance_upper"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 10>(), cls_doc.ctor.doc_double)
        .def(py::init<const multibody::MultibodyPlant<AutoDiffXd>* const,
                 const multibody::Frame<AutoDiffXd>&,
                 const Eigen::Ref<const Eigen::Vector3d>&,
                 const multibody::Frame<AutoDiffXd>&,
                 const Eigen::Ref<const Eigen::Vector3d>&,
                 const Eigen::Ref<const Eigen::Vector3d>&, double, double,
                 systems::Context<AutoDiffXd>*>(),
            py::arg("plant"), py::arg("frame_point"), py::arg("p_B1P"),
            py::arg("frame_line"), py::arg("p_B2Q"), py::arg("n_B2"),
            py::arg("distance_lower"), py::arg("distance_upper"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 10>(), cls_doc.ctor.doc_autodiff);
  }
  {
    using Class = PolyhedronConstraint;
    constexpr auto& cls_doc = doc.PolyhedronConstraint;
    py::class_<Class, Constraint
#ifdef PYDRAKE_USE_PYBIND11
,
std::shared_ptr<Class>
#endif
>(
        m, "PolyhedronConstraint", cls_doc.doc)
        .def(py::init<const multibody::MultibodyPlant<double>* const,
                 const multibody::Frame<double>&,
                 const multibody::Frame<double>&,
                 const Eigen::Ref<const Eigen::Matrix3Xd>&,
                 const Eigen::Ref<const Eigen::MatrixXd>&,
                 const Eigen::Ref<const Eigen::VectorXd>&,
                 systems::Context<double>*>(),
            py::arg("plant"), py::arg("frameF"), py::arg("frameG"),
            py::arg("p_GP"), py::arg("A"), py::arg("b"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_double)
        .def(py::init<const multibody::MultibodyPlant<AutoDiffXd>* const,
                 const multibody::Frame<AutoDiffXd>&,
                 const multibody::Frame<AutoDiffXd>&,
                 const Eigen::Ref<const Eigen::Matrix3Xd>&,
                 const Eigen::Ref<const Eigen::MatrixXd>&,
                 const Eigen::Ref<const Eigen::VectorXd>&,
                 systems::Context<AutoDiffXd>*>(),
            py::arg("plant"), py::arg("frameF"), py::arg("frameG"),
            py::arg("p_GP"), py::arg("A"), py::arg("b"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_autodiff);
  }
  {
    using Class = DistanceConstraint;
    constexpr auto& cls_doc = doc.DistanceConstraint;
    py::class_<Class, Constraint
#ifdef PYDRAKE_USE_PYBIND11
,
std::shared_ptr<Class>
#endif
>(
        m, "DistanceConstraint", cls_doc.doc)
        .def(py::init<const multibody::MultibodyPlant<double>* const,
                 SortedPair<geometry::GeometryId>, systems::Context<double>*,
                 double, double>(),
            py::arg("plant"), py::arg("geometry_pair"),
            py::arg("plant_context"), py::arg("distance_lower"),
            py::arg("distance_upper"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 4>(), cls_doc.ctor.doc_double)
        .def(py::init<const multibody::MultibodyPlant<AutoDiffXd>* const,
                 SortedPair<geometry::GeometryId>,
                 systems::Context<AutoDiffXd>*, double, double>(),
            py::arg("plant"), py::arg("geometry_pair"),
            py::arg("plant_context"), py::arg("distance_lower"),
            py::arg("distance_upper"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 4>(), cls_doc.ctor.doc_autodiff);
  }

  {
    using Class = GazeTargetConstraint;
    constexpr auto& cls_doc = doc.GazeTargetConstraint;
    py::class_<Class, Constraint
#ifdef PYDRAKE_USE_PYBIND11
,
std::shared_ptr<Class>
#endif
>(
        m, "GazeTargetConstraint", cls_doc.doc)
        .def(py::init<const MultibodyPlant<double>*, const Frame<double>&,
                 const Eigen::Ref<const Eigen::Vector3d>&,
                 const Eigen::Ref<const Eigen::Vector3d>&, const Frame<double>&,
                 const Eigen::Ref<const Eigen::Vector3d>&, double,
                 systems::Context<double>*>(),
            py::arg("plant"), py::arg("frameA"), py::arg("p_AS"),
            py::arg("n_A"), py::arg("frameB"), py::arg("p_BT"),
            py::arg("cone_half_angle"), py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 9>(), cls_doc.ctor.doc_double)
        .def(py::init<const MultibodyPlant<AutoDiffXd>*,
                 const Frame<AutoDiffXd>&,
                 const Eigen::Ref<const Eigen::Vector3d>&,
                 const Eigen::Ref<const Eigen::Vector3d>&,
                 const Frame<AutoDiffXd>&,
                 const Eigen::Ref<const Eigen::Vector3d>&, double,
                 systems::Context<AutoDiffXd>*>(),
            py::arg("plant"), py::arg("frameA"), py::arg("p_AS"),
            py::arg("n_A"), py::arg("frameB"), py::arg("p_BT"),
            py::arg("cone_half_angle"), py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 9>(), cls_doc.ctor.doc_autodiff);
  }

  {
    using Class = MinimumDistanceLowerBoundConstraint;
    constexpr auto& cls_doc = doc.MinimumDistanceLowerBoundConstraint;
    py::class_<Class, Constraint
#ifdef PYDRAKE_USE_PYBIND11
,
std::shared_ptr<Class>
#endif
>
        minimum_distance_lower_bound_constraint(
            m, "MinimumDistanceLowerBoundConstraint", cls_doc.doc);

    std::string py_penalty_doc =
        "The penalty function `penalty_function(x: float, compute_grad: bool) "
        "-> tuple[float, Optional[float]]` returns `[penalty_val,  "
        "penalty_gradient]` when `compute_grad=True`, or `[penalty_value, "
        "None]` "
        "when `compute_grad=False`. See minimum_value_constraint.h on the "
        "requirement on MinimumValuePenaltyFunction. Set penalty_function=None "
        "and then the constraint will use the default penalty function.";
    const std::string constructor_double_mbp_py_penalty =
        cls_doc.ctor.doc_double_mbp + py_penalty_doc;

    minimum_distance_lower_bound_constraint.def(
        "__init__",
        [](Class* self, const multibody::MultibodyPlant<double>* plant,
            double bound, systems::Context<double>* plant_context,
            PyPenaltyFunction penalty_function,
            double influence_distance_offset) {
          new (self) Class(plant, bound, plant_context,
              UnwrapPyPenaltyFunction(penalty_function),
              influence_distance_offset);
        },
        py::arg("plant"), py::arg("bound"), py::arg("plant_context"),
        py::arg("penalty_function") = nullptr,
        py::arg("influence_distance_offset") = 0.01,
        // Keep alive, reference: `self` keeps `plant` alive.
        py::keep_alive<1, 2>(),
        // Keep alive, reference: `self` keeps `plant_context` alive.
        py::keep_alive<1, 4>(), constructor_double_mbp_py_penalty.c_str());

    const std::string constructor_autodiff_mbp_py_penalty =
        cls_doc.ctor.doc_autodiff_mbp + py_penalty_doc;

    minimum_distance_lower_bound_constraint.def(
        "__init__",
        [](Class* self, const multibody::MultibodyPlant<AutoDiffXd>* plant,
            double bound, systems::Context<AutoDiffXd>* plant_context,
            PyPenaltyFunction penalty_function,
            double influence_distance_offset) {
          new (self) Class(plant, bound, plant_context,
              UnwrapPyPenaltyFunction(penalty_function),
              influence_distance_offset);
        },
        py::arg("plant"), py::arg("bound"), py::arg("plant_context"),
        py::arg("penalty_function") = nullptr,
        py::arg("influence_distance_offset") = 0.01,
        // Keep alive, reference: `self` keeps `plant` alive.
        py::keep_alive<1, 2>(),
        // Keep alive, reference: `self` keeps `plant_context` alive.
        py::keep_alive<1, 4>(), constructor_autodiff_mbp_py_penalty.c_str());

    const std::string constructor_collision_checker_py_penalty =
        cls_doc.ctor.doc_collision_checker + py_penalty_doc;
    minimum_distance_lower_bound_constraint
        .def(
            "__init__",
            [](Class* self, const planning::CollisionChecker* collision_checker,
                double bound,
                planning::CollisionCheckerContext* collision_checker_context,
                PyPenaltyFunction penalty_function,
                double influence_distance_offset) {
              new (self)
                  Class(collision_checker, bound, collision_checker_context,
                      UnwrapPyPenaltyFunction(penalty_function),
                      influence_distance_offset);
            },
            py::arg("collision_checker"), py::arg("bound"),
            py::arg("collision_checker_context"),
            py::arg("penalty_function") =
                solvers::MinimumValuePenaltyFunction{},
            py::arg("influence_distance_offset") = 0.01,
            // Keep alive, reference: `self` keeps collision_checker
            // alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps collision_checker_context
            // alive.
            py::keep_alive<1, 4>(),
            constructor_collision_checker_py_penalty.c_str())
        .def("distance_bound", &Class::distance_bound,
            cls_doc.distance_bound.doc)
        .def("influence_distance", &Class::influence_distance,
            cls_doc.influence_distance.doc);
  }

  {
    using Class = MinimumDistanceUpperBoundConstraint;
    constexpr auto& cls_doc = doc.MinimumDistanceUpperBoundConstraint;
    py::class_<Class, Constraint
#ifdef PYDRAKE_USE_PYBIND11
,
std::shared_ptr<Class>
#endif
>
        minimum_distance_upper_bound_constraint(
            m, "MinimumDistanceUpperBoundConstraint", cls_doc.doc);

    std::string py_penalty_doc =
        "The penalty function `penalty_function(x: float, compute_grad: bool) "
        "-> tuple[float, Optional[float]]` returns `[penalty_val,  "
        "penalty_gradient]` when `compute_grad=True`, or `[penalty_value, "
        "None]` when `compute_grad=False`. See minimum_value_constraint.h on "
        "the requirement on MinimumValuePenaltyFunction. Set "
        "`penalty_function=None` and then the constraint will use the default "
        "penalty function.";
    const std::string constructor_double_mbp_py_penalty =
        cls_doc.ctor.doc_double_mbp + py_penalty_doc;

    minimum_distance_upper_bound_constraint.def(
        "__init__",
        [](Class* self, const multibody::MultibodyPlant<double>* plant,
            double bound, systems::Context<double>* plant_context,
            double influence_distance_offset,
            PyPenaltyFunction penalty_function) {
          new (self)
              Class(plant, bound, plant_context, influence_distance_offset,
                  UnwrapPyPenaltyFunction(penalty_function));
        },
        py::arg("plant"), py::arg("bound"), py::arg("plant_context"),
        py::arg("influence_distance_offset"),
        py::arg("penalty_function") = nullptr,
        // Keep alive, reference: `self` keeps `plant` alive.
        py::keep_alive<1, 2>(),
        // Keep alive, reference: `self` keeps `plant_context` alive.
        py::keep_alive<1, 4>(), constructor_double_mbp_py_penalty.c_str());

    const std::string constructor_autodiff_mbp_py_penalty =
        cls_doc.ctor.doc_autodiff_mbp + py_penalty_doc;

    minimum_distance_upper_bound_constraint.def(
        "__init__",
        [](Class* self, const multibody::MultibodyPlant<AutoDiffXd>* plant,
            double bound, systems::Context<AutoDiffXd>* plant_context,
            double influence_distance_offset,
            PyPenaltyFunction penalty_function) {
          new (self)
              Class(plant, bound, plant_context, influence_distance_offset,
                  UnwrapPyPenaltyFunction(penalty_function));
        },
        py::arg("plant"), py::arg("bound"), py::arg("plant_context"),
        py::arg("influence_distance_offset"),
        py::arg("penalty_function") = nullptr,
        // Keep alive, reference: `self` keeps `plant` alive.
        py::keep_alive<1, 2>(),
        // Keep alive, reference: `self` keeps `plant_context` alive.
        py::keep_alive<1, 4>(), constructor_autodiff_mbp_py_penalty.c_str());

    const std::string constructor_collision_checker_py_penalty =
        cls_doc.ctor.doc_collision_checker + py_penalty_doc;
    minimum_distance_upper_bound_constraint
        .def(
            "__init__",
            [](Class* self, const planning::CollisionChecker* collision_checker,
                double bound,
                planning::CollisionCheckerContext* collision_checker_context,
                double influence_distance_offset,
                PyPenaltyFunction penalty_function) {
              new (self) Class(collision_checker, bound,
                  collision_checker_context, influence_distance_offset,
                  UnwrapPyPenaltyFunction(penalty_function));
            },
            py::arg("collision_checker"), py::arg("bound"),
            py::arg("collision_checker_context"),
            py::arg("influence_distance_offset"),
            py::arg("penalty_function") =
                solvers::MinimumValuePenaltyFunction{},
            // Keep alive, reference: `self` keeps collision_checker alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps collision_checker_context
            // alive.
            py::keep_alive<1, 4>(),
            constructor_collision_checker_py_penalty.c_str())
        .def("distance_bound", &Class::distance_bound,
            cls_doc.distance_bound.doc)
        .def("influence_distance", &Class::influence_distance,
            cls_doc.influence_distance.doc);
  }

  {
    using Class = PositionConstraint;
    constexpr auto& cls_doc = doc.PositionConstraint;
    py::class_<Class, Constraint
#ifdef PYDRAKE_USE_PYBIND11
,
std::shared_ptr<Class>
#endif
>(
        m, "PositionConstraint", cls_doc.doc)
        .def(py::init<const MultibodyPlant<double>*, const Frame<double>&,
                 const Eigen::Ref<const Eigen::Vector3d>&,
                 const Eigen::Ref<const Eigen::Vector3d>&, const Frame<double>&,
                 std::optional<Eigen::Vector3d>, systems::Context<double>*>(),
            py::arg("plant"), py::arg("frameA"), py::arg("p_AQ_lower"),
            py::arg("p_AQ_upper"), py::arg("frameB"), py::arg("p_BQ"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_double)
        .def(py::init<const MultibodyPlant<double>*, const Frame<double>&,
                 const std::optional<math::RigidTransformd>&,
                 const Eigen::Ref<const Eigen::Vector3d>&,
                 const Eigen::Ref<const Eigen::Vector3d>&, const Frame<double>&,
                 std::optional<Eigen::Vector3d>, systems::Context<double>*>(),
            py::arg("plant"), py::arg("frameAbar"), py::arg("X_AbarA"),
            py::arg("p_AQ_lower"), py::arg("p_AQ_upper"), py::arg("frameB"),
            py::arg("p_BQ"), py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 9>(), cls_doc.ctor.doc_double_Abar)
        .def(py::init<const MultibodyPlant<AutoDiffXd>*,
                 const Frame<AutoDiffXd>&,
                 const Eigen::Ref<const Eigen::Vector3d>&,
                 const Eigen::Ref<const Eigen::Vector3d>&,
                 const Frame<AutoDiffXd>&, std::optional<Eigen::Vector3d>,
                 systems::Context<AutoDiffXd>*>(),
            py::arg("plant"), py::arg("frameA"), py::arg("p_AQ_lower"),
            py::arg("p_AQ_upper"), py::arg("frameB"), py::arg("p_BQ"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_autodiff)
        .def(py::init<const MultibodyPlant<AutoDiffXd>*,
                 const Frame<AutoDiffXd>&,
                 const std::optional<math::RigidTransformd>&,
                 const Eigen::Ref<const Eigen::Vector3d>&,
                 const Eigen::Ref<const Eigen::Vector3d>&,
                 const Frame<AutoDiffXd>&, std::optional<Eigen::Vector3d>,
                 systems::Context<AutoDiffXd>*>(),
            py::arg("plant"), py::arg("frameAbar"), py::arg("X_AbarA"),
            py::arg("p_AQ_lower"), py::arg("p_AQ_upper"), py::arg("frameB"),
            py::arg("p_BQ"), py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 9>(), cls_doc.ctor.doc_autodiff_Abar)
        .def("set_bounds", &Class::set_bounds, py::arg("new_lb"),
            py::arg("new_ub"), constraint_doc.set_bounds.doc)
        .def("UpdateLowerBound", &Class::UpdateLowerBound, py::arg("new_lb"),
            constraint_doc.UpdateLowerBound.doc)
        .def("UpdateUpperBound", &Class::UpdateUpperBound, py::arg("new_ub"),
            constraint_doc.UpdateUpperBound.doc);
  }

  {
    using Class = PositionCost;
    constexpr auto& cls_doc = doc.PositionCost;
    py::class_<Class, solvers::Cost
#ifdef PYDRAKE_USE_PYBIND11
,
std::shared_ptr<Class>
#endif
>(
        m, "PositionCost", cls_doc.doc)
        .def(py::init<const MultibodyPlant<double>*, const Frame<double>&,
                 const Eigen::Ref<const Eigen::Vector3d>&, const Frame<double>&,
                 const Eigen::Ref<const Eigen::Vector3d>&,
                 const Eigen::Ref<const Eigen::Matrix3d>&,
                 systems::Context<double>*>(),
            py::arg("plant"), py::arg("frameA"), py::arg("p_AP"),
            py::arg("frameB"), py::arg("p_BQ"), py::arg("C"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_double)
        .def(py::init<const MultibodyPlant<AutoDiffXd>*,
                 const Frame<AutoDiffXd>&,
                 const Eigen::Ref<const Eigen::Vector3d>&,
                 const Frame<AutoDiffXd>&,
                 const Eigen::Ref<const Eigen::Vector3d>&,
                 const Eigen::Ref<const Eigen::Matrix3d>&,
                 systems::Context<AutoDiffXd>*>(),
            py::arg("plant"), py::arg("frameA"), py::arg("p_AP"),
            py::arg("frameB"), py::arg("p_BQ"), py::arg("C"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_autodiff);
  }

  {
    using Class = ComPositionConstraint;
    constexpr auto& cls_doc = doc.ComPositionConstraint;
    py::class_<Class, Constraint
#ifdef PYDRAKE_USE_PYBIND11
,
std::shared_ptr<Class>
#endif
>(
        m, "ComPositionConstraint", cls_doc.doc)
        .def(py::init<const MultibodyPlant<double>*,
                 const std::optional<std::vector<ModelInstanceIndex>>&,
                 const Frame<double>&, systems::Context<double>*>(),
            py::arg("plant"), py::arg("model_instances"),
            py::arg("expressed_frame"), py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 5>(), cls_doc.ctor.doc_double)
        .def(py::init<const MultibodyPlant<AutoDiffXd>*,
                 const std::optional<std::vector<ModelInstanceIndex>>&,
                 const Frame<AutoDiffXd>&, systems::Context<AutoDiffXd>*>(),
            py::arg("plant"), py::arg("model_instances"),
            py::arg("expressed_frame"), py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 5>(), cls_doc.ctor.doc_autodiff);
  }

  {
    using Class = ComInPolyhedronConstraint;
    constexpr auto& cls_doc = doc.ComInPolyhedronConstraint;
    py::class_<Class, Constraint
#ifdef PYDRAKE_USE_PYBIND11
,
std::shared_ptr<Class>
#endif
>(
        m, "ComInPolyhedronConstraint", cls_doc.doc)
        .def(
            py::init<const MultibodyPlant<double>*,
                std::optional<std::vector<ModelInstanceIndex>>,
                const Frame<double>&, const Eigen::Ref<const Eigen::MatrixX3d>&,
                const Eigen::Ref<const Eigen::VectorXd>&,
                const Eigen::Ref<const Eigen::VectorXd>&,
                systems::Context<double>*>(),
            py::arg("plant"), py::arg("model_instances"),
            py::arg("expressed_frame"), py::arg("A"), py::arg("lb"),
            py::arg("ub"), py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_double)
        .def(py::init<const MultibodyPlant<AutoDiffXd>*,
                 std::optional<std::vector<ModelInstanceIndex>>,
                 const Frame<AutoDiffXd>&,
                 const Eigen::Ref<const Eigen::MatrixX3d>&,
                 const Eigen::Ref<const Eigen::VectorXd>&,
                 const Eigen::Ref<const Eigen::VectorXd>&,
                 systems::Context<AutoDiffXd>*>(),
            py::arg("plant"), py::arg("model_instances"),
            py::arg("expressed_frame"), py::arg("A"), py::arg("lb"),
            py::arg("ub"), py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_autodiff);
  }

  {
    using Class = OrientationConstraint;
    constexpr auto& cls_doc = doc.OrientationConstraint;
    py::class_<Class, Constraint
#ifdef PYDRAKE_USE_PYBIND11
,
std::shared_ptr<Class>
#endif
>(
        m, "OrientationConstraint", cls_doc.doc)
        .def(py::init<const MultibodyPlant<double>* const, const Frame<double>&,
                 const math::RotationMatrix<double>&, const Frame<double>&,
                 const math::RotationMatrix<double>&, double,
                 systems::Context<double>*>(),
            py::arg("plant"), py::arg("frameAbar"), py::arg("R_AbarA"),
            py::arg("frameBbar"), py::arg("R_BbarB"), py::arg("theta_bound"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_double)
        .def(py::init<const MultibodyPlant<AutoDiffXd>* const,
                 const Frame<AutoDiffXd>&, const math::RotationMatrix<double>&,
                 const Frame<AutoDiffXd>&, const math::RotationMatrix<double>&,
                 double, systems::Context<AutoDiffXd>*>(),
            py::arg("plant"), py::arg("frameAbar"), py::arg("R_AbarA"),
            py::arg("frameBbar"), py::arg("R_BbarB"), py::arg("theta_bound"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_autodiff);
  }

  {
    using Class = OrientationCost;
    constexpr auto& cls_doc = doc.OrientationCost;
    py::class_<Class, solvers::Cost
#ifdef PYDRAKE_USE_PYBIND11
,
std::shared_ptr<Class>
#endif
>(
        m, "OrientationCost", cls_doc.doc)
        .def(py::init<const MultibodyPlant<double>*, const Frame<double>&,
                 const math::RotationMatrix<double>&, const Frame<double>&,
                 const math::RotationMatrix<double>&, double,
                 systems::Context<double>*>(),
            py::arg("plant"), py::arg("frameAbar"), py::arg("R_AbarA"),
            py::arg("frameBbar"), py::arg("R_BbarB"), py::arg("c"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_double)
        .def(py::init<const MultibodyPlant<AutoDiffXd>*,
                 const Frame<AutoDiffXd>&, const math::RotationMatrix<double>&,
                 const Frame<AutoDiffXd>&, const math::RotationMatrix<double>&,
                 double, systems::Context<AutoDiffXd>*>(),
            py::arg("plant"), py::arg("frameAbar"), py::arg("R_AbarA"),
            py::arg("frameBbar"), py::arg("R_BbarB"), py::arg("c"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_autodiff);
  }

  {
    using Class = UnitQuaternionConstraint;
    constexpr auto& cls_doc = doc.UnitQuaternionConstraint;
    py::class_<Class, Constraint
#ifdef PYDRAKE_USE_PYBIND11
,
std::shared_ptr<Class>
#endif
>(
        m, "UnitQuaternionConstraint", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc);
    m.def("AddUnitQuaternionConstraintOnPlant",
        &AddUnitQuaternionConstraintOnPlant<double>, py::arg("plant"),
        py::arg("q_vars"), py::arg("prog"),
        doc.AddUnitQuaternionConstraintOnPlant.doc);
    m.def("AddUnitQuaternionConstraintOnPlant",
        &AddUnitQuaternionConstraintOnPlant<AutoDiffXd>, py::arg("plant"),
        py::arg("q_vars"), py::arg("prog"),
        doc.AddUnitQuaternionConstraintOnPlant.doc);
  }
  {
    using Class = GlobalInverseKinematics;
    constexpr auto& cls_doc = doc.GlobalInverseKinematics;
    py::class_<Class> global_ik(m, "GlobalInverseKinematics", cls_doc.doc);

    py::class_<GlobalInverseKinematics::Options>(
        global_ik, "Options", cls_doc.Options.doc)
        .def(py::init<>(), cls_doc.Options.ctor.doc)
        .def_rw("num_intervals_per_half_axis",
            &GlobalInverseKinematics::Options::num_intervals_per_half_axis,
            cls_doc.Options.num_intervals_per_half_axis.doc)
        .def_rw("approach", &GlobalInverseKinematics::Options::approach,
            cls_doc.Options.approach.doc)
        .def_rw("interval_binning",
            &GlobalInverseKinematics::Options::interval_binning,
            cls_doc.Options.interval_binning.doc)
        .def_rw("linear_constraint_only",
            &GlobalInverseKinematics::Options::linear_constraint_only,
            cls_doc.Options.linear_constraint_only.doc)
        .def("__repr__", [](const GlobalInverseKinematics::Options& self) {
          return py::str(
              "GlobalInverseKinematics.Options("
              "num_intervals_per_half_axis={}, "
              "approach={}, "
              "interval_binning={}, "
              "linear_constraint_only={})")
              .format(self.num_intervals_per_half_axis, self.approach,
                  self.interval_binning, self.linear_constraint_only);
        });

    py::class_<GlobalInverseKinematics::Polytope3D>(
        global_ik, "Polytope3D", cls_doc.Polytope3D.doc)
        .def(py::init<const Eigen::Ref<const Eigen::MatrixX3d>&,
                 const Eigen::Ref<const Eigen::VectorXd>&>(),
            py::arg("A"), py::arg("b"), cls_doc.Polytope3D.ctor.doc)
        .def_rw("A", &GlobalInverseKinematics::Polytope3D::A,
            cls_doc.Polytope3D.A.doc)
        .def_rw("b", &GlobalInverseKinematics::Polytope3D::b,
            cls_doc.Polytope3D.b.doc)
        .def("__repr__", [](const GlobalInverseKinematics::Polytope3D& self) {
          return py::str("GlobalInverseKinematics.Polytope(A={}, b={})")
              .format(self.A, self.b);
        });

    global_ik
        .def(py::init<const MultibodyPlant<double>&,
                 const GlobalInverseKinematics::Options&>(),
            py::arg("plant"),
            py::arg("options") = GlobalInverseKinematics::Options(),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),  // BR
            cls_doc.ctor.doc)
        .def("prog", &Class::prog, py_rvp::reference_internal, cls_doc.prog.doc)
        .def("get_mutable_prog", &Class::get_mutable_prog,
            py_rvp::reference_internal, cls_doc.get_mutable_prog.doc)
        .def("body_rotation_matrix", &Class::body_rotation_matrix,
            py::arg("body_index"), cls_doc.body_rotation_matrix.doc)
        .def("body_position", &Class::body_position, py::arg("body_index"),
            cls_doc.body_position.doc)
        .def("ReconstructGeneralizedPositionSolution",
            &Class::ReconstructGeneralizedPositionSolution, py::arg("result"),
            cls_doc.ReconstructGeneralizedPositionSolution.doc)
        .def(
            "AddWorldPositionConstraint",
            [](Class* self, BodyIndex body_index, const Eigen::Vector3d& p_BQ,
                const Eigen::Vector3d& box_lb_F,
                const Eigen::Vector3d& box_ub_F,
                const math::RigidTransformd& X_WF) {
              return self->AddWorldPositionConstraint(
                  body_index, p_BQ, box_lb_F, box_ub_F, X_WF);
            },
            py::arg("body_index"), py::arg("p_BQ"), py::arg("box_lb_F"),
            py::arg("box_ub_F"), py::arg("X_WF") = math::RigidTransformd(),
            cls_doc.AddWorldPositionConstraint.doc)
        .def(
            "AddWorldRelativePositionConstraint",
            [](Class* self, BodyIndex body_index_B, const Eigen::Vector3d& p_BQ,
                BodyIndex body_index_A, const Eigen::Vector3d& p_AP,
                const Eigen::Vector3d& box_lb_F,
                const Eigen::Vector3d& box_ub_F,
                const math::RigidTransformd& X_WF) {
              return self->AddWorldRelativePositionConstraint(body_index_B,
                  p_BQ, body_index_A, p_AP, box_lb_F, box_ub_F, X_WF);
            },
            py::arg("body_index_B"), py::arg("p_BQ"), py::arg("body_index_A"),
            py::arg("p_AP"), py::arg("box_lb_F"), py::arg("box_ub_F"),
            py::arg("X_WF") = math::RigidTransformd(),
            cls_doc.AddWorldRelativePositionConstraint.doc)
        .def(
            "AddWorldOrientationConstraint",
            [](Class* self, BodyIndex body_index,
                const Eigen::Quaterniond& desired_orientation,
                double angle_tol) {
              return self->AddWorldOrientationConstraint(
                  body_index, desired_orientation, angle_tol);
            },
            py::arg("body_index"), py::arg("desired_orientation"),
            py::arg("angle_tol"), cls_doc.AddWorldOrientationConstraint.doc)
        .def(
            "AddPostureCost",
            [](Class* self, const Eigen::Ref<const Eigen::VectorXd>& q_desired,
                const Eigen::Ref<const Eigen::VectorXd>& body_position_cost,
                const Eigen::Ref<const Eigen::VectorXd>&
                    body_orientation_cost) {
              return self->AddPostureCost(
                  q_desired, body_position_cost, body_orientation_cost);
            },
            py::arg("q_desired"), py::arg("body_position_cost"),
            py::arg("body_orientation_cost"), cls_doc.AddPostureCost.doc)
        .def(
            "BodyPointInOneOfRegions",
            [](Class* self, BodyIndex body_index,
                const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
                const std::vector<Eigen::Matrix3Xd>& region_vertices) {
              return self->BodyPointInOneOfRegions(
                  body_index, p_BQ, region_vertices);
            },
            py::arg("body_index"), py::arg("p_BQ"), py::arg("region_vertices"),
            cls_doc.BodyPointInOneOfRegions.doc)
        .def(
            "BodySphereInOneOfPolytopes",
            [](Class* self, BodyIndex body_index,
                const Eigen::Ref<const Eigen::Vector3d>& p_BQ, double radius,
                const std::vector<GlobalInverseKinematics::Polytope3D>&
                    polytopes) {
              return self->BodySphereInOneOfPolytopes(
                  body_index, p_BQ, radius, polytopes);
            },
            py::arg("body_index"), py::arg("p_BQ"), py::arg("radius"),
            py::arg("polytopes"), cls_doc.BodySphereInOneOfPolytopes.doc)
        .def("AddJointLimitConstraint", &Class::AddJointLimitConstraint,
            py::arg("body_index"), py::arg("joint_lower_bound"),
            py::arg("joint_upper_bound"),
            py::arg("linear_constraint_approximation") = false,
            cls_doc.AddJointLimitConstraint.doc)
        .def("SetInitialGuess", &Class::SetInitialGuess, py::arg("q"),
            cls_doc.SetInitialGuess.doc);
    // TODO(cohnt): Convert methods that use Polytope3D to use ConvexSets.
  }

  // TODO(SeanCurtis-TRI): Refactor this into its own stand-alone .cc file and
  // re-introduce the inverse_kinematics_py.cc that just assembles the full
  // module.
  internal::DefineIkDifferential(m);

  // NOLINTNEXTLINE(readability/fn_size)
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
