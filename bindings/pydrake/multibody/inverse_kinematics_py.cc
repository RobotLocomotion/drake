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
      py::tuple penalty_tuple(2);
      const bool compute_grad = dpenalty != nullptr;
      penalty_tuple = penalty_function(x, compute_grad);
      *penalty = penalty_tuple[0].cast<double>();
      if (compute_grad) {
        *dpenalty = penalty_tuple[1].cast<double>();
      }
    };
  } else {
    return solvers::MinimumValuePenaltyFunction{};
  }
}

using solvers::Constraint;

PYBIND11_MODULE(inverse_kinematics, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc =
      pydrake_doc_multibody_inverse_kinematics.drake.multibody;
  constexpr auto& constraint_doc = pydrake_doc_solvers.drake.solvers.Constraint;

  m.doc() = "InverseKinematics module";

  py::module::import("pydrake.math");
  py::module::import("pydrake.multibody.plant");
  py::module::import("pydrake.planning");
  py::module::import("pydrake.solvers");

  {
    using Class = InverseKinematics;
    constexpr auto& cls_doc = doc.InverseKinematics;
    py::class_<Class> cls(
        m, "InverseKinematics", py::dynamic_attr(), cls_doc.doc);
    cls.def(py::init<const MultibodyPlant<double>&, bool>(), py::arg("plant"),
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
        .def("AddDistanceConstraint", &Class::AddDistanceConstraint,
            py::arg("geometry_pair"), py::arg("distance_lower"),
            py::arg("distance_upper"), cls_doc.AddDistanceConstraint.doc)
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
    using Ptr = std::shared_ptr<Class>;
    py::class_<Class, Constraint, Ptr>(
        m, "AngleBetweenVectorsConstraint", cls_doc.doc)
        .def(py::init([](const MultibodyPlant<double>* plant,
                          const Frame<double>& frameA,
                          const Eigen::Ref<const Eigen::Vector3d>& a_A,
                          const Frame<double>& frameB,
                          const Eigen::Ref<const Eigen::Vector3d>& b_B,
                          double angle_lower, double angle_upper,
                          systems::Context<double>* plant_context) {
          return std::make_unique<Class>(plant, frameA, a_A, frameB, b_B,
              angle_lower, angle_upper, plant_context);
        }),
            py::arg("plant"), py::arg("frameA"), py::arg("a_A"),
            py::arg("frameB"), py::arg("b_B"), py::arg("angle_lower"),
            py::arg("angle_upper"), py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 9>(), cls_doc.ctor.doc_double)
        .def(py::init([](const MultibodyPlant<AutoDiffXd>* plant,
                          const Frame<AutoDiffXd>& frameA,
                          const Eigen::Ref<const Eigen::Vector3d>& a_A,
                          const Frame<AutoDiffXd>& frameB,
                          const Eigen::Ref<const Eigen::Vector3d>& b_B,
                          double angle_lower, double angle_upper,
                          systems::Context<AutoDiffXd>* plant_context) {
          return std::make_unique<Class>(plant, frameA, a_A, frameB, b_B,
              angle_lower, angle_upper, plant_context);
        }),
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
    using Ptr = std::shared_ptr<Class>;
    py::class_<Class, solvers::Cost, Ptr>(
        m, "AngleBetweenVectorsCost", cls_doc.doc)
        .def(py::init([](const MultibodyPlant<double>* plant,
                          const Frame<double>& frameA,
                          const Eigen::Ref<const Eigen::Vector3d>& a_A,
                          const Frame<double>& frameB,
                          const Eigen::Ref<const Eigen::Vector3d>& b_B,
                          double c, systems::Context<double>* plant_context) {
          return std::make_unique<Class>(
              plant, frameA, a_A, frameB, b_B, c, plant_context);
        }),
            py::arg("plant"), py::arg("frameA"), py::arg("a_A"),
            py::arg("frameB"), py::arg("b_B"), py::arg("c"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_double)
        .def(
            py::init([](const MultibodyPlant<AutoDiffXd>* plant,
                         const Frame<AutoDiffXd>& frameA,
                         const Eigen::Ref<const Eigen::Vector3d>& a_A,
                         const Frame<AutoDiffXd>& frameB,
                         const Eigen::Ref<const Eigen::Vector3d>& b_B, double c,
                         systems::Context<AutoDiffXd>* plant_context) {
              return std::make_unique<Class>(
                  plant, frameA, a_A, frameB, b_B, c, plant_context);
            }),
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
    using Ptr = std::shared_ptr<Class>;
    py::class_<Class, Constraint, Ptr>(
        m, "PointToPointDistanceConstraint", cls_doc.doc)
        .def(py::init([](const multibody::MultibodyPlant<double>* const plant,
                          const multibody::Frame<double>& frame1,
                          const Eigen::Ref<const Eigen::Vector3d>& p_B1P1,
                          const multibody::Frame<double>& frame2,
                          const Eigen::Ref<const Eigen::Vector3d>& p_B2P2,
                          double distance_lower, double distance_upper,
                          systems::Context<double>* plant_context) {
          return std::make_shared<Class>(plant, frame1, p_B1P1, frame2, p_B2P2,
              distance_lower, distance_upper, plant_context);
        }),
            py::arg("plant"), py::arg("frame1"), py::arg("p_B1P1"),
            py::arg("frame2"), py::arg("p_B2P2"), py::arg("distance_lower"),
            py::arg("distance_upper"), py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 9>(), cls_doc.ctor.doc_double)
        .def(py::init(
                 [](const multibody::MultibodyPlant<AutoDiffXd>* const plant,
                     const multibody::Frame<AutoDiffXd>& frame1,
                     const Eigen::Ref<const Eigen::Vector3d>& p_B1P1,
                     const multibody::Frame<AutoDiffXd>& frame2,
                     const Eigen::Ref<const Eigen::Vector3d>& p_B2P2,
                     double distance_lower, double distance_upper,
                     systems::Context<AutoDiffXd>* plant_context) {
                   return std::make_shared<Class>(plant, frame1, p_B1P1, frame2,
                       p_B2P2, distance_lower, distance_upper, plant_context);
                 }),
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
    using Ptr = std::shared_ptr<Class>;
    py::class_<Class, Constraint, Ptr>(
        m, "PointToLineDistanceConstraint", cls_doc.doc)
        .def(py::init([](const multibody::MultibodyPlant<double>* const plant,
                          const multibody::Frame<double>& frame_point,
                          const Eigen::Ref<const Eigen::Vector3d>& p_B1P,
                          const multibody::Frame<double>& frame_line,
                          const Eigen::Ref<const Eigen::Vector3d>& p_B2Q,
                          const Eigen::Ref<const Eigen::Vector3d>& n_B2,
                          double distance_lower, double distance_upper,
                          systems::Context<double>* plant_context) {
          return std::make_shared<Class>(plant, frame_point, p_B1P, frame_line,
              p_B2Q, n_B2, distance_lower, distance_upper, plant_context);
        }),
            py::arg("plant"), py::arg("frame_point"), py::arg("p_B1P"),
            py::arg("frame_line"), py::arg("p_B2Q"), py::arg("n_B2"),
            py::arg("distance_lower"), py::arg("distance_upper"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 10>(), cls_doc.ctor.doc_double)
        .def(py::init(
                 [](const multibody::MultibodyPlant<AutoDiffXd>* const plant,
                     const multibody::Frame<AutoDiffXd>& frame_point,
                     const Eigen::Ref<const Eigen::Vector3d>& p_B1P,
                     const multibody::Frame<AutoDiffXd>& frame_line,
                     const Eigen::Ref<const Eigen::Vector3d>& p_B2Q,
                     const Eigen::Ref<const Eigen::Vector3d>& n_B2,
                     double distance_lower, double distance_upper,
                     systems::Context<AutoDiffXd>* plant_context) {
                   return std::make_shared<Class>(plant, frame_point, p_B1P,
                       frame_line, p_B2Q, n_B2, distance_lower, distance_upper,
                       plant_context);
                 }),
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
    using Ptr = std::shared_ptr<Class>;
    py::class_<Class, Constraint, Ptr>(m, "PolyhedronConstraint", cls_doc.doc)
        .def(py::init([](const multibody::MultibodyPlant<double>* const plant,
                          const multibody::Frame<double>& frameF,
                          const multibody::Frame<double>& frameG,
                          const Eigen::Ref<const Eigen::Matrix3Xd>& p_GP,
                          const Eigen::Ref<const Eigen::MatrixXd>& A,
                          const Eigen::Ref<const Eigen::VectorXd>& b,
                          systems::Context<double>* plant_context) {
          return std::make_shared<Class>(
              plant, frameF, frameG, p_GP, A, b, plant_context);
        }),
            py::arg("plant"), py::arg("frameF"), py::arg("frameG"),
            py::arg("p_GP"), py::arg("A"), py::arg("b"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_double)
        .def(py::init(
                 [](const multibody::MultibodyPlant<AutoDiffXd>* const plant,
                     const multibody::Frame<AutoDiffXd>& frameF,
                     const multibody::Frame<AutoDiffXd>& frameG,
                     const Eigen::Ref<const Eigen::Matrix3Xd>& p_GP,
                     const Eigen::Ref<const Eigen::MatrixXd>& A,
                     const Eigen::Ref<const Eigen::VectorXd>& b,
                     systems::Context<AutoDiffXd>* plant_context) {
                   return std::make_shared<Class>(
                       plant, frameF, frameG, p_GP, A, b, plant_context);
                 }),
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
    using Ptr = std::shared_ptr<Class>;
    py::class_<Class, Constraint, Ptr>(m, "DistanceConstraint", cls_doc.doc)
        .def(py::init([](const multibody::MultibodyPlant<double>* const plant,
                          SortedPair<geometry::GeometryId> geometry_pair,
                          systems::Context<double>* plant_context,
                          double distance_lower, double distance_upper) {
          return std::make_unique<Class>(plant, geometry_pair, plant_context,
              distance_lower, distance_upper);
        }),
            py::arg("plant"), py::arg("geometry_pair"),
            py::arg("plant_context"), py::arg("distance_lower"),
            py::arg("distance_upper"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 4>(), cls_doc.ctor.doc_double)
        .def(py::init(
                 [](const multibody::MultibodyPlant<AutoDiffXd>* const plant,
                     SortedPair<geometry::GeometryId> geometry_pair,
                     systems::Context<AutoDiffXd>* plant_context,
                     double distance_lower, double distance_upper) {
                   return std::make_unique<Class>(plant, geometry_pair,
                       plant_context, distance_lower, distance_upper);
                 }),
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
    using Ptr = std::shared_ptr<Class>;
    py::class_<Class, Constraint, Ptr>(m, "GazeTargetConstraint", cls_doc.doc)
        .def(py::init([](const MultibodyPlant<double>* plant,
                          const Frame<double>& frameA,
                          const Eigen::Ref<const Eigen::Vector3d>& p_AS,
                          const Eigen::Ref<const Eigen::Vector3d>& n_A,
                          const Frame<double>& frameB,
                          const Eigen::Ref<const Eigen::Vector3d>& p_BT,
                          double cone_half_angle,
                          systems::Context<double>* plant_context) {
          return std::make_unique<Class>(plant, frameA, p_AS, n_A, frameB, p_BT,
              cone_half_angle, plant_context);
        }),
            py::arg("plant"), py::arg("frameA"), py::arg("p_AS"),
            py::arg("n_A"), py::arg("frameB"), py::arg("p_BT"),
            py::arg("cone_half_angle"), py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 9>(), cls_doc.ctor.doc_double)
        .def(py::init([](const MultibodyPlant<AutoDiffXd>* plant,
                          const Frame<AutoDiffXd>& frameA,
                          const Eigen::Ref<const Eigen::Vector3d>& p_AS,
                          const Eigen::Ref<const Eigen::Vector3d>& n_A,
                          const Frame<AutoDiffXd>& frameB,
                          const Eigen::Ref<const Eigen::Vector3d>& p_BT,
                          double cone_half_angle,
                          systems::Context<AutoDiffXd>* plant_context) {
          return std::make_unique<Class>(plant, frameA, p_AS, n_A, frameB, p_BT,
              cone_half_angle, plant_context);
        }),
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
    using Ptr = std::shared_ptr<Class>;
    py::class_<Class, Constraint, Ptr> minimum_distance_lower_bound_constraint(
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
        py::init([](const multibody::MultibodyPlant<double>* plant,
                     double bound, systems::Context<double>* plant_context,
                     PyPenaltyFunction penalty_function,
                     double influence_distance_offset) {
          return std::make_unique<MinimumDistanceLowerBoundConstraint>(plant,
              bound, plant_context, UnwrapPyPenaltyFunction(penalty_function),
              influence_distance_offset);
        }),
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
        py::init([](const multibody::MultibodyPlant<AutoDiffXd>* plant,
                     double bound, systems::Context<AutoDiffXd>* plant_context,
                     PyPenaltyFunction penalty_function,
                     double influence_distance_offset) {
          return std::make_unique<MinimumDistanceLowerBoundConstraint>(plant,
              bound, plant_context, UnwrapPyPenaltyFunction(penalty_function),
              influence_distance_offset);
        }),
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
        .def(py::init([](const planning::CollisionChecker* collision_checker,
                          double bound,
                          planning::CollisionCheckerContext*
                              collision_checker_context,
                          PyPenaltyFunction penalty_function,
                          double influence_distance_offset) {
          return std::make_unique<Class>(collision_checker, bound,
              collision_checker_context,
              UnwrapPyPenaltyFunction(penalty_function),
              influence_distance_offset);
        }),
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
    using Ptr = std::shared_ptr<Class>;
    py::class_<Class, Constraint, Ptr> minimum_distance_upper_bound_constraint(
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
        py::init([](const multibody::MultibodyPlant<double>* plant,
                     double bound, systems::Context<double>* plant_context,
                     double influence_distance_offset,
                     PyPenaltyFunction penalty_function) {
          return std::make_unique<MinimumDistanceUpperBoundConstraint>(plant,
              bound, plant_context, influence_distance_offset,
              UnwrapPyPenaltyFunction(penalty_function));
        }),
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
        py::init([](const multibody::MultibodyPlant<AutoDiffXd>* plant,
                     double bound, systems::Context<AutoDiffXd>* plant_context,
                     double influence_distance_offset,
                     PyPenaltyFunction penalty_function) {
          return std::make_unique<MinimumDistanceUpperBoundConstraint>(plant,
              bound, plant_context, influence_distance_offset,
              UnwrapPyPenaltyFunction(penalty_function));
        }),
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
        .def(py::init([](const planning::CollisionChecker* collision_checker,
                          double bound,
                          planning::CollisionCheckerContext*
                              collision_checker_context,
                          double influence_distance_offset,
                          PyPenaltyFunction penalty_function) {
          return std::make_unique<Class>(collision_checker, bound,
              collision_checker_context, influence_distance_offset,
              UnwrapPyPenaltyFunction(penalty_function));
        }),
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
    using Ptr = std::shared_ptr<Class>;
    py::class_<Class, Constraint, Ptr>(m, "PositionConstraint", cls_doc.doc)
        .def(py::init([](const MultibodyPlant<double>* plant,
                          const Frame<double>& frameA,
                          const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
                          const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
                          const Frame<double>& frameB,
                          std::optional<Eigen::Vector3d> p_BQ,
                          systems::Context<double>* plant_context) {
          return std::make_unique<Class>(plant, frameA, p_AQ_lower, p_AQ_upper,
              frameB, p_BQ, plant_context);
        }),
            py::arg("plant"), py::arg("frameA"), py::arg("p_AQ_lower"),
            py::arg("p_AQ_upper"), py::arg("frameB"), py::arg("p_BQ"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_double)
        .def(py::init([](const MultibodyPlant<double>* plant,
                          const Frame<double>& frameAbar,
                          const std::optional<math::RigidTransformd>& X_AbarA,
                          const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
                          const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
                          const Frame<double>& frameB,
                          std::optional<Eigen::Vector3d> p_BQ,
                          systems::Context<double>* plant_context) {
          return std::make_unique<Class>(plant, frameAbar, X_AbarA, p_AQ_lower,
              p_AQ_upper, frameB, p_BQ, plant_context);
        }),
            py::arg("plant"), py::arg("frameAbar"), py::arg("X_AbarA"),
            py::arg("p_AQ_lower"), py::arg("p_AQ_upper"), py::arg("frameB"),
            py::arg("p_BQ"), py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 9>(), cls_doc.ctor.doc_double_Abar)
        .def(py::init([](const MultibodyPlant<AutoDiffXd>* plant,
                          const Frame<AutoDiffXd>& frameA,
                          const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
                          const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
                          const Frame<AutoDiffXd>& frameB,
                          std::optional<Eigen::Vector3d> p_BQ,
                          systems::Context<AutoDiffXd>* plant_context) {
          return std::make_unique<Class>(plant, frameA, p_AQ_lower, p_AQ_upper,
              frameB, p_BQ, plant_context);
        }),
            py::arg("plant"), py::arg("frameA"), py::arg("p_AQ_lower"),
            py::arg("p_AQ_upper"), py::arg("frameB"), py::arg("p_BQ"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_autodiff)
        .def(py::init([](const MultibodyPlant<AutoDiffXd>* plant,
                          const Frame<AutoDiffXd>& frameAbar,
                          const std::optional<math::RigidTransformd>& X_AbarA,
                          const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
                          const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
                          const Frame<AutoDiffXd>& frameB,
                          std::optional<Eigen::Vector3d> p_BQ,
                          systems::Context<AutoDiffXd>* plant_context) {
          return std::make_unique<Class>(plant, frameAbar, X_AbarA, p_AQ_lower,
              p_AQ_upper, frameB, p_BQ, plant_context);
        }),
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
    using Ptr = std::shared_ptr<Class>;
    py::class_<Class, solvers::Cost, Ptr>(m, "PositionCost", cls_doc.doc)
        .def(py::init([](const MultibodyPlant<double>* plant,
                          const Frame<double>& frameA,
                          const Eigen::Ref<const Eigen::Vector3d>& p_AP,
                          const Frame<double>& frameB,
                          const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
                          const Eigen::Ref<const Eigen::Matrix3d>& C,
                          systems::Context<double>* plant_context) {
          return std::make_unique<Class>(
              plant, frameA, p_AP, frameB, p_BQ, C, plant_context);
        }),
            py::arg("plant"), py::arg("frameA"), py::arg("p_AP"),
            py::arg("frameB"), py::arg("p_BQ"), py::arg("C"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_double)
        .def(py::init([](const MultibodyPlant<AutoDiffXd>* plant,
                          const Frame<AutoDiffXd>& frameA,
                          const Eigen::Ref<const Eigen::Vector3d>& p_AP,
                          const Frame<AutoDiffXd>& frameB,
                          const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
                          const Eigen::Ref<const Eigen::Matrix3d>& C,
                          systems::Context<AutoDiffXd>* plant_context) {
          return std::make_unique<Class>(
              plant, frameA, p_AP, frameB, p_BQ, C, plant_context);
        }),
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
    using Ptr = std::shared_ptr<Class>;
    py::class_<Class, Constraint, Ptr>(m, "ComPositionConstraint", cls_doc.doc)
        .def(py::init([](const MultibodyPlant<double>* plant,
                          const std::optional<std::vector<ModelInstanceIndex>>&
                              model_instances,
                          const Frame<double>& expressed_frame,
                          systems::Context<double>* plant_context) {
          return std::make_unique<Class>(
              plant, model_instances, expressed_frame, plant_context);
        }),
            py::arg("plant"), py::arg("model_instances"),
            py::arg("expressed_frame"), py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 5>(), cls_doc.ctor.doc_double)
        .def(py::init([](const MultibodyPlant<AutoDiffXd>* plant,
                          const std::optional<std::vector<ModelInstanceIndex>>&
                              model_instances,
                          const Frame<AutoDiffXd>& expressed_frame,
                          systems::Context<AutoDiffXd>* plant_context) {
          return std::make_unique<Class>(
              plant, model_instances, expressed_frame, plant_context);
        }),
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
    using Ptr = std::shared_ptr<Class>;
    py::class_<Class, Constraint, Ptr>(
        m, "ComInPolyhedronConstraint", cls_doc.doc)
        .def(py::init([](const MultibodyPlant<double>* plant,
                          std::optional<std::vector<ModelInstanceIndex>>
                              model_instances,
                          const Frame<double>& expressed_frame,
                          const Eigen::Ref<const Eigen::MatrixX3d>& A,
                          const Eigen::Ref<const Eigen::VectorXd>& lb,
                          const Eigen::Ref<const Eigen::VectorXd>& ub,
                          systems::Context<double>* plant_context) {
          return std::make_unique<Class>(plant, model_instances,
              expressed_frame, A, lb, ub, plant_context);
        }),
            py::arg("plant"), py::arg("model_instances"),
            py::arg("expressed_frame"), py::arg("A"), py::arg("lb"),
            py::arg("ub"), py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_double)
        .def(py::init([](const MultibodyPlant<AutoDiffXd>* plant,
                          std::optional<std::vector<ModelInstanceIndex>>
                              model_instances,
                          const Frame<AutoDiffXd>& expressed_frame,
                          const Eigen::Ref<const Eigen::MatrixX3d>& A,
                          const Eigen::Ref<const Eigen::VectorXd>& lb,
                          const Eigen::Ref<const Eigen::VectorXd>& ub,
                          systems::Context<AutoDiffXd>* plant_context) {
          return std::make_unique<Class>(plant, model_instances,
              expressed_frame, A, lb, ub, plant_context);
        }),
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
    using Ptr = std::shared_ptr<Class>;
    py::class_<Class, Constraint, Ptr>(m, "OrientationConstraint", cls_doc.doc)
        .def(py::init([](const MultibodyPlant<double>* const plant,
                          const Frame<double>& frameAbar,
                          const math::RotationMatrix<double>& R_AbarA,
                          const Frame<double>& frameBbar,
                          const math::RotationMatrix<double>& R_BbarB,
                          double theta_bound,
                          systems::Context<double>* plant_context) {
          return std::make_unique<Class>(plant, frameAbar, R_AbarA, frameBbar,
              R_BbarB, theta_bound, plant_context);
        }),
            py::arg("plant"), py::arg("frameAbar"), py::arg("R_AbarA"),
            py::arg("frameBbar"), py::arg("R_BbarB"), py::arg("theta_bound"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_double)
        .def(py::init([](const MultibodyPlant<AutoDiffXd>* const plant,
                          const Frame<AutoDiffXd>& frameAbar,
                          const math::RotationMatrix<double>& R_AbarA,
                          const Frame<AutoDiffXd>& frameBbar,
                          const math::RotationMatrix<double>& R_BbarB,
                          double theta_bound,
                          systems::Context<AutoDiffXd>* plant_context) {
          return std::make_unique<Class>(plant, frameAbar, R_AbarA, frameBbar,
              R_BbarB, theta_bound, plant_context);
        }),
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
    using Ptr = std::shared_ptr<Class>;
    py::class_<Class, solvers::Cost, Ptr>(m, "OrientationCost", cls_doc.doc)
        .def(py::init([](const MultibodyPlant<double>* plant,
                          const Frame<double>& frameAbar,
                          const math::RotationMatrix<double>& R_AbarA,
                          const Frame<double>& frameBbar,
                          const math::RotationMatrix<double>& R_BbarB, double c,
                          systems::Context<double>* plant_context) {
          return std::make_unique<Class>(
              plant, frameAbar, R_AbarA, frameBbar, R_BbarB, c, plant_context);
        }),
            py::arg("plant"), py::arg("frameAbar"), py::arg("R_AbarA"),
            py::arg("frameBbar"), py::arg("R_BbarB"), py::arg("c"),
            py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 8>(), cls_doc.ctor.doc_double)
        .def(py::init([](const MultibodyPlant<AutoDiffXd>* plant,
                          const Frame<AutoDiffXd>& frameAbar,
                          const math::RotationMatrix<double>& R_AbarA,
                          const Frame<AutoDiffXd>& frameBbar,
                          const math::RotationMatrix<double>& R_BbarB, double c,
                          systems::Context<AutoDiffXd>* plant_context) {
          return std::make_unique<Class>(
              plant, frameAbar, R_AbarA, frameBbar, R_BbarB, c, plant_context);
        }),
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
    using Ptr = std::shared_ptr<Class>;
    py::class_<Class, Constraint, Ptr>(
        m, "UnitQuaternionConstraint", cls_doc.doc)
        .def(py::init([]() { return std::make_unique<Class>(); }),
            cls_doc.ctor.doc);
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
        .def_readwrite("num_intervals_per_half_axis",
            &GlobalInverseKinematics::Options::num_intervals_per_half_axis,
            cls_doc.Options.num_intervals_per_half_axis.doc)
        .def_readwrite("approach", &GlobalInverseKinematics::Options::approach,
            cls_doc.Options.approach.doc)
        .def_readwrite("interval_binning",
            &GlobalInverseKinematics::Options::interval_binning,
            cls_doc.Options.interval_binning.doc)
        .def_readwrite("linear_constraint_only",
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
        .def_readwrite("A", &GlobalInverseKinematics::Polytope3D::A,
            cls_doc.Polytope3D.A.doc)
        .def_readwrite("b", &GlobalInverseKinematics::Polytope3D::b,
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
