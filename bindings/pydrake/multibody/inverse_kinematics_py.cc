#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/sorted_pair_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h"
#include "drake/multibody/inverse_kinematics/distance_constraint.h"
#include "drake/multibody/inverse_kinematics/gaze_target_constraint.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/inverse_kinematics/minimum_distance_constraint.h"
#include "drake/multibody/inverse_kinematics/orientation_constraint.h"
#include "drake/multibody/inverse_kinematics/position_constraint.h"

namespace drake {
namespace pydrake {
namespace {

using solvers::Constraint;
constexpr char ctor_doc_ad[] =
    "Overloaded constructor. Constructs the constraint using "
    "MultibodyPlant<AutoDiffXd>";

PYBIND11_MODULE(inverse_kinematics, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody;

  m.doc() = "InverseKinematics module";

  py::module::import("pydrake.math");
  py::module::import("pydrake.multibody.plant");
  py::module::import("pydrake.solvers.mathematicalprogram");

  {
    using Class = InverseKinematics;
    constexpr auto& cls_doc = doc.InverseKinematics;
    py::class_<Class>(m, "InverseKinematics", cls_doc.doc)
        .def(py::init<const MultibodyPlant<double>&>(), py::arg("plant"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),  // BR
            cls_doc.ctor.doc_1args)
        .def(py::init<const MultibodyPlant<double>&,
                 systems::Context<double>*>(),
            py::arg("plant"), py::arg("plant_context"),
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),  // BR
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 3>(),  // BR
            cls_doc.ctor.doc_2args)
        .def("AddPositionConstraint", &Class::AddPositionConstraint,
            py::arg("frameB"), py::arg("p_BQ"), py::arg("frameA"),
            py::arg("p_AQ_lower"), py::arg("p_AQ_upper"),
            cls_doc.AddPositionConstraint.doc)
        .def("AddOrientationConstraint", &Class::AddOrientationConstraint,
            py::arg("frameAbar"), py::arg("R_AbarA"), py::arg("frameBbar"),
            py::arg("R_BbarB"), py::arg("theta_bound"),
            cls_doc.AddOrientationConstraint.doc)
        .def("AddGazeTargetConstraint", &Class::AddGazeTargetConstraint,
            py::arg("frameA"), py::arg("p_AS"), py::arg("n_A"),
            py::arg("frameB"), py::arg("p_BT"), py::arg("cone_half_angle"),
            cls_doc.AddGazeTargetConstraint.doc)
        .def("AddAngleBetweenVectorsConstraint",
            &Class::AddAngleBetweenVectorsConstraint, py::arg("frameA"),
            py::arg("na_A"), py::arg("frameB"), py::arg("nb_B"),
            py::arg("angle_lower"), py::arg("angle_upper"),
            cls_doc.AddAngleBetweenVectorsConstraint.doc)
        .def("AddMinimumDistanceConstraint",
            &Class::AddMinimumDistanceConstraint, py::arg("minimum_distance"),
            py::arg("threshold_distance") = 1.0,
            cls_doc.AddMinimumDistanceConstraint.doc)
        .def("q", &Class::q, cls_doc.q.doc)
        .def("prog", &Class::prog, py_reference_internal, cls_doc.prog.doc)
        .def("get_mutable_prog", &Class::get_mutable_prog,
            py_reference_internal, cls_doc.get_mutable_prog.doc);
  }
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
            py::keep_alive<1, 9>(), cls_doc.ctor.doc)
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
            py::keep_alive<1, 9>(), ctor_doc_ad);
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
            py::keep_alive<1, 4>(), cls_doc.ctor.doc)
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
            py::keep_alive<1, 4>(), ctor_doc_ad);
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
            py::keep_alive<1, 9>(), cls_doc.ctor.doc)
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
            py::keep_alive<1, 9>(), ctor_doc_ad);
  }

  {
    using Class = MinimumDistanceConstraint;
    constexpr auto& cls_doc = doc.MinimumDistanceConstraint;
    using Ptr = std::shared_ptr<Class>;
    py::class_<Class, Constraint, Ptr>(
        m, "MinimumDistanceConstraint", cls_doc.doc)
        .def(py::init([](const multibody::MultibodyPlant<double>* const plant,
                          double minimum_distance,
                          systems::Context<double>* plant_context,
                          MinimumDistancePenaltyFunction penalty_function,
                          double influence_distance_offset) {
          return std::make_unique<Class>(plant, minimum_distance, plant_context,
              penalty_function, influence_distance_offset);
        }),
            py::arg("plant"), py::arg("minimum_distance"),
            py::arg("plant_context"),
            py::arg("penalty_function") = MinimumDistancePenaltyFunction{},
            py::arg("influence_distance_offset") = 1,
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 4>(), cls_doc.ctor.doc)
        .def(py::init(
                 [](const multibody::MultibodyPlant<AutoDiffXd>* const plant,
                     double minimum_distance,
                     systems::Context<AutoDiffXd>* plant_context,
                     MinimumDistancePenaltyFunction penalty_function,
                     double influence_distance_offset) {
                   return std::make_unique<Class>(plant, minimum_distance,
                       plant_context, penalty_function,
                       influence_distance_offset);
                 }),
            py::arg("plant"), py::arg("minimum_distance"),
            py::arg("plant_context"),
            py::arg("penalty_function") = MinimumDistancePenaltyFunction{},
            py::arg("influence_distance_offset") = 1,
            // Keep alive, reference: `self` keeps `plant` alive.
            py::keep_alive<1, 2>(),
            // Keep alive, reference: `self` keeps `plant_context` alive.
            py::keep_alive<1, 4>(), ctor_doc_ad);
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
                          const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
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
            py::keep_alive<1, 8>(), cls_doc.ctor.doc)
        .def(py::init([](const MultibodyPlant<AutoDiffXd>* plant,
                          const Frame<AutoDiffXd>& frameA,
                          const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
                          const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
                          const Frame<AutoDiffXd>& frameB,
                          const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
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
            py::keep_alive<1, 8>(), ctor_doc_ad);
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
            py::keep_alive<1, 8>(), cls_doc.ctor.doc)
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
            py::keep_alive<1, 8>(), ctor_doc_ad);
  }
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
