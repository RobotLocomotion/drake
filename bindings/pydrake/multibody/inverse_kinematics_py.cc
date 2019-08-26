#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/symbolic_types_pybind.h"
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

void DoScalarDependentDefinitions(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;

  constexpr auto& doc = pydrake_doc.drake.multibody;
  {
    using Class = AngleBetweenVectorsConstraint;
    constexpr auto& cls_doc = doc.AngleBetweenVectorsConstraint;

    auto cls =
        py::class_<Class>(m, "AngleBetweenVectorsConstraint", cls_doc.doc);
    {
      cls  // BR
          .def(py::init([](const MultibodyPlant<double>* plant,
                            const Frame<double>& frameA,
                            const Eigen::Ref<const Eigen::Vector3d>& a_A,
                            const Frame<double>& frameB,
                            const Eigen::Ref<const Eigen::Vector3d>& b_B,
                            double angle_lower, double angle_upper,
                            systems::Context<double>* context) {
            return std::make_unique<Class>(plant, frameA, a_A, frameB, b_B,
                angle_lower, angle_upper, context);
          }),
              py::arg("plant"), py::arg("frameA"), py::arg("a_A"),
              py::arg("frameB"), py::arg("b_B"), py::arg("angle_lower"),
              py::arg("angle_upper"), py::arg("context"),
              cls_doc.ctor.doc_8args_plant_frameA_a_A_frameB_b_B_angle_lower_angle_upper_context);
    }
    {
      cls  // BR
          .def(py::init([](const MultibodyPlant<AutoDiffXd>* plant,
                            const Frame<AutoDiffXd>& frameA,
                            const Eigen::Ref<const Eigen::Vector3d>& a_A,
                            const Frame<AutoDiffXd>& frameB,
                            const Eigen::Ref<const Eigen::Vector3d>& b_B,
                            double angle_lower, double angle_upper,
                            systems::Context<AutoDiffXd>* context) {
            return std::make_unique<Class>(plant, frameA, a_A, frameB, b_B,
                angle_lower, angle_upper, context);
          }),
              py::arg("plant"), py::arg("frameA"), py::arg("a_A"),
              py::arg("frameB"), py::arg("b_B"), py::arg("angle_lower"),
              py::arg("angle_upper"), py::arg("context"),
              cls_doc.ctor.doc_8args_plant_ad_frameA_a_A_frameB_b_B_angle_lower_angle_upper_context);
    }
  }

  {
    using Class = DistanceConstraint;
    constexpr auto& cls_doc = doc.DistanceConstraint;
    auto cls = py::class_<Class>(m, "DistanceConstraint", cls_doc.doc);

    {
      cls  // BR
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
              cls_doc.ctor.doc_5args_plant_geometry_pair_plant_context_distance_lower_distance_upper
              );
    }
    {
      cls  // BR
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
              cls_doc.ctor.doc_5args_plant_ad_geometry_pair_plant_context_distance_lower_distance_upper
              );
    }
  }

  {
    using Class = GazeTargetConstraint;
    constexpr auto& cls_doc = doc.GazeTargetConstraint;
    auto cls = py::class_<Class>(m, "GazeTargetConstraint", cls_doc.doc);
    {
      cls  // BR
          .def(py::init([](const MultibodyPlant<double>* plant,
                            const Frame<double>& frameA,
                            const Eigen::Ref<const Eigen::Vector3d>& p_AS,
                            const Eigen::Ref<const Eigen::Vector3d>& n_A,
                            const Frame<double>& frameB,
                            const Eigen::Ref<const Eigen::Vector3d>& p_BT,
                            double cone_half_angle,
                            systems::Context<double>* context) {
            return std::make_unique<Class>(plant, frameA, p_AS, n_A, frameB,
                p_BT, cone_half_angle, context);
          }),
              py::arg("plant"), py::arg("frameA"), py::arg("p_AS"),
              py::arg("n_A"), py::arg("frameB"), py::arg("p_BT"),
              py::arg("cone_half_angle"), py::arg("context"),
              cls_doc.ctor.doc_8args_plant_frameA_p_AS_n_A_frameB_p_BT_cone_half_angle_context
              );
    }
    {
      cls  // BR
          .def(py::init([](const MultibodyPlant<AutoDiffXd>* plant,
                            const Frame<AutoDiffXd>& frameA,
                            const Eigen::Ref<const Eigen::Vector3d>& p_AS,
                            const Eigen::Ref<const Eigen::Vector3d>& n_A,
                            const Frame<AutoDiffXd>& frameB,
                            const Eigen::Ref<const Eigen::Vector3d>& p_BT,
                            double cone_half_angle,
                            systems::Context<AutoDiffXd>* context) {
            return std::make_unique<Class>(plant, frameA, p_AS, n_A, frameB,
                p_BT, cone_half_angle, context);
          }),
              py::arg("plant"), py::arg("frameA"), py::arg("p_AS"),
              py::arg("n_A"), py::arg("frameB"), py::arg("p_BT"),
              py::arg("cone_half_angle"), py::arg("context"),
              cls_doc.ctor.doc_8args_plant_ad_frameA_p_AS_n_A_frameB_p_BT_cone_half_angle_context
              );
    }
  }

  {
    using Class = MinimumDistanceConstraint;
    constexpr auto& cls_doc = doc.MinimumDistanceConstraint;
    auto cls = py::class_<Class>(m, "MinimumDistanceConstraint", cls_doc.doc);
    {
      cls  // BR
          .def(
              py::init([](const multibody::MultibodyPlant<double>* const plant,
                           double minimum_distance,
                           systems::Context<double>* plant_context,
                           MinimumDistancePenaltyFunction penalty_function = {},
                           double influence_distance_offset = 1) {
                return std::make_unique<Class>(plant, minimum_distance,
                    plant_context, penalty_function, influence_distance_offset);
              }),
              py::arg("plant"), py::arg("minimum_distance"),
              py::arg("plant_context"), py::arg("penalty_function"),
              py::arg("influence_distance_offset"),
              cls_doc.ctor.doc_5args_plant_minimum_distance_plant_context_penalty_function_influence_distance_offset);
    }
    {
      cls  // BR
          .def(py::init(
                   [](const multibody::MultibodyPlant<AutoDiffXd>* const plant,
                       double minimum_distance,
                       systems::Context<AutoDiffXd>* plant_context,
                       MinimumDistancePenaltyFunction penalty_function = {},
                       double influence_distance_offset = 1) {
                     return std::make_unique<Class>(plant, minimum_distance,
                         plant_context, penalty_function,
                         influence_distance_offset);
                   }),
              py::arg("plant"), py::arg("minimum_distance"),
              py::arg("plant_context"), py::arg("penalty_function"),
              py::arg("influence_distance_offset"),
              cls_doc.ctor.doc_5args_plant_ad_minimum_distance_plant_context_penalty_function_influence_distance_offset);
    }
  }

  {
    using Class = PositionConstraint;
    constexpr auto& cls_doc = doc.PositionConstraint;
    auto cls = py::class_<Class>(m, "PositionConstraint", cls_doc.doc);
    {
      cls  // BR
          .def(py::init([](const MultibodyPlant<double>* plant,
                            const Frame<double>& frameA,
                            const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
                            const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
                            const Frame<double>& frameB,
                            const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
                            systems::Context<double>* context) {
            return std::make_unique<Class>(
                plant, frameA, p_AQ_lower, p_AQ_upper, frameB, p_BQ, context);
          }),
              py::arg("plant"), py::arg("frameA"), py::arg("p_AQ_lower"),
              py::arg("p_AQ_upper"), py::arg("frameB"), py::arg("p_BQ"),
              py::arg("context"),
              cls_doc.ctor.doc_7args_plant_frameA_p_AQ_lower_p_AQ_upper_frameB_p_BQ_context
              );
    }
    {
      cls  // BR
          .def(py::init([](const MultibodyPlant<AutoDiffXd>* plant,
                            const Frame<AutoDiffXd>& frameA,
                            const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
                            const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
                            const Frame<AutoDiffXd>& frameB,
                            const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
                            systems::Context<AutoDiffXd>* context) {
            return std::make_unique<Class>(
                plant, frameA, p_AQ_lower, p_AQ_upper, frameB, p_BQ, context);
          }),
              py::arg("plant"), py::arg("frameA"), py::arg("p_AQ_lower"),
              py::arg("p_AQ_upper"), py::arg("frameB"), py::arg("p_BQ"),
              py::arg("context"),
              cls_doc.ctor.doc_7args_plant_ad_frameA_p_AQ_lower_p_AQ_upper_frameB_p_BQ_context
              );
    }
  }

  {
    using Class = OrientationConstraint;
    constexpr auto& cls_doc = doc.OrientationConstraint;
    auto cls = py::class_<Class>(m, "OrientationConstraint", cls_doc.doc);

    {
      cls  // BR
          .def(py::init(
                   [](const MultibodyPlant<double>* const plant,
                       const Frame<double>& frameAbar,
                       const math::RotationMatrix<double>& R_AbarA,
                       const Frame<double>& frameBbar,
                       const math::RotationMatrix<double>& R_BbarB,
                       double theta_bound, systems::Context<double>* context) {
                     return std::make_unique<Class>(plant, frameAbar, R_AbarA,
                         frameBbar, R_BbarB, theta_bound, context);
                   }),
              py::arg("plant"), py::arg("frameAbar"), py::arg("R_AbarA"),
              py::arg("frameBbar"), py::arg("R_BbarB"), py::arg("theta_bound"),
              py::arg("context"),
              cls_doc.ctor.doc_7args_plant_frameAbar_R_AbarA_frameBbar_R_BbarB_theta_bound_context
              );
    }
    {
      cls  // BR
          .def(py::init([](const MultibodyPlant<AutoDiffXd>* const plant,
                            const Frame<AutoDiffXd>& frameAbar,
                            const math::RotationMatrix<double>& R_AbarA,
                            const Frame<AutoDiffXd>& frameBbar,
                            const math::RotationMatrix<double>& R_BbarB,
                            double theta_bound,
                            systems::Context<AutoDiffXd>* context) {
            return std::make_unique<Class>(plant, frameAbar, R_AbarA, frameBbar,
                R_BbarB, theta_bound, context);
          }),
              py::arg("plant"), py::arg("frameAbar"), py::arg("R_AbarA"),
              py::arg("frameBbar"), py::arg("R_BbarB"), py::arg("theta_bound"),
              py::arg("context"),
              cls_doc.ctor.doc_7args_plant_ad_frameAbar_R_AbarA_frameBbar_R_BbarB_theta_bound_context
              );
    }
  }
}
}  // namespace

PYBIND11_MODULE(inverse_kinematics, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody;

  m.doc() = "InverseKinematics module";

  py::module::import("pydrake.solvers.mathematicalprogram");
  py::module::import("pydrake.math");

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

  DoScalarDependentDefinitions(m);
}

}  // namespace pydrake
}  // namespace drake
