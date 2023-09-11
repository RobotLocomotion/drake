#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/planning/planning_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/planning/body_shape_description.h"
#include "drake/planning/collision_checker_context.h"
#include "drake/planning/collision_checker_params.h"
#include "drake/planning/distance_and_interpolation_provider.h"
#include "drake/planning/edge_measure.h"
#include "drake/planning/linear_distance_and_interpolation_provider.h"
#include "drake/planning/robot_clearance.h"
#include "drake/planning/robot_collision_type.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefinePlanningCollisionCheckerInterfaceTypes(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning;
  constexpr auto& doc = pydrake_doc.drake.planning;

  {
    using Class = BodyShapeDescription;
    constexpr auto& cls_doc = doc.BodyShapeDescription;
    py::class_<Class> cls(m, "BodyShapeDescription", cls_doc.doc);
    cls  // BR
        .def(py::init<const geometry::Shape&, const math::RigidTransformd&,
                 std::string, std::string>(),
            py::arg("shape"), py::arg("X_BS"), py::arg("model_instance_name"),
            py::arg("body_name"), cls_doc.ctor.doc)
        .def(py::init<const Class&>(), py::arg("other"))
        .def("shape", &Class::shape, py_rvp::reference_internal,
            cls_doc.shape.doc)
        .def("pose_in_body", &Class::pose_in_body, cls_doc.pose_in_body.doc)
        .def("model_instance_name", &Class::model_instance_name,
            cls_doc.model_instance_name.doc)
        .def("body_name", &Class::body_name, cls_doc.body_name.doc);
    DefCopyAndDeepCopy(&cls);
  }

  m.def("MakeBodyShapeDescription", &MakeBodyShapeDescription, py::arg("plant"),
      py::arg("plant_context"), py::arg("geometry_id"),
      doc.MakeBodyShapeDescription.doc);

  {
    using Class = CollisionCheckerContext;
    constexpr auto& cls_doc = doc.CollisionCheckerContext;
    py::class_<Class, std::shared_ptr<Class>> cls(
        m, "CollisionCheckerContext", cls_doc.doc);
    cls  // BR
        .def(py::init<const RobotDiagram<double>*>(), py::arg("model"),
            // Keep alive, reference: `self` keeps `model` alive.
            py::keep_alive<1, 2>(), cls_doc.ctor.doc)
        .def("model_context", &Class::model_context, py_rvp::reference_internal,
            cls_doc.model_context.doc)
        .def("plant_context", &Class::plant_context, py_rvp::reference_internal,
            cls_doc.plant_context.doc)
        .def("scene_graph_context", &Class::scene_graph_context,
            py_rvp::reference_internal, cls_doc.scene_graph_context.doc)
        .def("GetQueryObject", &Class::GetQueryObject,
            py_rvp::reference_internal, cls_doc.GetQueryObject.doc);
    DefClone(&cls);
  }

  {
    using Class = DistanceAndInterpolationProvider;
    constexpr auto& cls_doc = doc.DistanceAndInterpolationProvider;
    py::class_<Class, std::shared_ptr<Class>> cls(
        m, "DistanceAndInterpolationProvider", cls_doc.doc);
    cls  // BR
        .def("ComputeConfigurationDistance",
            &Class::ComputeConfigurationDistance,
            cls_doc.ComputeConfigurationDistance.doc)
        .def("InterpolateBetweenConfigurations",
            &Class::InterpolateBetweenConfigurations,
            cls_doc.InterpolateBetweenConfigurations.doc);
  }

  {
    using Class = LinearDistanceAndInterpolationProvider;
    constexpr auto& cls_doc = doc.LinearDistanceAndInterpolationProvider;
    py::class_<Class, DistanceAndInterpolationProvider,
        std::shared_ptr<LinearDistanceAndInterpolationProvider>>
        cls(m, "LinearDistanceAndInterpolationProvider", cls_doc.doc);
    cls  // BR
        .def(py::init<const drake::multibody::MultibodyPlant<double>&>(),
            py::arg("plant"), cls_doc.ctor.doc_1args_plant)
        .def(py::init<const drake::multibody::MultibodyPlant<double>&,
                 const Eigen::VectorXd&>(),
            py::arg("plant"), py::arg("distance_weights"),
            cls_doc.ctor.doc_2args_plant_distance_weights)
        .def(py::init<const drake::multibody::MultibodyPlant<double>&,
                 const std::map<drake::multibody::JointIndex,
                     Eigen::VectorXd>&>(),
            py::arg("plant"), py::arg("joint_distance_weights"),
            cls_doc.ctor.doc_2args_plant_joint_distance_weights)
        .def("distance_weights", &Class::distance_weights,
            py_rvp::reference_internal, cls_doc.distance_weights.doc)
        .def("quaternion_dof_start_indices",
            &Class::quaternion_dof_start_indices,
            cls_doc.quaternion_dof_start_indices.doc);
  }

  {
    using Class = CollisionCheckerParams;
    constexpr auto& cls_doc = doc.CollisionCheckerParams;
    py::class_<Class> cls(m, "CollisionCheckerParams", cls_doc.doc);
    cls  // BR
        .def(py::init<>())
        .def(ParamInit<Class>())
        .def_property(
            "model",
            [](const Class& self) -> const RobotDiagram<double>* {
              return self.model.get();
            },
            [](Class& self, std::unique_ptr<RobotDiagram<double>> model) {
              self.model = std::move(model);
            },
            cls_doc.model.doc)
        .def_readwrite("distance_and_interpolation_provider",
            &Class::distance_and_interpolation_provider,
            cls_doc.distance_and_interpolation_provider.doc)
        .def_readwrite("robot_model_instances", &Class::robot_model_instances,
            cls_doc.robot_model_instances.doc)
        .def_readwrite("configuration_distance_function",
            &Class::configuration_distance_function,
            cls_doc.configuration_distance_function.doc)
        .def_readwrite("edge_step_size", &Class::edge_step_size,
            cls_doc.edge_step_size.doc)
        .def_readwrite("env_collision_padding", &Class::env_collision_padding,
            cls_doc.env_collision_padding.doc)
        .def_readwrite("self_collision_padding", &Class::self_collision_padding,
            cls_doc.self_collision_padding.doc)
        .def_readwrite("implicit_context_parallelism",
            &Class::implicit_context_parallelism,
            cls_doc.implicit_context_parallelism.doc);
    // N.B. Any time we bind new CollisionCheckerParams fields that have pointer
    // semantics (e.g., std::unique_ptr), we should revisit the CollisionChecker
    // constructor bindings (e.g., SceneGraphCollisionChecker) and fix up their
    // keep_alive<> annotations.
  }

  {
    using Class = EdgeMeasure;
    constexpr auto& cls_doc = doc.EdgeMeasure;
    py::class_<Class> cls(m, "EdgeMeasure", cls_doc.doc);
    cls  // BR
        .def(py::init<double, double>(), py::arg("distance"), py::arg("alpha"),
            cls_doc.ctor.doc)
        .def(py::init<const Class&>(), py::arg("other"))
        .def("completely_free", &Class::completely_free,
            cls_doc.completely_free.doc)
        .def("partially_free", &Class::partially_free,
            cls_doc.partially_free.doc)
        .def("distance", &Class::distance, cls_doc.distance.doc)
        .def("alpha", &Class::alpha, cls_doc.alpha.doc)
        .def("alpha_or", &Class::alpha_or, py::arg("default_value"),
            cls_doc.alpha_or.doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = RobotClearance;
    constexpr auto& cls_doc = doc.RobotClearance;
    py::class_<Class> cls(m, "RobotClearance", cls_doc.doc);
    cls  // BR
        .def(py::init<int>(), py::arg("num_positions"), cls_doc.ctor.doc)
        .def(py::init<const Class&>(), py::arg("other"))
        .def("size", &Class::size, cls_doc.size.doc)
        .def("num_positions", &Class::num_positions, cls_doc.num_positions.doc)
        .def("robot_indices", &Class::robot_indices, cls_doc.robot_indices.doc)
        .def("other_indices", &Class::other_indices, cls_doc.other_indices.doc)
        .def("collision_types", &Class::collision_types,
            cls_doc.collision_types.doc)
        .def("distances", &Class::distances, py_rvp::reference_internal,
            cls_doc.distances.doc)
        .def("jacobians", &Class::jacobians, py_rvp::reference_internal,
            cls_doc.jacobians.doc)
        .def("mutable_jacobians", &Class::mutable_jacobians,
            py_rvp::reference_internal, cls_doc.mutable_jacobians.doc)
        .def("Reserve", &Class::Reserve, py::arg("size"), cls_doc.Reserve.doc)
        .def("Append", &Class::Append, py::arg("robot_index"),
            py::arg("other_index"), py::arg("collision_type"),
            py::arg("distance"), py::arg("jacobian"), cls_doc.Append.doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = RobotCollisionType;
    constexpr auto& cls_doc = doc.RobotCollisionType;
    py::enum_<Class> cls(m, "RobotCollisionType", cls_doc.doc);
    cls  // BR
        .value("kNoCollision", Class::kNoCollision, cls_doc.kNoCollision.doc)
        .value("kEnvironmentCollision", Class::kEnvironmentCollision,
            cls_doc.kEnvironmentCollision.doc)
        .value(
            "kSelfCollision", Class::kSelfCollision, cls_doc.kSelfCollision.doc)
        .value("kEnvironmentAndSelfCollision",
            Class::kEnvironmentAndSelfCollision,
            cls_doc.kEnvironmentAndSelfCollision.doc)
        // We don't bind the un-Pythonic SetInEnvironmentCollision and
        // SetInSelfCollision. Instead, we'll define some Python-only sugar.
        .def(
            "MakeUpdated",
            [](const Class& self, std::optional<bool> in_environment_collision,
                std::optional<bool> in_self_collision) -> Class {
              Class result = self;
              if (in_environment_collision.has_value()) {
                result = SetInEnvironmentCollision(
                    result, *in_environment_collision);
              }
              if (in_self_collision.has_value()) {
                result = SetInSelfCollision(result, *in_self_collision);
              }
              return result;
            },
            py::kw_only(), py::arg("in_environment_collision") = py::none(),
            py::arg("in_self_collision") = py::none(),
            "Returns the RobotCollisionType value that reflects an "
            "incrementally different collision status versus the current"
            " value; the ``self`` object is unchanged. "
            "If ``in_environment_collision`` and/or ``in_self_collision`` is "
            "given, the return value will reflect those given value(s). "
            "This function subsumes the C++ free functions "
            "``SetInEnvironmentCollision()`` and ``SetInSelfCollision()``.");
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
