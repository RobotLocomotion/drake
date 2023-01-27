#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/planning/planning_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/planning/body_shape_description.h"
#include "drake/planning/collision_checker_context.h"
#include "drake/planning/collision_checker_params.h"
#include "drake/planning/edge_measure.h"
#include "drake/planning/robot_clearance.h"
#include "drake/planning/robot_collision_type.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefinePlanningCollisionCheckerAccoutrements(py::module m) {
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
    using Class = CollisionCheckerParams;
    constexpr auto& cls_doc = doc.CollisionCheckerParams;
    py::class_<Class> cls(m, "CollisionCheckerParams", cls_doc.doc);
    cls  // BR
        .def(py::init<>())
        .def_property(
            "model",
            [](const Class& self) -> const RobotDiagram<double>* {
              return self.model.get();
            },
            [](Class& self, std::unique_ptr<RobotDiagram<double>> model) {
              self.model = std::move(model);
            },
            cls_doc.model.doc)
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
            cls_doc.self_collision_padding.doc);
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
        .def("SetInEnvironmentCollision", &SetInEnvironmentCollision,
            py::arg("in_environment_collision"),
            doc.SetInEnvironmentCollision.doc)
        .def("SetInSelfCollision", &SetInSelfCollision,
            py::arg("in_self_collision"), doc.SetInSelfCollision.doc);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
