#include "drake/bindings/pydrake/common/wrap_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/planning/planning_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/distance_and_interpolation_provider.h"
#include "drake/planning/scene_graph_collision_checker.h"
#include "drake/planning/unimplemented_collision_checker.h"

namespace drake {
namespace pydrake {
namespace internal {

using multibody::Body;
using multibody::BodyIndex;
using multibody::Frame;

void DefinePlanningCollisionChecker(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::planning;
  constexpr auto& doc = pydrake_doc.drake.planning;

  {
    using Class = CollisionChecker;
    constexpr auto& cls_doc = doc.CollisionChecker;
    py::class_<Class> cls(m, "CollisionChecker", cls_doc.doc);
    cls  // BR
        .def("model", &Class::model, py_rvp::reference_internal,
            cls_doc.model.doc)
        .def("plant", &Class::plant, py_rvp::reference_internal,
            cls_doc.plant.doc)
        .def("get_body", &Class::get_body, py::arg("body_index"),
            py_rvp::reference_internal, cls_doc.get_body.doc)
        .def("robot_model_instances", &Class::robot_model_instances,
            cls_doc.robot_model_instances.doc)
        .def("IsPartOfRobot",
            overload_cast_explicit<bool, const Body<double>&>(
                &Class::IsPartOfRobot),
            py::arg("body"), cls_doc.IsPartOfRobot.doc)
        .def("IsPartOfRobot",
            overload_cast_explicit<bool, BodyIndex>(&Class::IsPartOfRobot),
            py::arg("body_index"), cls_doc.IsPartOfRobot.doc)
        .def("GetZeroConfiguration", &Class::GetZeroConfiguration,
            cls_doc.GetZeroConfiguration.doc)
        .def("num_allocated_contexts", &Class::num_allocated_contexts,
            cls_doc.num_allocated_contexts.doc)
        .def("model_context", &Class::model_context,
            py::arg("context_number") = std::nullopt,
            py_rvp::reference_internal, cls_doc.model_context.doc)
        .def("plant_context", &Class::plant_context,
            py::arg("context_number") = std::nullopt,
            py_rvp::reference_internal, cls_doc.plant_context.doc)
        .def("UpdatePositions", &Class::UpdatePositions,
            py_rvp::reference_internal, py::arg("q"),
            py::arg("context_number") = std::nullopt,
            cls_doc.UpdatePositions.doc)
        .def("UpdateContextPositions", &Class::UpdateContextPositions,
            py::arg("model_context"), py::arg("q"),
            // Keep alive, ownership: `return` keeps `model_context` alive.
            py::keep_alive<0, 2>(), py_rvp::reference,
            cls_doc.UpdateContextPositions.doc)
        .def("MakeStandaloneModelContext", &Class::MakeStandaloneModelContext,
            cls_doc.MakeStandaloneModelContext.doc)
        .def("PerformOperationAgainstAllModelContexts",
            WrapCallbacks(&Class::PerformOperationAgainstAllModelContexts),
            py::arg("operation"),
            cls_doc.PerformOperationAgainstAllModelContexts.doc)
        .def("AddCollisionShape", &Class::AddCollisionShape,
            py::arg("group_name"), py::arg("description"),
            cls_doc.AddCollisionShape.doc)
        .def("AddCollisionShapes",
            py::overload_cast<const std::string&,
                const std::vector<BodyShapeDescription>&>(
                &Class::AddCollisionShapes),
            py::arg("group_name"), py::arg("descriptions"),
            cls_doc.AddCollisionShapes.doc_2args)
        .def("AddCollisionShapes",
            py::overload_cast<const std::map<std::string,
                std::vector<BodyShapeDescription>>&>(
                &Class::AddCollisionShapes),
            py::arg("geometry_groups"), cls_doc.AddCollisionShapes.doc_1args)
        .def("AddCollisionShapeToFrame", &Class::AddCollisionShapeToFrame,
            py::arg("group_name"), py::arg("frameA"), py::arg("shape"),
            py::arg("X_AG"), cls_doc.AddCollisionShapeToFrame.doc)
        .def("AddCollisionShapeToBody", &Class::AddCollisionShapeToBody,
            py::arg("group_name"), py::arg("bodyA"), py::arg("shape"),
            py::arg("X_AG"), cls_doc.AddCollisionShapeToBody.doc)
        .def("GetAllAddedCollisionShapes", &Class::GetAllAddedCollisionShapes,
            cls_doc.GetAllAddedCollisionShapes.doc)
        .def("RemoveAllAddedCollisionShapes",
            py::overload_cast<const std::string&>(
                &Class::RemoveAllAddedCollisionShapes),
            py::arg("group_name"),
            cls_doc.RemoveAllAddedCollisionShapes.doc_1args)
        .def("RemoveAllAddedCollisionShapes",
            py::overload_cast<>(&Class::RemoveAllAddedCollisionShapes),
            cls_doc.RemoveAllAddedCollisionShapes.doc_0args)
        .def("MaybeGetUniformRobotEnvironmentPadding",
            &Class::MaybeGetUniformRobotEnvironmentPadding,
            cls_doc.MaybeGetUniformRobotEnvironmentPadding.doc)
        .def("MaybeGetUniformRobotRobotPadding",
            &Class::MaybeGetUniformRobotRobotPadding,
            cls_doc.MaybeGetUniformRobotRobotPadding.doc)
        .def("GetPaddingBetween",
            overload_cast_explicit<double, BodyIndex, BodyIndex>(
                &Class::GetPaddingBetween),
            py::arg("bodyA_index"), py::arg("bodyB_index"),
            cls_doc.GetPaddingBetween.doc_2args_bodyA_index_bodyB_index)
        .def("GetPaddingBetween",
            overload_cast_explicit<double, const Body<double>&,
                const Body<double>&>(&Class::GetPaddingBetween),
            py::arg("bodyA"), py::arg("bodyB"),
            cls_doc.GetPaddingBetween.doc_2args_bodyA_bodyB)
        .def("SetPaddingBetween",
            py::overload_cast<BodyIndex, BodyIndex, double>(
                &Class::SetPaddingBetween),
            py::arg("bodyA_index"), py::arg("bodyB_index"), py::arg("padding"),
            cls_doc.SetPaddingBetween.doc_3args_bodyA_index_bodyB_index_padding)
        .def("SetPaddingBetween",
            py::overload_cast<const Body<double>&, const Body<double>&, double>(
                &Class::SetPaddingBetween),
            py::arg("bodyA"), py::arg("bodyB"), py::arg("padding"),
            cls_doc.SetPaddingBetween.doc_3args_bodyA_bodyB_padding)
        .def("GetPaddingMatrix", &Class::GetPaddingMatrix,
            py_rvp::reference_internal, cls_doc.GetPaddingMatrix.doc)
        .def("SetPaddingMatrix", &Class::SetPaddingMatrix,
            py::arg("collision_padding"), cls_doc.SetPaddingMatrix.doc)
        .def("GetLargestPadding", &Class::GetLargestPadding,
            cls_doc.GetLargestPadding.doc)
        .def("SetPaddingOneRobotBodyAllEnvironmentPairs",
            &Class::SetPaddingOneRobotBodyAllEnvironmentPairs,
            py::arg("body_index"), py::arg("padding"),
            cls_doc.SetPaddingOneRobotBodyAllEnvironmentPairs.doc)
        .def("SetPaddingAllRobotEnvironmentPairs",
            &Class::SetPaddingAllRobotEnvironmentPairs, py::arg("padding"),
            cls_doc.SetPaddingAllRobotEnvironmentPairs.doc)
        .def("SetPaddingAllRobotRobotPairs",
            &Class::SetPaddingAllRobotRobotPairs, py::arg("padding"),
            cls_doc.SetPaddingAllRobotRobotPairs.doc)
        .def("GetNominalFilteredCollisionMatrix",
            &Class::GetNominalFilteredCollisionMatrix,
            py_rvp::reference_internal,
            cls_doc.GetNominalFilteredCollisionMatrix.doc)
        .def("GetFilteredCollisionMatrix", &Class::GetFilteredCollisionMatrix,
            py_rvp::reference_internal, cls_doc.GetFilteredCollisionMatrix.doc)
        .def("SetCollisionFilterMatrix", &Class::SetCollisionFilterMatrix,
            py::arg("filter_matrix"), cls_doc.SetCollisionFilterMatrix.doc)
        .def("IsCollisionFilteredBetween",
            overload_cast_explicit<bool, BodyIndex, BodyIndex>(
                &Class::IsCollisionFilteredBetween),
            py::arg("bodyA_index"), py::arg("bodyB_index"),
            cls_doc.IsCollisionFilteredBetween
                .doc_2args_bodyA_index_bodyB_index)
        .def("IsCollisionFilteredBetween",
            overload_cast_explicit<bool, const Body<double>&,
                const Body<double>&>(&Class::IsCollisionFilteredBetween),
            py::arg("bodyA"), py::arg("bodyB"),
            cls_doc.IsCollisionFilteredBetween.doc_2args_bodyA_bodyB)
        .def("SetCollisionFilteredBetween",
            py::overload_cast<BodyIndex, BodyIndex, bool>(
                &Class::SetCollisionFilteredBetween),
            py::arg("bodyA_index"), py::arg("bodyB_index"),
            py::arg("filter_collision"),
            cls_doc.SetCollisionFilteredBetween
                .doc_3args_bodyA_index_bodyB_index_filter_collision)
        .def("SetCollisionFilteredBetween",
            py::overload_cast<const Body<double>&, const Body<double>&, bool>(
                &Class::SetCollisionFilteredBetween),
            py::arg("bodyA"), py::arg("bodyB"), py::arg("filter_collision"),
            cls_doc.SetCollisionFilteredBetween
                .doc_3args_bodyA_bodyB_filter_collision)
        .def("SetCollisionFilteredWithAllBodies",
            py::overload_cast<BodyIndex>(
                &Class::SetCollisionFilteredWithAllBodies),
            py::arg("body_index"),
            cls_doc.SetCollisionFilteredWithAllBodies.doc_1args_body_index)
        .def("SetCollisionFilteredWithAllBodies",
            py::overload_cast<const Body<double>&>(
                &Class::SetCollisionFilteredWithAllBodies),
            py::arg("body"),
            cls_doc.SetCollisionFilteredWithAllBodies.doc_1args_body)
        .def("CheckConfigCollisionFree", &Class::CheckConfigCollisionFree,
            py::arg("q"), py::arg("context_number") = std::nullopt,
            cls_doc.CheckConfigCollisionFree.doc)
        .def("CheckContextConfigCollisionFree",
            &Class::CheckContextConfigCollisionFree, py::arg("model_context"),
            py::arg("q"), cls_doc.CheckContextConfigCollisionFree.doc)
        .def("CheckConfigsCollisionFree", &Class::CheckConfigsCollisionFree,
            py::arg("configs"), py::arg("parallelize") = true,
            cls_doc.CheckConfigsCollisionFree.doc)
        .def("SetDistanceAndInterpolationProvider",
            &Class::SetDistanceAndInterpolationProvider, py::arg("provider"),
            cls_doc.SetDistanceAndInterpolationProvider.doc)
        .def("distance_and_interpolation_provider",
            &Class::distance_and_interpolation_provider,
            py_rvp::reference_internal,
            cls_doc.distance_and_interpolation_provider.doc)
        .def("SetConfigurationDistanceFunction",
            WrapCallbacks(&Class::SetConfigurationDistanceFunction),
            py::arg("distance_function"),
            cls_doc.SetConfigurationDistanceFunction.doc)
        .def("ComputeConfigurationDistance",
            &Class::ComputeConfigurationDistance, py::arg("q1"), py::arg("q2"),
            cls_doc.ComputeConfigurationDistance.doc)
        .def("MakeStandaloneConfigurationDistanceFunction",
            &Class::MakeStandaloneConfigurationDistanceFunction,
            cls_doc.MakeStandaloneConfigurationDistanceFunction.doc)
        .def("SetConfigurationInterpolationFunction",
            WrapCallbacks(&Class::SetConfigurationInterpolationFunction),
            py::arg("interpolation_function"),
            cls_doc.SetConfigurationInterpolationFunction.doc)
        .def("InterpolateBetweenConfigurations",
            &Class::InterpolateBetweenConfigurations, py::arg("q1"),
            py::arg("q2"), py::arg("ratio"),
            cls_doc.InterpolateBetweenConfigurations.doc)
        .def("MakeStandaloneConfigurationInterpolationFunction",
            &Class::MakeStandaloneConfigurationInterpolationFunction,
            cls_doc.MakeStandaloneConfigurationInterpolationFunction.doc)
        .def("edge_step_size", &Class::edge_step_size,
            cls_doc.edge_step_size.doc)
        .def("set_edge_step_size", &Class::set_edge_step_size,
            py::arg("edge_step_size"), cls_doc.set_edge_step_size.doc)
        .def("CheckEdgeCollisionFree", &Class::CheckEdgeCollisionFree,
            py::arg("q1"), py::arg("q2"),
            py::arg("context_number") = std::nullopt,
            cls_doc.CheckEdgeCollisionFree.doc)
        .def("CheckContextEdgeCollisionFree",
            &Class::CheckContextEdgeCollisionFree, py::arg("model_context"),
            py::arg("q1"), py::arg("q2"))
        .def("CheckEdgeCollisionFreeParallel",
            &Class::CheckEdgeCollisionFreeParallel, py::arg("q1"),
            py::arg("q2"), py::arg("parallelize") = true,
            cls_doc.CheckEdgeCollisionFreeParallel.doc)
        .def("CheckEdgesCollisionFree", &Class::CheckEdgesCollisionFree,
            py::arg("edges"), py::arg("parallelize") = true,
            cls_doc.CheckEdgesCollisionFree.doc)
        .def("MeasureEdgeCollisionFree", &Class::MeasureEdgeCollisionFree,
            py::arg("q1"), py::arg("q2"),
            py::arg("context_number") = std::nullopt,
            cls_doc.MeasureEdgeCollisionFree.doc)
        .def("MeasureContextEdgeCollisionFree",
            &Class::MeasureContextEdgeCollisionFree, py::arg("model_context"),
            py::arg("q1"), py::arg("q2"),
            cls_doc.MeasureContextEdgeCollisionFree.doc)
        .def("MeasureEdgeCollisionFreeParallel",
            &Class::MeasureEdgeCollisionFreeParallel, py::arg("q1"),
            py::arg("q2"), py::arg("parallelize") = true,
            cls_doc.MeasureEdgeCollisionFreeParallel.doc)
        .def("MeasureEdgesCollisionFree", &Class::MeasureEdgesCollisionFree,
            py::arg("edges"), py::arg("parallelize") = true,
            cls_doc.MeasureEdgesCollisionFree.doc)
        .def("CalcRobotClearance", &Class::CalcRobotClearance, py::arg("q"),
            py::arg("influence_distance"),
            py::arg("context_number") = std::nullopt,
            cls_doc.CalcRobotClearance.doc)
        .def("CalcContextRobotClearance", &Class::CalcContextRobotClearance,
            py::arg("model_context"), py::arg("q"),
            py::arg("influence_distance"),
            cls_doc.CalcContextRobotClearance.doc)
        .def("MaxNumDistances", &Class::MaxNumDistances,
            py::arg("context_number") = std::nullopt,
            cls_doc.MaxNumDistances.doc)
        .def("MaxContextNumDistances", &Class::MaxContextNumDistances,
            py::arg("model_context"), cls_doc.MaxContextNumDistances.doc)
        .def("ClassifyBodyCollisions", &Class::ClassifyBodyCollisions,
            py::arg("q"), py::arg("context_number") = std::nullopt,
            cls_doc.ClassifyBodyCollisions.doc)
        .def("ClassifyContextBodyCollisions",
            &Class::ClassifyContextBodyCollisions, py::arg("model_context"),
            py::arg("q"), cls_doc.ClassifyContextBodyCollisions.doc)
        .def("SupportsParallelChecking", &Class::SupportsParallelChecking,
            cls_doc.SupportsParallelChecking.doc);
    DefClone(&cls);
  }

  {
    using Class = SceneGraphCollisionChecker;
    constexpr auto& cls_doc = doc.SceneGraphCollisionChecker;
    py::class_<Class, CollisionChecker> cls(
        m, "SceneGraphCollisionChecker", cls_doc.doc);
    // TODO(jwnimmer-tri) Bind the __init__(params=...) constructor here once
    // we've solved the unique_ptr vs shared_ptr binding lifetime issue.
    py::object params_ctor = m.attr("CollisionCheckerParams");
    cls  // BR
        .def(py::init([params_ctor](std::unique_ptr<RobotDiagram<double>> model,
                          const py::kwargs& kwargs) {
          // For lifetime management, we need to treat pointer-like arguments
          // separately. Start by creating a Params object in Python with all
          // of the other non-pointer kwargs.
          py::object params_py = params_ctor(**kwargs);
          auto* params = params_py.cast<CollisionCheckerParams*>();
          DRAKE_DEMAND(params != nullptr);
          // Now, transfer ownership of the pointer.
          params->model = std::move(model);
          return std::make_unique<SceneGraphCollisionChecker>(
              std::move(*params));
        }),
            py::kw_only(), py::arg("model"),
            // Keep alive, ownership: `model` keeps `self` alive.
            py::keep_alive<2, 1>(),
            (std::string(cls_doc.ctor.doc) +
                "\n\n"
                "See :class:`pydrake.planning.CollisionCheckerParams` for the "
                "list of properties available here as kwargs.")
                .c_str());
  }

  {
    using Class = UnimplementedCollisionChecker;
    constexpr auto& cls_doc = doc.UnimplementedCollisionChecker;
    py::class_<Class, CollisionChecker> cls(
        m, "UnimplementedCollisionChecker", cls_doc.doc);
    // TODO(jwnimmer-tri) Bind the __init__(params=...) constructor here once
    // we've solved the unique_ptr vs shared_ptr binding lifetime issue.
    py::object params_ctor = m.attr("CollisionCheckerParams");
    cls  // BR
        .def(py::init([params_ctor](std::unique_ptr<RobotDiagram<double>> model,
                          bool supports_parallel_checking,
                          const py::kwargs& kwargs) {
          // For lifetime management, we need to treat pointer-like arguments
          // separately. Start by creating a Params object in Python with all
          // of the other non-pointer kwargs.
          py::object params_py = params_ctor(**kwargs);
          auto* params = params_py.cast<CollisionCheckerParams*>();
          DRAKE_DEMAND(params != nullptr);
          // Now, transfer ownership of the pointer.
          params->model = std::move(model);
          return std::make_unique<UnimplementedCollisionChecker>(
              std::move(*params), supports_parallel_checking);
        }),
            py::kw_only(), py::arg("model"),
            py::arg("supports_parallel_checking"),
            // Keep alive, ownership: `model` keeps `self` alive.
            py::keep_alive<2, 1>(),
            (std::string(cls_doc.ctor.doc) +
                "\n\n"
                "See :class:`pydrake.planning.CollisionCheckerParams` for the "
                "list of properties available here as kwargs.")
                .c_str());
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
