/* @file This includes the SceneGraph class and the major components of its
 API: SceneGraphInspector and QueryObject for examining its state and performing
 queries, and the query result types as well. They can be found in the
 pydrake.geometry module. */

#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/scene_graph.h"

namespace drake {
namespace pydrake {
namespace {

using systems::Context;
using systems::LeafSystem;

void DoScalarIndependentDefinitions(py::module m) {
  constexpr auto& doc = pydrake_doc.drake.geometry;

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;

  // HydroelasticContactRepresentation enumeration
  {
    using Class = HydroelasticContactRepresentation;
    constexpr auto& cls_doc = doc.HydroelasticContactRepresentation;
    py::enum_<Class>(m, "HydroelasticContactRepresentation", cls_doc.doc)
        .value("kTriangle", Class::kTriangle, cls_doc.kTriangle.doc)
        .value("kPolygon", Class::kPolygon, cls_doc.kPolygon.doc);
  }
}

template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = GetPyParam<T>();
  constexpr auto& doc = pydrake_doc.drake.geometry;

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;

  //  SceneGraphInspector
  {
    using Class = SceneGraphInspector<T>;
    constexpr auto& cls_doc = doc.SceneGraphInspector;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "SceneGraphInspector", param, cls_doc.doc);
    cls  // BR
         // Scene-graph wide data.
        .def("num_sources", &Class::num_sources, cls_doc.num_sources.doc)
        .def("num_frames", &Class::num_frames, cls_doc.num_frames.doc)
        .def("GetAllSourceIds", &Class::GetAllSourceIds,
            cls_doc.GetAllSourceIds.doc)
        .def("GetAllFrameIds", &Class::GetAllFrameIds,
            cls_doc.GetAllFrameIds.doc)
        .def("world_frame_id", &Class::world_frame_id,
            cls_doc.world_frame_id.doc)
        .def("num_geometries", &Class::num_geometries,
            cls_doc.num_geometries.doc)
        .def("GetAllGeometryIds", &Class::GetAllGeometryIds,
            cls_doc.GetAllGeometryIds.doc)
        .def("GetGeometryIds", &Class::GetGeometryIds, py::arg("geometry_set"),
            py::arg("role") = std::nullopt, cls_doc.GetGeometryIds.doc)
        .def("NumGeometriesWithRole", &Class::NumGeometriesWithRole,
            py::arg("role"), cls_doc.NumGeometriesWithRole.doc)
        .def("NumDynamicGeometries", &Class::NumDynamicGeometries,
            cls_doc.NumDynamicGeometries.doc)
        .def("NumAnchoredGeometries", &Class::NumAnchoredGeometries,
            cls_doc.NumAnchoredGeometries.doc)
        .def("GetCollisionCandidates", &Class::GetCollisionCandidates,
            cls_doc.GetCollisionCandidates.doc)
        // Sources and source-related data.
        .def("SourceIsRegistered", &Class::SourceIsRegistered,
            py::arg("source_id"), cls_doc.SourceIsRegistered.doc)
        .def("GetName",
            overload_cast_explicit<const std::string&, SourceId>(
                &Class::GetName),
            py_rvp::reference_internal, py::arg("source_id"),
            cls_doc.GetName.doc_1args_source_id)
        .def("NumFramesForSource", &Class::NumFramesForSource,
            py::arg("source_id"), cls_doc.NumFramesForSource.doc)
        .def("FramesForSource", &Class::FramesForSource, py::arg("source_id"),
            cls_doc.FramesForSource.doc)
        // Frames and their properties.
        .def("BelongsToSource",
            overload_cast_explicit<bool, FrameId, SourceId>(
                &Class::BelongsToSource),
            py::arg("frame_id"), py::arg("source_id"),
            cls_doc.BelongsToSource.doc_2args_frame_id_source_id)
        .def("GetOwningSourceName",
            overload_cast_explicit<const std::string&, FrameId>(
                &Class::GetOwningSourceName),
            py_rvp::reference_internal, py::arg("frame_id"),
            cls_doc.GetOwningSourceName.doc_1args_frame_id)
        .def("GetName",
            overload_cast_explicit<const std::string&, FrameId>(
                &Class::GetName),
            py_rvp::reference_internal, py::arg("frame_id"),
            cls_doc.GetName.doc_1args_frame_id)
        .def("GetFrameGroup", &Class::GetFrameGroup, py::arg("frame_id"),
            cls_doc.GetFrameGroup.doc)
        .def("NumGeometriesForFrame", &Class::NumGeometriesForFrame,
            py::arg("frame_id"), cls_doc.NumGeometriesForFrame.doc)
        .def("NumGeometriesForFrameWithRole",
            &Class::NumGeometriesForFrameWithRole, py::arg("frame_id"),
            py::arg("role"), cls_doc.NumGeometriesForFrameWithRole.doc)
        .def("GetGeometries", &Class::GetGeometries, py::arg("frame_id"),
            py::arg("role") = std::nullopt, cls_doc.GetGeometries.doc)
        .def("GetGeometryIdByName", &Class::GetGeometryIdByName,
            py::arg("frame_id"), py::arg("role"), py::arg("name"),
            cls_doc.GetGeometryIdByName.doc)
        // Geometries and their properties.
        .def("BelongsToSource",
            overload_cast_explicit<bool, GeometryId, SourceId>(
                &Class::BelongsToSource),
            py::arg("geometry_id"), py::arg("source_id"),
            cls_doc.BelongsToSource.doc_2args_geometry_id_source_id)
        .def("GetOwningSourceName",
            overload_cast_explicit<const std::string&, GeometryId>(
                &Class::GetOwningSourceName),
            py_rvp::reference_internal, py::arg("geometry_id"),
            cls_doc.GetOwningSourceName.doc_1args_geometry_id)
        .def("GetFrameId", &Class::GetFrameId, py::arg("geometry_id"),
            cls_doc.GetFrameId.doc)
        .def("GetName",
            overload_cast_explicit<const std::string&, GeometryId>(
                &Class::GetName),
            py_rvp::reference_internal, py::arg("geometry_id"),
            cls_doc.GetName.doc_1args_geometry_id)
        .def("GetShape", &Class::GetShape, py_rvp::reference_internal,
            py::arg("geometry_id"), cls_doc.GetShape.doc)
        .def("GetPoseInFrame", &Class::GetPoseInFrame,
            py_rvp::reference_internal, py::arg("geometry_id"),
            cls_doc.GetPoseInFrame.doc)
        .def("maybe_get_hydroelastic_mesh", &Class::maybe_get_hydroelastic_mesh,
            py::arg("geometry_id"), py_rvp::reference_internal,
            cls_doc.maybe_get_hydroelastic_mesh.doc)
        .def("GetProperties", &Class::GetProperties, py_rvp::reference_internal,
            py::arg("geometry_id"), py::arg("role"), cls_doc.GetProperties.doc)
        .def("GetProximityProperties", &Class::GetProximityProperties,
            py_rvp::reference_internal, py::arg("geometry_id"),
            cls_doc.GetProximityProperties.doc)
        .def("GetIllustrationProperties", &Class::GetIllustrationProperties,
            py_rvp::reference_internal, py::arg("geometry_id"),
            cls_doc.GetIllustrationProperties.doc)
        .def("GetPerceptionProperties", &Class::GetPerceptionProperties,
            py_rvp::reference_internal, py::arg("geometry_id"),
            cls_doc.GetPerceptionProperties.doc)
        .def("CollisionFiltered", &Class::CollisionFiltered,
            py::arg("geometry_id1"), py::arg("geometry_id2"),
            cls_doc.CollisionFiltered.doc)
        .def("CloneGeometryInstance", &Class::CloneGeometryInstance,
            py::arg("geometry_id"), cls_doc.CloneGeometryInstance.doc)
        .def("geometry_version", &Class::geometry_version,
            py_rvp::reference_internal, cls_doc.geometry_version.doc);
  }

  //  SceneGraph
  {
    using Class = SceneGraph<T>;
    constexpr auto& cls_doc = doc.SceneGraph;
    auto cls = DefineTemplateClassWithDefault<Class, LeafSystem<T>>(
        m, "SceneGraph", param, cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc)
        .def("get_source_pose_port", &Class::get_source_pose_port,
            py_rvp::reference_internal, cls_doc.get_source_pose_port.doc)
        .def("get_source_configuration_port",
            &Class::get_source_configuration_port, py_rvp::reference_internal,
            cls_doc.get_source_configuration_port.doc);

    cls  // BR
        .def("get_query_output_port", &Class::get_query_output_port,
            py_rvp::reference_internal, cls_doc.get_query_output_port.doc)
        .def("model_inspector", &Class::model_inspector,
            py_rvp::reference_internal, cls_doc.model_inspector.doc)
        .def("RegisterSource",
            py::overload_cast<const std::string&>(  // BR
                &Class::RegisterSource),
            py::arg("name") = "", cls_doc.RegisterSource.doc)
        .def("RegisterFrame",
            py::overload_cast<SourceId, const GeometryFrame&>(
                &Class::RegisterFrame),
            py::arg("source_id"), py::arg("frame"),
            cls_doc.RegisterFrame.doc_2args)
        .def("RegisterFrame",
            py::overload_cast<SourceId, FrameId, const GeometryFrame&>(
                &Class::RegisterFrame),
            py::arg("source_id"), py::arg("parent_id"), py::arg("frame"),
            cls_doc.RegisterFrame.doc_3args)
        .def("RenameFrame", &Class::RenameFrame, py::arg("frame_id"),
            py::arg("name"), cls_doc.RenameFrame.doc)
        .def("RegisterGeometry",
            py::overload_cast<SourceId, FrameId,
                std::unique_ptr<GeometryInstance>>(&Class::RegisterGeometry),
            py::arg("source_id"), py::arg("frame_id"), py::arg("geometry"),
            cls_doc.RegisterGeometry.doc_3args)
        .def("RegisterGeometry",
            overload_cast_explicit<GeometryId, systems::Context<T>*, SourceId,
                FrameId, std::unique_ptr<GeometryInstance>>(
                &Class::RegisterGeometry),
            py::arg("context"), py::arg("source_id"), py::arg("frame_id"),
            py::arg("geometry"), cls_doc.RegisterGeometry.doc_4args)
        .def("RegisterAnchoredGeometry",
            py::overload_cast<SourceId, std::unique_ptr<GeometryInstance>>(
                &Class::RegisterAnchoredGeometry),
            py::arg("source_id"), py::arg("geometry"),
            cls_doc.RegisterAnchoredGeometry.doc)
        .def("RenameGeometry", &Class::RenameGeometry, py::arg("geometry_id"),
            py::arg("name"), cls_doc.RenameGeometry.doc)
        .def("ChangeShape",
            py::overload_cast<SourceId, GeometryId, const Shape&,
                std::optional<math::RigidTransform<double>>>(
                &Class::ChangeShape),
            py::arg("source_id"), py::arg("geometry_id"), py::arg("shape"),
            py::arg("X_FG") = std::nullopt, cls_doc.ChangeShape.doc_model)
        .def("ChangeShape",
            overload_cast_explicit<void, systems::Context<T>*, SourceId,
                GeometryId, const Shape&,
                std::optional<math::RigidTransform<double>>>(
                &Class::ChangeShape),
            py::arg("context"), py::arg("source_id"), py::arg("geometry_id"),
            py::arg("shape"), py::arg("X_FG") = std::nullopt,
            cls_doc.ChangeShape.doc_context)
        .def("RemoveGeometry",
            py::overload_cast<SourceId, GeometryId>(&Class::RemoveGeometry),
            py::arg("source_id"), py::arg("geometry_id"),
            cls_doc.RemoveGeometry.doc_2args)
        .def("RemoveGeometry",
            overload_cast_explicit<void, systems::Context<T>*, SourceId,
                GeometryId>(&Class::RemoveGeometry),
            py::arg("context"), py::arg("source_id"), py::arg("geometry_id"),
            cls_doc.RemoveGeometry.doc_3args)
        .def("collision_filter_manager",
            overload_cast_explicit<CollisionFilterManager, Context<T>*>(
                &Class::collision_filter_manager),
            py::arg("context"), cls_doc.collision_filter_manager.doc_1args)
        .def("collision_filter_manager",
            overload_cast_explicit<CollisionFilterManager>(
                &Class::collision_filter_manager),
            cls_doc.collision_filter_manager.doc_0args)
        .def("AddRenderer",
            overload_cast_explicit<void, std::string,
                std::unique_ptr<render::RenderEngine>>(&Class::AddRenderer),
            py::arg("name"), py::arg("renderer"), cls_doc.AddRenderer.doc_2args)
        .def("AddRenderer",
            overload_cast_explicit<void, systems::Context<T>*, std::string,
                std::unique_ptr<render::RenderEngine>>(&Class::AddRenderer),
            py::arg("context"), py::arg("name"), py::arg("renderer"),
            cls_doc.AddRenderer.doc_3args)
        .def("RemoveRenderer",
            overload_cast_explicit<void, const std::string&>(
                &Class::RemoveRenderer),
            py::arg("name"), cls_doc.RemoveRenderer.doc_1args)
        .def("RemoveRenderer",
            overload_cast_explicit<void, systems::Context<T>*,
                const std::string&>(&Class::RemoveRenderer),
            py::arg("context"), py::arg("name"),
            cls_doc.RemoveRenderer.doc_2args)
        .def("HasRenderer",
            overload_cast_explicit<bool, const std::string&>(
                &Class::HasRenderer),
            py::arg("name"), cls_doc.HasRenderer.doc_1args)
        .def("HasRenderer",
            overload_cast_explicit<bool, const systems::Context<T>&,
                const std::string&>(&Class::HasRenderer),
            py::arg("context"), py::arg("name"), cls_doc.HasRenderer.doc_2args)
        .def("RendererCount",
            overload_cast_explicit<int>(&Class::RendererCount),
            cls_doc.RendererCount.doc_0args)
        .def("RendererCount",
            overload_cast_explicit<int, const systems::Context<T>&>(
                &Class::RendererCount),
            py::arg("context"), cls_doc.RendererCount.doc_1args)
        .def("GetRendererTypeName",
            overload_cast_explicit<std::string, const std::string&>(
                &Class::GetRendererTypeName),
            py::arg("name"), cls_doc.GetRendererTypeName.doc_1args)
        .def("GetRendererTypeName",
            overload_cast_explicit<std::string, const systems::Context<T>&,
                const std::string&>(&Class::GetRendererTypeName),
            py::arg("context"), py::arg("name"),
            cls_doc.GetRendererTypeName.doc_2args)
        // - Begin: AssignRole Overloads.
        // - - Proximity.
        .def(
            "AssignRole",
            [](Class& self, SourceId source_id, GeometryId geometry_id,
                ProximityProperties properties, RoleAssign assign) {
              self.AssignRole(source_id, geometry_id, properties, assign);
            },
            py::arg("source_id"), py::arg("geometry_id"), py::arg("properties"),
            py::arg("assign") = RoleAssign::kNew,
            cls_doc.AssignRole.doc_proximity_direct)
        .def(
            "AssignRole",
            [](Class& self, Context<T>* context, SourceId source_id,
                GeometryId geometry_id, ProximityProperties properties,
                RoleAssign assign) {
              self.AssignRole(
                  context, source_id, geometry_id, properties, assign);
            },
            py::arg("context"), py::arg("source_id"), py::arg("geometry_id"),
            py::arg("properties"), py::arg("assign") = RoleAssign::kNew,
            cls_doc.AssignRole.doc_proximity_context)
        // - - Perception.
        .def(
            "AssignRole",
            [](Class& self, SourceId source_id, GeometryId geometry_id,
                PerceptionProperties properties, RoleAssign assign) {
              self.AssignRole(source_id, geometry_id, properties, assign);
            },
            py::arg("source_id"), py::arg("geometry_id"), py::arg("properties"),
            py::arg("assign") = RoleAssign::kNew,
            cls_doc.AssignRole.doc_perception_direct)
        .def(
            "AssignRole",
            [](Class& self, Context<T>* context, SourceId source_id,
                GeometryId geometry_id, PerceptionProperties properties,
                RoleAssign assign) {
              self.AssignRole(
                  context, source_id, geometry_id, properties, assign);
            },
            py::arg("context"), py::arg("source_id"), py::arg("geometry_id"),
            py::arg("properties"), py::arg("assign") = RoleAssign::kNew,
            cls_doc.AssignRole.doc_perception_context)
        // - - Illustration.
        .def(
            "AssignRole",
            [](Class& self, SourceId source_id, GeometryId geometry_id,
                IllustrationProperties properties, RoleAssign assign) {
              self.AssignRole(source_id, geometry_id, properties, assign);
            },
            py::arg("source_id"), py::arg("geometry_id"), py::arg("properties"),
            py::arg("assign") = RoleAssign::kNew,
            cls_doc.AssignRole.doc_illustration_direct)
        .def(
            "AssignRole",
            [](Class& self, Context<T>* context, SourceId source_id,
                GeometryId geometry_id, IllustrationProperties properties,
                RoleAssign assign) {
              self.AssignRole(
                  context, source_id, geometry_id, properties, assign);
            },
            py::arg("context"), py::arg("source_id"), py::arg("geometry_id"),
            py::arg("properties"), py::arg("assign") = RoleAssign::kNew,
            cls_doc.AssignRole.doc_illustration_context)
        // - End: AssignRole Overloads.
        // - Begin: RemoveRole Overloads
        // - - FrameId.
        .def(
            "RemoveRole",
            [](Class& self, SourceId source_id, FrameId frame_id, Role role) {
              return self.RemoveRole(source_id, frame_id, role);
            },
            py::arg("source_id"), py::arg("frame_id"), py::arg("role"),
            cls_doc.RemoveRole.doc_frame_direct)
        .def(
            "RemoveRole",
            [](Class& self, Context<T>* context, SourceId source_id,
                FrameId frame_id, Role role) {
              return self.RemoveRole(context, source_id, frame_id, role);
            },
            py::arg("context"), py::arg("source_id"), py::arg("frame_id"),
            py::arg("role"), cls_doc.RemoveRole.doc_frame_context)
        // - - GeometryId.
        .def(
            "RemoveRole",
            [](Class& self, SourceId source_id, GeometryId geometry_id,
                Role role) {
              return self.RemoveRole(source_id, geometry_id, role);
            },
            py::arg("source_id"), py::arg("geometry_id"), py::arg("role"),
            cls_doc.RemoveRole.doc_geometry_direct)
        .def(
            "RemoveRole",
            [](Class& self, Context<T>* context, SourceId source_id,
                GeometryId geometry_id, Role role) {
              return self.RemoveRole(context, source_id, geometry_id, role);
            },
            py::arg("context"), py::arg("source_id"), py::arg("geometry_id"),
            py::arg("role"), cls_doc.RemoveRole.doc_geometry_context)
        // - End: RemoveRole Overloads.
        .def_static("world_frame_id", &Class::world_frame_id,
            cls_doc.world_frame_id.doc);
  }

  //  FramePoseVector
  {
    using Class = FramePoseVector<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "FramePoseVector", param, doc.KinematicsVector.doc);
    cls  // BR
        .def(py::init<>(), doc.KinematicsVector.ctor.doc_0args)
        .def(
            "clear", &FramePoseVector<T>::clear, doc.KinematicsVector.clear.doc)
        .def(
            "set_value",
            [](Class* self, FrameId id, const math::RigidTransform<T>& value) {
              self->set_value(id, value);
            },
            py::arg("id"), py::arg("value"), doc.KinematicsVector.set_value.doc)
        .def("size", &FramePoseVector<T>::size, doc.KinematicsVector.size.doc)
        // This intentionally copies the value to avoid segfaults from accessing
        // the result after clear() is called. (see #11583)
        .def("value", &FramePoseVector<T>::value, py::arg("id"),
            doc.KinematicsVector.value.doc)
        .def("has_id", &FramePoseVector<T>::has_id, py::arg("id"),
            doc.KinematicsVector.has_id.doc)
        .def("ids", &FramePoseVector<T>::ids, doc.KinematicsVector.ids.doc);
    AddValueInstantiation<FramePoseVector<T>>(m);
  }

  //  QueryObject
  {
    using Class = QueryObject<T>;
    constexpr auto& cls_doc = doc.QueryObject;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "QueryObject", param, cls_doc.doc);
    cls  // BR
        .def(py::init(), cls_doc.ctor.doc)
        .def("inspector", &QueryObject<T>::inspector,
            py_rvp::reference_internal, cls_doc.inspector.doc)
        .def("GetPoseInWorld",
            overload_cast_explicit<const math::RigidTransform<T>&, FrameId>(
                &Class::GetPoseInWorld),
            py::arg("frame_id"), py_rvp::reference_internal,
            cls_doc.GetPoseInWorld.doc_1args_frame_id)
        .def("GetPoseInParent", &Class::GetPoseInParent, py::arg("frame_id"),
            py_rvp::reference_internal, cls_doc.GetPoseInParent.doc)
        .def("GetPoseInWorld",
            overload_cast_explicit<const math::RigidTransform<T>&, GeometryId>(
                &Class::GetPoseInWorld),
            py::arg("geometry_id"), py_rvp::reference_internal,
            cls_doc.GetPoseInWorld.doc_1args_geometry_id)
        .def("ComputeSignedDistancePairwiseClosestPoints",
            &QueryObject<T>::ComputeSignedDistancePairwiseClosestPoints,
            py::arg("max_distance") = std::numeric_limits<double>::infinity(),
            cls_doc.ComputeSignedDistancePairwiseClosestPoints.doc)
        .def("ComputeSignedDistancePairClosestPoints",
            &QueryObject<T>::ComputeSignedDistancePairClosestPoints,
            py::arg("geometry_id_A"), py::arg("geometry_id_B"),
            cls_doc.ComputeSignedDistancePairClosestPoints.doc)
        .def("ComputePointPairPenetration",
            &QueryObject<T>::ComputePointPairPenetration,
            cls_doc.ComputePointPairPenetration.doc)
        .def("ComputeSignedDistanceToPoint",
            &QueryObject<T>::ComputeSignedDistanceToPoint, py::arg("p_WQ"),
            py::arg("threshold") = std::numeric_limits<double>::infinity(),
            cls_doc.ComputeSignedDistanceToPoint.doc)
        .def("FindCollisionCandidates",
            &QueryObject<T>::FindCollisionCandidates,
            cls_doc.FindCollisionCandidates.doc)
        .def("HasCollisions", &QueryObject<T>::HasCollisions,
            cls_doc.HasCollisions.doc)
        .def(
            "RenderColorImage",
            [](const Class* self, const render::ColorRenderCamera& camera,
                FrameId parent_frame, const math::RigidTransformd& X_PC) {
              systems::sensors::ImageRgba8U img(
                  camera.core().intrinsics().width(),
                  camera.core().intrinsics().height());
              self->RenderColorImage(camera, parent_frame, X_PC, &img);
              return img;
            },
            py::arg("camera"), py::arg("parent_frame"), py::arg("X_PC"),
            cls_doc.RenderColorImage.doc)
        .def(
            "RenderDepthImage",
            [](const Class* self, const render::DepthRenderCamera& camera,
                FrameId parent_frame, const math::RigidTransformd& X_PC) {
              systems::sensors::ImageDepth32F img(
                  camera.core().intrinsics().width(),
                  camera.core().intrinsics().height());
              self->RenderDepthImage(camera, parent_frame, X_PC, &img);
              return img;
            },
            py::arg("camera"), py::arg("parent_frame"), py::arg("X_PC"),
            cls_doc.RenderDepthImage.doc)
        .def(
            "RenderLabelImage",
            [](const Class* self, const render::ColorRenderCamera& camera,
                FrameId parent_frame, const math::RigidTransformd& X_PC) {
              systems::sensors::ImageLabel16I img(
                  camera.core().intrinsics().width(),
                  camera.core().intrinsics().height());
              self->RenderLabelImage(camera, parent_frame, X_PC, &img);
              return img;
            },
            py::arg("camera"), py::arg("parent_frame"), py::arg("X_PC"),
            cls_doc.RenderLabelImage.doc);

    if constexpr (scalar_predicate<T>::is_bool) {
      cls  // BR
          .def("ComputeContactSurfaces",
              &Class::template ComputeContactSurfaces<T>,
              py::arg("representation"), cls_doc.ComputeContactSurfaces.doc)
          .def(
              "ComputeContactSurfacesWithFallback",
              [](const Class* self,
                  HydroelasticContactRepresentation representation) {
                // For the Python bindings, we'll use return values instead of
                // output pointers.
                std::vector<ContactSurface<T>> surfaces;
                std::vector<PenetrationAsPointPair<T>> point_pairs;
                self->template ComputeContactSurfacesWithFallback<T>(
                    representation, &surfaces, &point_pairs);
                return std::make_pair(
                    std::move(surfaces), std::move(point_pairs));
              },
              py::arg("representation"),
              cls_doc.ComputeContactSurfacesWithFallback.doc);
    }

    AddValueInstantiation<QueryObject<T>>(m);
  }

  // SignedDistancePair
  {
    using Class = SignedDistancePair<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "SignedDistancePair", param, doc.SignedDistancePair.doc);
    cls  // BR
        .def(ParamInit<Class>(), doc.SignedDistancePair.ctor.doc)
        .def_readwrite("id_A", &SignedDistancePair<T>::id_A,
            doc.SignedDistancePair.id_A.doc)
        .def_readwrite("id_B", &SignedDistancePair<T>::id_B,
            doc.SignedDistancePair.id_B.doc)
        .def_readwrite("p_ACa", &SignedDistancePair<T>::p_ACa,
            return_value_policy_for_scalar_type<T>(),
            doc.SignedDistancePair.p_ACa.doc)
        .def_readwrite("p_BCb", &SignedDistancePair<T>::p_BCb,
            return_value_policy_for_scalar_type<T>(),
            doc.SignedDistancePair.p_BCb.doc)
        .def_readwrite("distance", &SignedDistancePair<T>::distance,
            doc.SignedDistancePair.distance.doc)
        .def_readwrite("nhat_BA_W", &SignedDistancePair<T>::nhat_BA_W,
            return_value_policy_for_scalar_type<T>(),
            doc.SignedDistancePair.nhat_BA_W.doc);
  }

  // SignedDistanceToPoint
  {
    using Class = SignedDistanceToPoint<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "SignedDistanceToPoint", param, doc.SignedDistanceToPoint.doc);
    cls  // BR
        .def(ParamInit<Class>(), doc.SignedDistanceToPoint.ctor.doc)
        .def_readwrite("id_G", &SignedDistanceToPoint<T>::id_G,
            doc.SignedDistanceToPoint.id_G.doc)
        .def_readwrite("p_GN", &SignedDistanceToPoint<T>::p_GN,
            return_value_policy_for_scalar_type<T>(),
            doc.SignedDistanceToPoint.p_GN.doc)
        .def_readwrite("distance", &SignedDistanceToPoint<T>::distance,
            doc.SignedDistanceToPoint.distance.doc)
        .def_readwrite("grad_W", &SignedDistanceToPoint<T>::grad_W,
            return_value_policy_for_scalar_type<T>(),
            doc.SignedDistanceToPoint.grad_W.doc);
  }

  // PenetrationAsPointPair
  {
    using Class = PenetrationAsPointPair<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "PenetrationAsPointPair", param, doc.PenetrationAsPointPair.doc);
    cls  // BR
        .def(ParamInit<Class>())
        .def_readwrite("id_A", &PenetrationAsPointPair<T>::id_A,
            doc.PenetrationAsPointPair.id_A.doc)
        .def_readwrite("id_B", &PenetrationAsPointPair<T>::id_B,
            doc.PenetrationAsPointPair.id_B.doc)
        .def_readwrite("p_WCa", &PenetrationAsPointPair<T>::p_WCa,
            py::return_value_policy::copy, doc.PenetrationAsPointPair.p_WCa.doc)
        .def_readwrite("p_WCb", &PenetrationAsPointPair<T>::p_WCb,
            py::return_value_policy::copy, doc.PenetrationAsPointPair.p_WCb.doc)
        .def_readwrite("nhat_BA_W", &PenetrationAsPointPair<T>::nhat_BA_W,
            doc.PenetrationAsPointPair.nhat_BA_W.doc)
        .def_readwrite("depth", &PenetrationAsPointPair<T>::depth,
            doc.PenetrationAsPointPair.depth.doc);
  }

  // ContactSurface
  // Currently we do not bind the constructor because users do not need to
  // construct it directly yet. We can get it from ComputeContactSurface*().
  if constexpr (scalar_predicate<T>::is_bool) {
    using Class = ContactSurface<T>;
    constexpr auto& cls_doc = doc.ContactSurface;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "ContactSurface", param, cls_doc.doc);
    cls  // BR
         // The two overloaded constructors are not bound yet.
        .def("id_M", &Class::id_M, cls_doc.id_M.doc)
        .def("id_N", &Class::id_N, cls_doc.id_N.doc)
        .def("num_faces", &Class::num_faces, cls_doc.num_faces.doc)
        .def("num_vertices", &Class::num_vertices, cls_doc.num_vertices.doc)
        .def("area", &Class::area, py::arg("face_index"), cls_doc.area.doc)
        .def("total_area", &Class::total_area, cls_doc.total_area.doc)
        .def("face_normal", &Class::face_normal, py::arg("face_index"),
            cls_doc.face_normal.doc)
        .def("centroid",
            overload_cast_explicit<Vector3<T>, int>(&Class::centroid),
            py::arg("face_index"), cls_doc.centroid.doc)
        .def("centroid",
            overload_cast_explicit<const Vector3<T>&>(&Class::centroid),
            cls_doc.centroid.doc)
        .def("is_triangle", &Class::is_triangle, cls_doc.is_triangle.doc)
        .def("representation", &Class::representation,
            cls_doc.representation.doc)
        .def("tri_mesh_W", &Class::tri_mesh_W, py_rvp::reference_internal,
            cls_doc.tri_mesh_W.doc)
        .def("tri_e_MN", &Class::tri_e_MN, py_rvp::reference_internal,
            cls_doc.tri_e_MN.doc)
        .def("poly_mesh_W", &Class::poly_mesh_W, py_rvp::reference_internal,
            cls_doc.poly_mesh_W.doc)
        .def("poly_e_MN", &Class::poly_e_MN, py_rvp::reference_internal,
            cls_doc.poly_e_MN.doc)
        .def("HasGradE_M", &Class::HasGradE_M, cls_doc.HasGradE_M.doc)
        .def("HasGradE_N", &Class::HasGradE_N, cls_doc.HasGradE_N.doc)
        .def("EvaluateGradE_M_W", &Class::EvaluateGradE_M_W, py::arg("index"),
            cls_doc.EvaluateGradE_M_W.doc)
        .def("EvaluateGradE_N_W", &Class::EvaluateGradE_N_W, py::arg("index"),
            cls_doc.EvaluateGradE_N_W.doc)
        .def("Equal", &Class::Equal, py::arg("surface"), cls_doc.Equal.doc);
    DefCopyAndDeepCopy(&cls);
  }
}  // NOLINT(readability/fn_size)
}  // namespace

void DefineGeometrySceneGraph(py::module m) {
  py::module::import("pydrake.systems.framework");
  DoScalarIndependentDefinitions(m);
  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      CommonScalarPack{});
}
}  // namespace pydrake
}  // namespace drake
