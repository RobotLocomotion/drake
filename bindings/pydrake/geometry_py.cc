#include <sstream>

#include "pybind11/eigen.h"
#include "pybind11/eval.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_properties.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/render/gl_renderer/render_engine_gl_factory.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render/render_engine_vtk_factory.h"
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace pydrake {
namespace {
using systems::Context;
using systems::LeafSystem;

template <typename Class>
void BindIdentifier(py::module m, const std::string& name, const char* id_doc) {
  constexpr auto& cls_doc = pydrake_doc.drake.Identifier;

  py::class_<Class>(m, name.c_str(), id_doc)
      .def("get_value", &Class::get_value, cls_doc.get_value.doc)
      .def("is_valid", &Class::is_valid, cls_doc.is_valid.doc)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def(py::self < py::self)
      // TODO(eric.cousineau): Use `py::hash()` instead of `py::detail::hash()`
      // pending merge of: https://github.com/pybind/pybind11/pull/2217
      .def(py::detail::hash(py::self))
      .def_static("get_new_id", &Class::get_new_id, cls_doc.get_new_id.doc)
      .def("__repr__", [name](const Class& self) {
        return py::str("<{} value={}>").format(name, self.get_value());
      });
}

void def_geometry_render(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry::render;
  m.doc() = "Local bindings for `drake::geometry::render`";
  constexpr auto& doc = pydrake_doc.drake.geometry.render;

  {
    using Class = CameraProperties;
    py::class_<Class>(m, "CameraProperties", doc.CameraProperties.doc)
        .def(py::init<int, int, double, std::string>(), py::arg("width"),
            py::arg("height"), py::arg("fov_y"), py::arg("renderer_name"),
            doc.CameraProperties.ctor.doc)
        .def_readwrite("width", &Class::width, doc.CameraProperties.width.doc)
        .def_readwrite(
            "height", &Class::height, doc.CameraProperties.height.doc)
        .def_readwrite("fov_y", &Class::fov_y, doc.CameraProperties.fov_y.doc)
        .def_readwrite("renderer_name", &Class::renderer_name,
            doc.CameraProperties.renderer_name.doc);
  }

  {
    using Class = DepthCameraProperties;
    py::class_<Class, CameraProperties>(
        m, "DepthCameraProperties", doc.DepthCameraProperties.doc)
        .def(py::init<int, int, double, std::string, double, double>(),
            py::arg("width"), py::arg("height"), py::arg("fov_y"),
            py::arg("renderer_name"), py::arg("z_near"), py::arg("z_far"),
            doc.DepthCameraProperties.ctor.doc)
        .def_readwrite(
            "z_near", &Class::z_near, doc.DepthCameraProperties.z_near.doc)
        .def_readwrite(
            "z_far", &Class::z_far, doc.DepthCameraProperties.z_far.doc);
  }

  {
    // TODO(SeanCurtis-TRI): Expose the full public API after the RenderIndex
    //  and GeometryIndex classes go away (everything will become GeometryId
    //  centric).
    using Class = RenderEngine;
    py::class_<Class>(m, "RenderEngine");
  }

  py::class_<RenderEngineVtkParams>(
      m, "RenderEngineVtkParams", doc.RenderEngineVtkParams.doc)
      .def(ParamInit<RenderEngineVtkParams>())
      .def_readwrite("default_label", &RenderEngineVtkParams::default_label,
          doc.RenderEngineVtkParams.default_label.doc)
      .def_readwrite("default_diffuse", &RenderEngineVtkParams::default_diffuse,
          doc.RenderEngineVtkParams.default_diffuse.doc);

  m.def("MakeRenderEngineGl", &MakeRenderEngineGl, doc.MakeRenderEngineGl.doc);

  m.def("MakeRenderEngineVtk", &MakeRenderEngineVtk, py::arg("params"),
      doc.MakeRenderEngineVtk.doc);

  {
    py::class_<RenderLabel> render_label(m, "RenderLabel", doc.RenderLabel.doc);
    render_label
        .def(py::init<int>(), py::arg("value"), doc.RenderLabel.ctor.doc_1args)
        .def("is_reserved", &RenderLabel::is_reserved)
        .def("__int__", [](const RenderLabel& self) -> int { return self; })
        // EQ(==).
        .def(py::self == py::self)
        .def(py::self == int{})
        .def(int{} == py::self)
        // NE(!=).
        .def(py::self != py::self)
        .def(py::self != int{})
        .def(int{} != py::self);
    render_label.attr("kEmpty") = RenderLabel::kEmpty;
    render_label.attr("kDoNotRender") = RenderLabel::kDoNotRender;
    render_label.attr("kDontCare") = RenderLabel::kDontCare;
    render_label.attr("kUnspecified") = RenderLabel::kUnspecified;
    render_label.attr("kMaxUnreserved") = RenderLabel::kMaxUnreserved;
  }

  AddValueInstantiation<RenderLabel>(m);
}

template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = GetPyParam<T>();
  constexpr auto& doc = pydrake_doc.drake.geometry;

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;
  py::module::import("pydrake.systems.framework");

  //  SceneGraphInspector
  {
    using Class = SceneGraphInspector<T>;
    constexpr auto& cls_doc = doc.SceneGraphInspector;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "SceneGraphInspector", param, cls_doc.doc);
    cls  // BR
        .def("num_sources", &Class::num_sources, cls_doc.num_sources.doc)
        .def("num_frames", &Class::num_frames, cls_doc.num_frames.doc)
        .def("num_geometries", &Class::num_geometries,
            cls_doc.num_geometries.doc)
        .def("GetAllGeometryIds", &Class::GetAllGeometryIds,
            cls_doc.GetAllGeometryIds.doc)
        .def("GetFrameId", &Class::GetFrameId, py::arg("geometry_id"),
            cls_doc.GetFrameId.doc)
        .def("GetGeometryIdByName", &Class::GetGeometryIdByName,
            py::arg("frame_id"), py::arg("role"), py::arg("name"),
            cls_doc.GetGeometryIdByName.doc)
        .def("GetNameByFrameId",
            overload_cast_explicit<const std::string&, FrameId>(
                &Class::GetName),
            py_reference_internal, py::arg("frame_id"),
            cls_doc.GetName.doc_1args_frame_id)
        .def("GetNameByGeometryId",
            overload_cast_explicit<const std::string&, GeometryId>(
                &Class::GetName),
            py_reference_internal, py::arg("geometry_id"),
            cls_doc.GetName.doc_1args_geometry_id)
        .def("GetShape", &Class::GetShape, py_reference_internal,
            py::arg("geometry_id"), cls_doc.GetShape.doc)
        .def("GetPoseInFrame", &Class::GetPoseInFrame, py_reference_internal,
            py::arg("geometry_id"), cls_doc.GetPoseInFrame.doc)
        .def("GetProximityProperties", &Class::GetProximityProperties,
            py_reference_internal, py::arg("geometry_id"),
            cls_doc.GetProximityProperties.doc)
        .def("GetIllustrationProperties", &Class::GetIllustrationProperties,
            py_reference_internal, py::arg("geometry_id"),
            cls_doc.GetIllustrationProperties.doc)
        .def("GetPerceptionProperties", &Class::GetPerceptionProperties,
            py_reference_internal, py::arg("geometry_id"),
            cls_doc.GetPerceptionProperties.doc)
        .def("CloneGeometryInstance", &Class::CloneGeometryInstance,
            py::arg("geometry_id"), cls_doc.CloneGeometryInstance.doc);
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
            py_reference_internal, cls_doc.get_source_pose_port.doc)
        .def(
            "get_pose_bundle_output_port",
            [](Class* self) -> const systems::OutputPort<T>& {
              return self->get_pose_bundle_output_port();
            },
            py_reference_internal, cls_doc.get_pose_bundle_output_port.doc)
        .def("get_query_output_port", &Class::get_query_output_port,
            py_reference_internal, cls_doc.get_query_output_port.doc)
        .def("model_inspector", &Class::model_inspector, py_reference_internal,
            cls_doc.model_inspector.doc)
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
        .def("RegisterGeometry",
            py::overload_cast<SourceId, FrameId,
                std::unique_ptr<GeometryInstance>>(&Class::RegisterGeometry),
            py::arg("source_id"), py::arg("frame_id"), py::arg("geometry"),
            cls_doc.RegisterGeometry.doc_3args_source_id_frame_id_geometry)
        .def("RegisterGeometry",
            py::overload_cast<SourceId, GeometryId,
                std::unique_ptr<GeometryInstance>>(&Class::RegisterGeometry),
            py::arg("source_id"), py::arg("geometry_id"), py::arg("geometry"),
            cls_doc.RegisterGeometry.doc_3args_source_id_geometry_id_geometry)
        .def("RegisterAnchoredGeometry",
            py::overload_cast<SourceId, std::unique_ptr<GeometryInstance>>(
                &Class::RegisterAnchoredGeometry),
            py::arg("source_id"), py::arg("geometry"),
            cls_doc.RegisterAnchoredGeometry.doc)
        .def("ExcludeCollisionsBetween",
            py::overload_cast<const GeometrySet&, const GeometrySet&>(
                &Class::ExcludeCollisionsBetween),
            py_reference_internal, py::arg("setA"), py::arg("setB"),
            cls_doc.ExcludeCollisionsBetween.doc_2args)
        .def("ExcludeCollisionsBetween",
            overload_cast_explicit<void, Context<T>*, const GeometrySet&,
                const GeometrySet&>(&Class::ExcludeCollisionsBetween),
            py_reference_internal, py::arg("context"), py::arg("setA"),
            py::arg("setB"), cls_doc.ExcludeCollisionsBetween.doc_3args)
        .def("ExcludeCollisionsWithin",
            py::overload_cast<const GeometrySet&>(
                &Class::ExcludeCollisionsWithin),
            py_reference_internal, py::arg("set"),
            cls_doc.ExcludeCollisionsWithin.doc_1args)
        .def("ExcludeCollisionsWithin",
            overload_cast_explicit<void, Context<T>*, const GeometrySet&>(
                &Class::ExcludeCollisionsWithin),
            py_reference_internal, py::arg("context"), py::arg("set"),
            cls_doc.ExcludeCollisionsWithin.doc_2args)
        .def("AddRenderer", &Class::AddRenderer, py::arg("name"),
            py::arg("renderer"), cls_doc.AddRenderer.doc)
        .def("HasRenderer", &Class::HasRenderer, py::arg("name"),
            cls_doc.HasRenderer.doc)
        .def("RendererCount", &Class::RendererCount, cls_doc.RendererCount.doc)
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
        .def_static("world_frame_id", &Class::world_frame_id,
            cls_doc.world_frame_id.doc);
  }

  //  FramePoseVector
  {
    using Class = FramePoseVector<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "FramePoseVector", param, doc.FrameKinematicsVector.doc);
    cls  // BR
        .def(py::init<>(), doc.FrameKinematicsVector.ctor.doc_0args)
        .def("clear", &FramePoseVector<T>::clear,
            doc.FrameKinematicsVector.clear.doc)
        .def(
            "set_value",
            [](Class* self, FrameId id, const math::RigidTransform<T>& value) {
              self->set_value(id, value);
            },
            py::arg("id"), py::arg("value"),
            doc.FrameKinematicsVector.set_value.doc)
        .def("size", &FramePoseVector<T>::size,
            doc.FrameKinematicsVector.size.doc)
        // This intentionally copies the value to avoid segfaults from accessing
        // the result after clear() is called. (see #11583)
        .def("value", &FramePoseVector<T>::value, py::arg("id"),
            doc.FrameKinematicsVector.value.doc)
        .def("has_id", &FramePoseVector<T>::has_id, py::arg("id"),
            doc.FrameKinematicsVector.has_id.doc)
        .def("frame_ids", &FramePoseVector<T>::frame_ids,
            doc.FrameKinematicsVector.frame_ids.doc);
    AddValueInstantiation<FramePoseVector<T>>(m);
  }

  //  QueryObject
  {
    using Class = QueryObject<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "QueryObject", param, doc.QueryObject.doc);
    cls  // BR
        .def(py::init(), doc.QueryObject.ctor.doc)
        .def("inspector", &QueryObject<T>::inspector, py_reference_internal,
            doc.QueryObject.inspector.doc)
        .def("ComputeSignedDistancePairwiseClosestPoints",
            &QueryObject<T>::ComputeSignedDistancePairwiseClosestPoints,
            py::arg("max_distance") = std::numeric_limits<double>::infinity(),
            doc.QueryObject.ComputeSignedDistancePairwiseClosestPoints.doc)
        .def("ComputeSignedDistancePairClosestPoints",
            &QueryObject<T>::ComputeSignedDistancePairClosestPoints,
            py::arg("id_A"), py::arg("id_B"),
            doc.QueryObject.ComputeSignedDistancePairClosestPoints.doc)
        .def("ComputePointPairPenetration",
            &QueryObject<T>::ComputePointPairPenetration,
            doc.QueryObject.ComputePointPairPenetration.doc)
        .def("ComputeSignedDistanceToPoint",
            &QueryObject<T>::ComputeSignedDistanceToPoint, py::arg("p_WQ"),
            py::arg("threshold") = std::numeric_limits<double>::infinity(),
            doc.QueryObject.ComputeSignedDistanceToPoint.doc)
        .def("FindCollisionCandidates",
            &QueryObject<T>::FindCollisionCandidates,
            doc.QueryObject.FindCollisionCandidates.doc)
        .def("HasCollisions", &QueryObject<T>::HasCollisions,
            doc.QueryObject.HasCollisions.doc)
        .def(
            "RenderColorImage",
            [](const Class* self, const render::CameraProperties& camera,
                FrameId parent_frame, const math::RigidTransformd& X_PC,
                bool show_window) {
              systems::sensors::ImageRgba8U img(camera.width, camera.height);
              self->RenderColorImage(
                  camera, parent_frame, X_PC, show_window, &img);
              return img;
            },
            py::arg("camera"), py::arg("parent_frame"), py::arg("X_PC"),
            py::arg("show_window") = false,
            doc.QueryObject.RenderColorImage.doc)
        .def(
            "RenderDepthImage",
            [](const Class* self, const render::DepthCameraProperties& camera,
                FrameId parent_frame, const math::RigidTransformd& X_PC) {
              systems::sensors::ImageDepth32F img(camera.width, camera.height);
              self->RenderDepthImage(camera, parent_frame, X_PC, &img);
              return img;
            },
            py::arg("camera"), py::arg("parent_frame"), py::arg("X_PC"),
            doc.QueryObject.RenderDepthImage.doc)
        .def(
            "RenderLabelImage",
            [](const Class* self, const render::CameraProperties& camera,
                FrameId parent_frame, const math::RigidTransformd& X_PC,
                bool show_window = false) {
              systems::sensors::ImageLabel16I img(camera.width, camera.height);
              self->RenderLabelImage(
                  camera, parent_frame, X_PC, show_window, &img);
              return img;
            },
            py::arg("camera"), py::arg("parent_frame"), py::arg("X_PC"),
            py::arg("show_window") = false,
            doc.QueryObject.RenderLabelImage.doc);

    AddValueInstantiation<QueryObject<T>>(m);
  }

  // SignedDistancePair
  {
    using Class = SignedDistancePair<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "SignedDistancePair", param, doc.SignedDistancePair.doc);
    cls  // BR
        .def(ParamInit<Class>(), doc.SignedDistancePair.ctor.doc_7args)
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
            doc.SignedDistancePair.nhat_BA_W.doc)
        .def_readwrite("is_nhat_BA_W_unique",
            &SignedDistancePair<T>::is_nhat_BA_W_unique,
            doc.SignedDistancePair.is_nhat_BA_W_unique.doc);
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
        .def(ParamInit<Class>(), doc.PenetrationAsPointPair.ctor.doc)
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

  // SurfaceVertex
  {
    using Class = SurfaceVertex<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "SurfaceVertex", param, doc.SurfaceVertex.doc);
    cls  // BR
        .def(py::init<const Vector3<T>&>(), py::arg("r_MV"),
            doc.SurfaceVertex.ctor.doc)
        .def("r_MV", &Class::r_MV, doc.SurfaceVertex.r_MV.doc);
  }

  // SurfaceMesh
  {
    using Class = SurfaceMesh<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "SurfaceMesh", param, doc.SurfaceMesh.doc);
    cls  // BR
        .def(
            py::init<std::vector<SurfaceFace>, std::vector<SurfaceVertex<T>>>(),
            py::arg("faces"), py::arg("vertices"), doc.SurfaceMesh.ctor.doc)
        .def("faces", &Class::faces, doc.SurfaceMesh.faces.doc)
        .def("vertices", &Class::vertices, doc.SurfaceMesh.vertices.doc);
  }
}

void DoScalarIndependentDefinitions(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;
  constexpr auto& doc = pydrake_doc.drake.geometry;

  {
    using Class = Rgba;
    constexpr auto& cls_doc = doc.Rgba;
    py::class_<Class>(m, "Rgba", cls_doc.doc)
        .def(py::init<double, double, double, double>(), py::arg("r"),
            py::arg("g"), py::arg("b"), py::arg("a") = 1., cls_doc.ctor.doc)
        .def("r", &Class::r, cls_doc.r.doc)
        .def("g", &Class::g, cls_doc.g.doc)
        .def("b", &Class::b, cls_doc.b.doc)
        .def("a", &Class::a, cls_doc.a.doc)
        .def("set", &Class::set, py::arg("r"), py::arg("g"), py::arg("b"),
            py::arg("a") = 1., cls_doc.set.doc)
        .def(py::self == py::self)
        .def(py::self != py::self)
        .def("__repr__", [](const Class& self) {
          return py::str("Rgba(r={}, g={}, b={}, a={})")
              .format(self.r(), self.g(), self.b(), self.a());
        });
    AddValueInstantiation<Rgba>(m);
  }

  BindIdentifier<SourceId>(m, "SourceId", doc.SourceId.doc);
  BindIdentifier<FrameId>(m, "FrameId", doc.FrameId.doc);
  BindIdentifier<GeometryId>(m, "GeometryId", doc.GeometryId.doc);

  {
    constexpr auto& cls_doc = doc.Role;
    py::enum_<Role>(m, "Role", py::arithmetic(), cls_doc.doc)
        .value("kUnassigned", Role::kUnassigned, cls_doc.kUnassigned.doc)
        .value("kProximity", Role::kProximity, cls_doc.kProximity.doc)
        .value("kIllustration", Role::kIllustration, cls_doc.kIllustration.doc)
        .value("kPerception", Role::kPerception, cls_doc.kPerception.doc);
  }

  {
    constexpr auto& cls_doc = doc.RoleAssign;
    using Class = RoleAssign;
    py::enum_<Class>(m, "RoleAssign", cls_doc.doc)
        .value("kNew", Class::kNew, cls_doc.kNew.doc)
        .value("kReplace", Class::kReplace, cls_doc.kReplace.doc);
  }

  m.def("ConnectDrakeVisualizer",
      py::overload_cast<systems::DiagramBuilder<double>*,
          const SceneGraph<double>&, lcm::DrakeLcmInterface*, geometry::Role>(
          &ConnectDrakeVisualizer),
      py::arg("builder"), py::arg("scene_graph"), py::arg("lcm") = nullptr,
      py::arg("role") = geometry::Role::kIllustration,
      // Keep alive, ownership: `return` keeps `builder` alive.
      py::keep_alive<0, 1>(),
      // See #11531 for why `py_reference` is needed.
      py_reference, doc.ConnectDrakeVisualizer.doc_4args);
  m.def("ConnectDrakeVisualizer",
      py::overload_cast<systems::DiagramBuilder<double>*,
          const SceneGraph<double>&, const systems::OutputPort<double>&,
          lcm::DrakeLcmInterface*, geometry::Role>(&ConnectDrakeVisualizer),
      py::arg("builder"), py::arg("scene_graph"),
      py::arg("pose_bundle_output_port"), py::arg("lcm") = nullptr,
      py::arg("role") = geometry::Role::kIllustration,
      // Keep alive, ownership: `return` keeps `builder` alive.
      py::keep_alive<0, 1>(),
      // See #11531 for why `py_reference` is needed.
      py_reference, doc.ConnectDrakeVisualizer.doc_5args);
  m.def("DispatchLoadMessage", &DispatchLoadMessage, py::arg("scene_graph"),
      py::arg("lcm"), py::arg("role") = geometry::Role::kIllustration,
      doc.DispatchLoadMessage.doc);

  // Shape constructors
  {
    py::class_<Shape> shape_cls(m, "Shape", doc.Shape.doc);
    DefClone(&shape_cls);
    py::class_<Sphere, Shape>(m, "Sphere", doc.Sphere.doc)
        .def(py::init<double>(), py::arg("radius"), doc.Sphere.ctor.doc)
        .def("radius", &Sphere::radius, doc.Sphere.radius.doc);
    py::class_<Cylinder, Shape>(m, "Cylinder", doc.Cylinder.doc)
        .def(py::init<double, double>(), py::arg("radius"), py::arg("length"),
            doc.Cylinder.ctor.doc)
        .def("radius", &Cylinder::radius, doc.Cylinder.radius.doc)
        .def("length", &Cylinder::length, doc.Cylinder.length.doc);
    py::class_<Box, Shape>(m, "Box", doc.Box.doc)
        .def(py::init<double, double, double>(), py::arg("width"),
            py::arg("depth"), py::arg("height"), doc.Box.ctor.doc)
        .def("width", &Box::width, doc.Box.width.doc)
        .def("depth", &Box::depth, doc.Box.depth.doc)
        .def("height", &Box::height, doc.Box.height.doc)
        .def("size", &Box::size, py_reference_internal, doc.Box.size.doc);
    py::class_<HalfSpace, Shape>(m, "HalfSpace", doc.HalfSpace.doc)
        .def(py::init<>(), doc.HalfSpace.ctor.doc)
        .def_static("MakePose", &HalfSpace::MakePose, py::arg("Hz_dir_F"),
            py::arg("p_FB"), doc.HalfSpace.MakePose.doc);
    py::class_<Mesh, Shape>(m, "Mesh", doc.Mesh.doc)
        .def(py::init<std::string, double>(), py::arg("absolute_filename"),
            py::arg("scale") = 1.0, doc.Mesh.ctor.doc)
        .def("filename", &Mesh::filename, doc.Mesh.filename.doc)
        .def("scale", &Mesh::scale, doc.Mesh.scale.doc);
    py::class_<Convex, Shape>(m, "Convex", doc.Convex.doc)
        .def(py::init<std::string, double>(), py::arg("absolute_filename"),
            py::arg("scale") = 1.0, doc.Convex.ctor.doc)
        .def("filename", &Convex::filename, doc.Convex.filename.doc)
        .def("scale", &Convex::scale, doc.Convex.scale.doc);
  }

  // GeometryFrame
  {
    using Class = GeometryFrame;
    constexpr auto& cls_doc = doc.GeometryFrame;
    py::class_<Class>(m, "GeometryFrame", cls_doc.doc)
        .def(py::init<const std::string&, int>(), py::arg("frame_name"),
            py::arg("frame_group_id") = 0, cls_doc.ctor.doc)
        .def("id", &Class::id, cls_doc.id.doc)
        .def("name", &Class::name, cls_doc.name.doc)
        .def("frame_group", &Class::frame_group, cls_doc.frame_group.doc);
  }

  // GeometryInstance
  {
    using Class = GeometryInstance;
    constexpr auto& cls_doc = doc.GeometryInstance;
    py::class_<Class>(m, "GeometryInstance", cls_doc.doc)
        .def(py::init<const math::RigidTransform<double>&,
                 std::unique_ptr<Shape>, const std::string&>(),
            py::arg("X_PG"), py::arg("shape"), py::arg("name"),
            cls_doc.ctor.doc)
        .def("id", &Class::id, cls_doc.id.doc)
        .def("pose", &Class::pose, py_reference_internal, cls_doc.pose.doc)
        .def(
            "set_pose", &Class::set_pose, py::arg("X_PG"), cls_doc.set_pose.doc)
        .def("shape", &Class::shape, py_reference_internal, cls_doc.shape.doc)
        .def("release_shape", &Class::release_shape, cls_doc.release_shape.doc)
        .def("name", &Class::name, cls_doc.name.doc)
        .def("set_name", &Class::set_name, cls_doc.set_name.doc)
        .def("set_proximity_properties", &Class::set_proximity_properties,
            py::arg("properties"), cls_doc.set_proximity_properties.doc)
        .def("set_illustration_properties", &Class::set_illustration_properties,
            py::arg("properties"), cls_doc.set_illustration_properties.doc)
        .def("set_perception_properties", &Class::set_perception_properties,
            py::arg("properties"), cls_doc.set_perception_properties.doc)
        .def("mutable_proximity_properties",
            &Class::mutable_proximity_properties, py_reference_internal,
            cls_doc.mutable_proximity_properties.doc)
        .def("proximity_properties", &Class::proximity_properties,
            py_reference_internal, cls_doc.proximity_properties.doc)
        .def("mutable_illustration_properties",
            &Class::mutable_illustration_properties, py_reference_internal,
            cls_doc.mutable_illustration_properties.doc)
        .def("illustration_properties", &Class::illustration_properties,
            py_reference_internal, cls_doc.illustration_properties.doc)
        .def("mutable_perception_properties",
            &Class::mutable_perception_properties, py_reference_internal,
            cls_doc.mutable_perception_properties.doc)
        .def("perception_properties", &Class::perception_properties,
            py_reference_internal, cls_doc.perception_properties.doc);
  }

  {
    using Class = GeometryProperties;
    constexpr auto& cls_doc = doc.GeometryProperties;
    py::handle abstract_value_cls =
        py::module::import("pydrake.common.value").attr("AbstractValue");
    py::class_<Class>(m, "GeometryProperties", cls_doc.doc)
        .def("HasGroup", &Class::HasGroup, py::arg("group_name"),
            cls_doc.HasGroup.doc)
        .def("num_groups", &Class::num_groups, cls_doc.num_groups.doc)
        .def(
            "GetPropertiesInGroup",
            [](const Class& self, const std::string& group_name) {
              py::dict out;
              py::object py_self = py::cast(&self, py_reference);
              for (auto& [name, abstract] :
                  self.GetPropertiesInGroup(group_name)) {
                out[name.c_str()] =
                    py::cast(abstract.get(), py_reference_internal, py_self);
              }
              return out;
            },
            py::arg("group_name"), cls_doc.GetPropertiesInGroup.doc)
        .def("GetGroupNames", &Class::GetGroupNames, cls_doc.GetGroupNames.doc)
        .def(
            "AddProperty",
            [abstract_value_cls](Class& self, const std::string& group_name,
                const std::string& name, py::object value) {
              py::object abstract = abstract_value_cls.attr("Make")(value);
              self.AddPropertyAbstract(
                  group_name, name, abstract.cast<const AbstractValue&>());
            },
            py::arg("group_name"), py::arg("name"), py::arg("value"),
            cls_doc.AddProperty.doc)
        .def("HasProperty", &Class::HasProperty, py::arg("group_name"),
            py::arg("name"), cls_doc.HasProperty.doc)
        .def(
            "GetProperty",
            [](const Class& self, const std::string& group_name,
                const std::string& name) {
              py::object abstract = py::cast(
                  self.GetPropertyAbstract(group_name, name), py_reference);
              return abstract.attr("get_value")();
            },
            py::arg("group_name"), py::arg("name"), cls_doc.GetProperty.doc)
        .def(
            "GetPropertyOrDefault",
            [](const Class& self, const std::string& group_name,
                const std::string& name, py::object default_value) {
              // For now, ignore typing. This is less efficient, but eh, it's
              // Python.
              if (self.HasProperty(group_name, name)) {
                py::object py_self = py::cast(&self, py_reference);
                return py_self.attr("GetProperty")(group_name, name);
              } else {
                return default_value;
              }
            },
            py::arg("group_name"), py::arg("name"), py::arg("default_value"),
            cls_doc.GetPropertyOrDefault.doc)
        .def_static("default_group_name", &Class::default_group_name,
            cls_doc.default_group_name.doc)
        .def(
            "__str__",
            [](const Class& self) {
              std::stringstream ss;
              ss << self;
              return ss.str();
            },
            "Returns formatted string.");
  }

  // GeometrySet
  {
    using Class = GeometrySet;
    constexpr auto& cls_doc = doc.GeometrySet;
    py::class_<Class>(m, "GeometrySet", cls_doc.doc)
        .def(py::init(), doc.GeometrySet.ctor.doc);
  }

  py::class_<ProximityProperties, GeometryProperties>(
      m, "ProximityProperties", doc.ProximityProperties.doc)
      .def(py::init(), doc.ProximityProperties.ctor.doc);
  py::class_<IllustrationProperties, GeometryProperties>(
      m, "IllustrationProperties", doc.IllustrationProperties.doc)
      .def(py::init(), doc.IllustrationProperties.ctor.doc);
  py::class_<PerceptionProperties, GeometryProperties>(
      m, "PerceptionProperties", doc.PerceptionProperties.doc)
      .def(py::init(), doc.PerceptionProperties.ctor.doc);
  m.def("MakePhongIllustrationProperties", &MakePhongIllustrationProperties,
      py_reference_internal, py::arg("diffuse"),
      doc.MakePhongIllustrationProperties.doc);

  m.def("ReadObjToSurfaceMesh",
      py::overload_cast<const std::string&, double>(
          &geometry::ReadObjToSurfaceMesh),
      py::arg("filename"), py::arg("scale") = 1.0,
      doc.ReadObjToSurfaceMesh.doc_2args_filename_scale);
}

void def_geometry(py::module m) {
  DoScalarIndependentDefinitions(m);
  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      NonSymbolicScalarPack{});
}

void def_geometry_testing(py::module m) {
  class FakeTag;
  using FakeId = Identifier<FakeTag>;

  BindIdentifier<FakeId>(m, "FakeId", "Fake documentation.");
  // Get a valid, constant FakeId to test hashing with new instances returned.
  FakeId fake_id_constant{FakeId::get_new_id()};
  m.def("get_fake_id_constant",
      [fake_id_constant]() { return fake_id_constant; });
}

void def_geometry_all(py::module m) {
  py::dict vars = m.attr("__dict__");
  py::exec(
      "from pydrake.geometry import *\n"
      "from pydrake.geometry.render import *\n",
      py::globals(), vars);
}

PYBIND11_MODULE(geometry, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.lcm");

  def_geometry(m);
  def_geometry_render(m.def_submodule("render"));
  def_geometry_testing(m.def_submodule("_testing"));
  def_geometry_all(m.def_submodule("all"));
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
