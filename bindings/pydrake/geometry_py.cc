#include "pybind11/eigen.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render/render_engine_vtk_factory.h"
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"

namespace drake {
namespace pydrake {
namespace {
using systems::LeafSystem;

template <typename Class>
void BindIdentifier(py::module m, const std::string& name, const char* id_doc) {
  auto& cls_doc = pydrake_doc.drake.geometry.Identifier;

  py::class_<Class> cls(m, name.c_str(), id_doc);
  py::handle cls_handle = cls;
  cls  // BR
      .def(py::init([cls_handle]() {
        WarnDeprecated(
            py::str("The constructor for {} in Python is deprecated. "
                    "Use `get_new_id()` if necessary.")
                .format(cls_handle));
        return Class{};
      }),
          cls_doc.ctor.doc)
      .def("get_value", &Class::get_value, cls_doc.get_value.doc)
      .def("is_valid", &Class::is_valid, cls_doc.is_valid.doc)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def(py::self < py::self)
      .def_static("get_new_id", &Class::get_new_id, cls_doc.get_new_id.doc);
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

  m.def("MakeRenderEngineVtk", &MakeRenderEngineVtk, py::arg("params"),
      doc.MakeRenderEngineVtk.doc);

  {
    py::class_<RenderLabel> render_label(m, "RenderLabel", doc.RenderLabel.doc);
    render_label
        .def(py::init<int>(), py::arg("value"), doc.RenderLabel.ctor.doc_1args)
        .def("is_reserved", &RenderLabel::is_reserved)
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
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "SceneGraphInspector", param, doc.SceneGraphInspector.doc);
    cls  // BR
        .def("num_sources", &SceneGraphInspector<T>::num_sources,
            doc.SceneGraphInspector.num_sources.doc)
        .def("num_frames", &SceneGraphInspector<T>::num_frames,
            doc.SceneGraphInspector.num_frames.doc)
        .def("num_geometries", &SceneGraphInspector<T>::num_geometries,
            doc.SceneGraphInspector.num_geometries.doc)
        .def("GetAllGeometryIds", &SceneGraphInspector<T>::GetAllGeometryIds,
            doc.SceneGraphInspector.GetAllGeometryIds.doc)
        .def("GetFrameId", &SceneGraphInspector<T>::GetFrameId,
            py::arg("geometry_id"), doc.SceneGraphInspector.GetFrameId.doc)
        .def("GetGeometryIdByName",
            &SceneGraphInspector<T>::GetGeometryIdByName, py::arg("frame_id"),
            py::arg("role"), py::arg("name"),
            doc.SceneGraphInspector.GetGeometryIdByName.doc)
        .def("GetNameByFrameId",
            overload_cast_explicit<const std::string&, FrameId>(
                &SceneGraphInspector<T>::GetName),
            py_reference_internal, py::arg("frame_id"),
            doc.SceneGraphInspector.GetName.doc_1args_frame_id)
        .def("GetNameByGeometryId",
            overload_cast_explicit<const std::string&, GeometryId>(
                &SceneGraphInspector<T>::GetName),
            py_reference_internal, py::arg("geometry_id"),
            doc.SceneGraphInspector.GetName.doc_1args_geometry_id)
        .def("GetPoseInFrame", &SceneGraphInspector<T>::GetPoseInFrame,
            py_reference_internal, py::arg("geometry_id"),
            doc.SceneGraphInspector.GetPoseInFrame.doc)
        .def("GetShape", &SceneGraphInspector<T>::GetShape,
            py_reference_internal, py::arg("geometry_id"),
            doc.SceneGraphInspector.GetShape.doc);
  }

  //  SceneGraph
  {
    auto cls = DefineTemplateClassWithDefault<SceneGraph<T>, LeafSystem<T>>(
        m, "SceneGraph", param, doc.SceneGraph.doc);
    cls  // BR
        .def(py::init<>(), doc.SceneGraph.ctor.doc)
        .def("get_source_pose_port", &SceneGraph<T>::get_source_pose_port,
            py_reference_internal, doc.SceneGraph.get_source_pose_port.doc)
        .def("get_pose_bundle_output_port",
            [](SceneGraph<T>* self) -> const systems::OutputPort<T>& {
              return self->get_pose_bundle_output_port();
            },
            py_reference_internal,
            doc.SceneGraph.get_pose_bundle_output_port.doc)
        .def("get_query_output_port", &SceneGraph<T>::get_query_output_port,
            py_reference_internal, doc.SceneGraph.get_query_output_port.doc)
        .def("model_inspector", &SceneGraph<T>::model_inspector,
            py_reference_internal, doc.SceneGraph.model_inspector.doc)
        .def("RegisterSource",
            py::overload_cast<const std::string&>(  // BR
                &SceneGraph<T>::RegisterSource),
            py::arg("name") = "", doc.SceneGraph.RegisterSource.doc)
        .def("RegisterFrame",
            py::overload_cast<SourceId, const GeometryFrame&>(
                &SceneGraph<T>::RegisterFrame),
            py::arg("source_id"), py::arg("frame"),
            doc.SceneGraph.RegisterFrame.doc_2args)
        .def("RegisterFrame",
            py::overload_cast<SourceId, FrameId, const GeometryFrame&>(
                &SceneGraph<T>::RegisterFrame),
            py::arg("source_id"), py::arg("parent_id"), py::arg("frame"),
            doc.SceneGraph.RegisterFrame.doc_3args)
        .def("RegisterGeometry",
            py::overload_cast<SourceId, FrameId,
                std::unique_ptr<GeometryInstance>>(
                &SceneGraph<T>::RegisterGeometry),
            py::arg("source_id"), py::arg("frame_id"), py::arg("geometry"),
            doc.SceneGraph.RegisterGeometry
                .doc_3args_source_id_frame_id_geometry)
        .def("RegisterGeometry",
            py::overload_cast<SourceId, GeometryId,
                std::unique_ptr<GeometryInstance>>(
                &SceneGraph<T>::RegisterGeometry),
            py::arg("source_id"), py::arg("geometry_id"), py::arg("geometry"),
            doc.SceneGraph.RegisterGeometry
                .doc_3args_source_id_geometry_id_geometry)
        .def("RegisterAnchoredGeometry",
            py::overload_cast<SourceId, std::unique_ptr<GeometryInstance>>(
                &SceneGraph<T>::RegisterAnchoredGeometry),
            py::arg("source_id"), py::arg("geometry"),
            doc.SceneGraph.RegisterAnchoredGeometry.doc)
        .def("AddRenderer", &SceneGraph<T>::AddRenderer,
            py::arg("renderer_name"), py::arg("renderer"),
            doc.SceneGraph.AddRenderer.doc)
        .def_static("world_frame_id", &SceneGraph<T>::world_frame_id,
            doc.SceneGraph.world_frame_id.doc);
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
        .def("set_value",
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
        .def("inspector", &QueryObject<T>::inspector, py_reference_internal,
            doc.QueryObject.inspector.doc)
        .def("ComputeSignedDistancePairwiseClosestPoints",
            &QueryObject<T>::ComputeSignedDistancePairwiseClosestPoints,
            py::arg("max_distance") = std::numeric_limits<double>::infinity(),
            doc.QueryObject.ComputeSignedDistancePairwiseClosestPoints.doc)
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
        .def("RenderColorImage",
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
        .def("RenderDepthImage",
            [](const Class* self, const render::DepthCameraProperties& camera,
                FrameId parent_frame, const math::RigidTransformd& X_PC) {
              systems::sensors::ImageDepth32F img(camera.width, camera.height);
              self->RenderDepthImage(camera, parent_frame, X_PC, &img);
              return img;
            },
            py::arg("camera"), py::arg("parent_frame"), py::arg("X_PC"),
            doc.QueryObject.RenderDepthImage.doc)
        .def("RenderLabelImage",
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
    py::class_<Shape>(m, "Shape", doc.Shape.doc);
    py::class_<Sphere, Shape>(m, "Sphere", doc.Sphere.doc)
        .def(py::init<double>(), py::arg("radius"), doc.Sphere.ctor.doc)
        .def("get_radius", &Sphere::get_radius, doc.Sphere.get_radius.doc);
    py::class_<Cylinder, Shape>(m, "Cylinder", doc.Cylinder.doc)
        .def(py::init<double, double>(), py::arg("radius"), py::arg("length"),
            doc.Cylinder.ctor.doc)
        .def("get_radius", &Cylinder::get_radius, doc.Cylinder.get_radius.doc)
        .def("get_length", &Cylinder::get_length, doc.Cylinder.get_length.doc);
    py::class_<Box, Shape>(m, "Box", doc.Box.doc)
        .def(py::init<double, double, double>(), py::arg("width"),
            py::arg("depth"), py::arg("height"), doc.Box.ctor.doc)
        .def("width", &Box::width, doc.Box.width.doc)
        .def("depth", &Box::depth, doc.Box.depth.doc)
        .def("height", &Box::height, doc.Box.height.doc)
        .def("size", &Box::size, py_reference_internal, doc.Box.size.doc);
    py::class_<HalfSpace, Shape>(m, "HalfSpace", doc.HalfSpace.doc)
        .def(py::init<>(), doc.HalfSpace.ctor.doc);
    py::class_<Mesh, Shape>(m, "Mesh", doc.Mesh.doc)
        .def(py::init<std::string, double>(), py::arg("absolute_filename"),
            py::arg("scale") = 1.0, doc.Mesh.ctor.doc);
    py::class_<Convex, Shape>(m, "Convex", doc.Convex.doc)
        .def(py::init<std::string, double>(), py::arg("absolute_filename"),
            py::arg("scale") = 1.0, doc.Convex.ctor.doc);
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
        .def("name", &Class::name, cls_doc.name.doc);
  }

  // Rendering
  def_geometry_render(m.def_submodule("render"));

  m.def("ReadObjToSurfaceMesh",
      py::overload_cast<const std::string&, double>(
          &geometry::ReadObjToSurfaceMesh),
      py::arg("filename"), py::arg("scale") = 1.0,
      doc.ReadObjToSurfaceMesh.doc_2args_filename_scale);
}

PYBIND11_MODULE(geometry, m) {
  py::module::import("pydrake.systems.lcm");
  DoScalarIndependentDefinitions(m);
  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      NonSymbolicScalarPack{});
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
