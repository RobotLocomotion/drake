#include <sstream>

#include "pybind11/eigen.h"
#include "pybind11/eval.h"
#include "pybind11/operators.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/type_safe_index_pybind.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/geometry_py.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/query_results/penetration_as_point_pair.h"
#include "drake/geometry/render/gl_renderer/render_engine_gl_factory.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render/render_engine_vtk_factory.h"
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/scene_graph.h"

// TODO(SeanCurtis-TRI) When pybind issue 3019 gets resolved, we won't need to
//  define this locally anymore. In fact, it will probably cause link errors.
namespace pybind11 {
namespace detail {
template <>
struct type_caster<std::monostate> {
 public:
  PYBIND11_TYPE_CASTER(std::monostate, _("None"));

  bool load(handle src, bool) { return src.ptr() == Py_None; }

  static handle cast(
      std::monostate, return_value_policy /* policy */, handle /* parent */) {
    Py_RETURN_NONE;
  }
};
}  // namespace detail
}  // namespace pybind11

namespace drake {
namespace pydrake {
namespace {

using Eigen::Vector3d;
using geometry::GeometryId;
using geometry::PerceptionProperties;
using geometry::Shape;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRenderCamera;
using geometry::render::RenderEngine;
using math::RigidTransformd;
using systems::Context;
using systems::LeafSystem;
using systems::sensors::CameraInfo;
using systems::sensors::ColorD;
using systems::sensors::ColorI;
using systems::sensors::Image;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;
using systems::sensors::PixelType;

class PyRenderEngine : public py::wrapper<RenderEngine> {
 public:
  using Base = RenderEngine;
  using BaseWrapper = py::wrapper<Base>;
  PyRenderEngine() : BaseWrapper() {}

  void UpdateViewpoint(RigidTransformd const& X_WR) override {
    PYBIND11_OVERLOAD_PURE(void, Base, UpdateViewpoint, X_WR);
  }

  bool DoRegisterVisual(GeometryId id, Shape const& shape,
      PerceptionProperties const& properties,
      RigidTransformd const& X_WG) override {
    PYBIND11_OVERLOAD_PURE(
        bool, Base, DoRegisterVisual, id, shape, properties, X_WG);
  }

  void DoUpdateVisualPose(GeometryId id, RigidTransformd const& X_WG) override {
    PYBIND11_OVERLOAD_PURE(void, Base, DoUpdateVisualPose, id, X_WG);
  }

  bool DoRemoveGeometry(GeometryId id) override {
    PYBIND11_OVERLOAD_PURE(bool, Base, DoRemoveGeometry, id);
  }

  std::unique_ptr<RenderEngine> DoClone() const override {
    PYBIND11_OVERLOAD_PURE(std::unique_ptr<RenderEngine>, Base, DoClone);
  }

  void DoRenderColorImage(ColorRenderCamera const& camera,
      ImageRgba8U* color_image_out) const override {
    PYBIND11_OVERLOAD_PURE(
        void, Base, DoRenderColorImage, camera, color_image_out);
  }

  void DoRenderDepthImage(DepthRenderCamera const& camera,
      ImageDepth32F* depth_image_out) const override {
    PYBIND11_OVERLOAD_PURE(
        void, Base, DoRenderDepthImage, camera, depth_image_out);
  }

  void DoRenderLabelImage(ColorRenderCamera const& camera,
      ImageLabel16I* label_image_out) const override {
    PYBIND11_OVERLOAD_PURE(
        void, Base, DoRenderLabelImage, camera, label_image_out);
  }

  void SetDefaultLightPosition(Vector3d const& X_DL) override {
    PYBIND11_OVERLOAD(void, Base, SetDefaultLightPosition, X_DL);
  }

  // Expose these protected methods (which are either virtual methods with
  // default implementations, or helper functions) so that Python
  // implementations can access them.
  using Base::GetColorDFromLabel;
  using Base::GetColorIFromLabel;
  using Base::GetRenderLabelOrThrow;
  using Base::LabelFromColor;
  using Base::SetDefaultLightPosition;

  template <typename ImageType>
  static void ThrowIfInvalid(const systems::sensors::CameraInfo& intrinsics,
      const ImageType* image, const char* image_type) {
    return Base::ThrowIfInvalid<ImageType>(intrinsics, image, image_type);
  }
};

void def_geometry_render(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry::render;
  m.doc() = "Local bindings for `drake::geometry::render`";
  constexpr auto& doc = pydrake_doc.drake.geometry.render;

  {
    using Class = ClippingRange;
    const auto& cls_doc = doc.ClippingRange;
    py::class_<Class>(m, "ClippingRange", cls_doc.doc)
        .def(py::init<Class const&>(), py::arg("other"), "Copy constructor")
        .def(py::init<double, double>(), py::arg("near"), py::arg("far"),
            cls_doc.ctor.doc)
        .def("far", static_cast<double (Class::*)() const>(&Class::far),
            cls_doc.far.doc)
        .def("near", static_cast<double (Class::*)() const>(&Class::near),
            cls_doc.near.doc);
  }
  {
    using Class = ColorRenderCamera;
    const auto& cls_doc = doc.ColorRenderCamera;
    py::class_<Class> cls(m, "ColorRenderCamera", cls_doc.doc);
    cls  // BR
        .def(py::init<Class const&>(), py::arg("other"), "Copy constructor")
        .def(py::init<RenderCameraCore, bool>(), py::arg("core"),
            py::arg("show_window") = false, cls_doc.ctor.doc)
        .def("core",
            static_cast<RenderCameraCore const& (Class::*)() const>(
                &Class::core),
            cls_doc.core.doc)
        .def("show_window",
            static_cast<bool (Class::*)() const>(&Class::show_window),
            cls_doc.show_window.doc);
    DefCopyAndDeepCopy(&cls);
  }
  {
    using Class = DepthRange;
    const auto& cls_doc = doc.DepthRange;
    py::class_<Class> cls(m, "DepthRange");
    cls  // BR
        .def(py::init<Class const&>(), py::arg("other"), "Copy constructor")
        .def(py::init<double, double>(), py::arg("min_in"), py::arg("min_out"),
            cls_doc.ctor.doc)
        .def("max_depth",
            static_cast<double (Class::*)() const>(&Class::max_depth),
            cls_doc.max_depth.doc)
        .def("min_depth",
            static_cast<double (Class::*)() const>(&Class::min_depth),
            cls_doc.min_depth.doc);
    DefCopyAndDeepCopy(&cls);
  }
  {
    using Class = DepthRenderCamera;
    const auto& cls_doc = doc.DepthRenderCamera;
    py::class_<Class> cls(m, "DepthRenderCamera");
    cls  // BR
        .def(py::init<Class const&>(), py::arg("other"), "Copy constructor")
        .def(py::init<RenderCameraCore, DepthRange>(), py::arg("core"),
            py::arg("depth_range"), cls_doc.ctor.doc)
        .def("core",
            static_cast<RenderCameraCore const& (Class::*)() const>(
                &Class::core),
            cls_doc.core.doc)
        .def("depth_range",
            static_cast<DepthRange const& (Class::*)() const>(
                &Class::depth_range),
            cls_doc.depth_range.doc);
    DefCopyAndDeepCopy(&cls);
  }
  {
    using Class = RenderCameraCore;
    const auto& cls_doc = doc.RenderCameraCore;
    py::class_<Class> cls(m, "RenderCameraCore");
    cls  // BR
        .def(py::init<Class const&>(), py::arg("other"), "Copy constructor")
        .def(
            py::init<std::string, CameraInfo, ClippingRange, RigidTransformd>(),
            py::arg("renderer_name"), py::arg("intrinsics"),
            py::arg("clipping"), py::arg("X_BS"), cls_doc.ctor.doc)
        .def("clipping",
            static_cast<ClippingRange const& (Class::*)() const>(
                &Class::clipping),
            cls_doc.clipping.doc)
        .def("intrinsics",
            static_cast<CameraInfo const& (Class::*)() const>(
                &Class::intrinsics),
            cls_doc.intrinsics.doc)
        .def("renderer_name",
            static_cast<::std::string const& (Class::*)() const>(
                &Class::renderer_name),
            cls_doc.renderer_name.doc)
        .def("sensor_pose_in_camera_body",
            static_cast<RigidTransformd const& (Class::*)() const>(
                &Class::sensor_pose_in_camera_body),
            cls_doc.sensor_pose_in_camera_body.doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = RenderEngine;
    const auto& cls_doc = doc.RenderEngine;
    py::class_<Class, PyRenderEngine>(m, "RenderEngine")
        .def(py::init<>(), cls_doc.ctor.doc)
        .def("Clone",
            static_cast<::std::unique_ptr<Class> (Class::*)() const>(
                &Class::Clone),
            cls_doc.Clone.doc)
        .def("RegisterVisual",
            static_cast<bool (Class::*)(GeometryId, Shape const&,
                PerceptionProperties const&, RigidTransformd const&, bool)>(
                &Class::RegisterVisual),
            py::arg("id"), py::arg("shape"), py::arg("properties"),
            py::arg("X_WG"), py::arg("needs_updates") = true,
            cls_doc.RegisterVisual.doc)
        .def("RemoveGeometry",
            static_cast<bool (Class::*)(GeometryId)>(&Class::RemoveGeometry),
            py::arg("id"), cls_doc.RemoveGeometry.doc)
        .def("has_geometry",
            static_cast<bool (Class::*)(GeometryId) const>(
                &Class::has_geometry),
            py::arg("id"), cls_doc.has_geometry.doc)
        .def("UpdateViewpoint",
            static_cast<void (Class::*)(RigidTransformd const&)>(
                &Class::UpdateViewpoint),
            py::arg("X_WR"), cls_doc.UpdateViewpoint.doc)
        .def("RenderColorImage",
            static_cast<void (Class::*)(ColorRenderCamera const&, ImageRgba8U*)
                    const>(&Class::RenderColorImage),
            py::arg("camera"), py::arg("color_image_out"),
            cls_doc.RenderColorImage.doc)
        .def("RenderDepthImage",
            static_cast<void (Class::*)(DepthRenderCamera const&,
                ImageDepth32F*) const>(&Class::RenderDepthImage),
            py::arg("camera"), py::arg("depth_image_out"),
            cls_doc.RenderDepthImage.doc)
        .def("RenderLabelImage",
            static_cast<void (Class::*)(ColorRenderCamera const&,
                ImageLabel16I*) const>(&Class::RenderLabelImage),
            py::arg("camera"), py::arg("label_image_out"),
            cls_doc.RenderLabelImage.doc)
        .def("default_render_label",
            static_cast<RenderLabel (Class::*)() const>(
                &Class::default_render_label),
            cls_doc.default_render_label.doc)
        // N.B. We're binding against the trampoline class PyRenderEngine,
        // rather than the direct class RenderEngine, solely for protected
        // helper methods and non-pure virtual functions because we want them
        // exposed for Python implementations of the interface.
        .def("GetRenderLabelOrThrow",
            static_cast<RenderLabel (Class::*)(PerceptionProperties const&)
                    const>(&PyRenderEngine::GetRenderLabelOrThrow),
            py::arg("properties"), cls_doc.GetRenderLabelOrThrow.doc)
        .def_static("LabelFromColor",
            static_cast<RenderLabel (*)(ColorI const&)>(
                &PyRenderEngine::LabelFromColor),
            py::arg("color"), cls_doc.LabelFromColor.doc)
        .def_static("GetColorIFromLabel",
            static_cast<ColorI (*)(RenderLabel const&)>(
                &PyRenderEngine::GetColorIFromLabel),
            py::arg("label"), cls_doc.GetColorIFromLabel.doc)
        .def_static("GetColorDFromLabel",
            static_cast<ColorD (*)(RenderLabel const&)>(
                &PyRenderEngine::GetColorDFromLabel),
            py::arg("label"), cls_doc.GetColorDFromLabel.doc)
        .def("SetDefaultLightPosition",
            static_cast<void (Class::*)(Vector3d const&)>(
                &PyRenderEngine::SetDefaultLightPosition),
            py::arg("X_DL"), cls_doc.SetDefaultLightPosition.doc)
        .def_static("ThrowIfInvalid",
            static_cast<void (*)(CameraInfo const&,
                Image<PixelType::kRgba8U> const*, char const*)>(
                &PyRenderEngine::ThrowIfInvalid<Image<PixelType::kRgba8U>>),
            py::arg("intrinsics"), py::arg("image"), py::arg("image_type"),
            cls_doc.ThrowIfInvalid.doc)
        .def_static("ThrowIfInvalid",
            static_cast<void (*)(CameraInfo const&,
                Image<PixelType::kDepth32F> const*, char const*)>(
                &PyRenderEngine::ThrowIfInvalid<Image<PixelType::kDepth32F>>),
            py::arg("intrinsics"), py::arg("image"), py::arg("image_type"),
            cls_doc.ThrowIfInvalid.doc)
        .def_static("ThrowIfInvalid",
            static_cast<void (*)(CameraInfo const&,
                Image<PixelType::kLabel16I> const*, char const*)>(
                &PyRenderEngine::ThrowIfInvalid<Image<PixelType::kLabel16I>>),
            py::arg("intrinsics"), py::arg("image"), py::arg("image_type"),
            cls_doc.ThrowIfInvalid.doc);
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
         // Scene-graph wide data.
        .def("num_sources", &Class::num_sources, cls_doc.num_sources.doc)
        .def("num_frames", &Class::num_frames, cls_doc.num_frames.doc);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls.def("all_frame_ids",
        WrapDeprecated(cls_doc.all_frame_ids.doc_deprecated,
            [](Class* self) -> std::vector<FrameId> {
              std::vector<FrameId> frame_ids;
              frame_ids.reserve(self->num_frames());
              for (FrameId id : self->all_frame_ids()) {
                frame_ids.push_back(id);
              }
              return frame_ids;
            }),
        cls_doc.all_frame_ids.doc_deprecated);
#pragma GCC diagnostic pop

    cls  // BR
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
        .def("GetPoseInParent", &Class::GetPoseInParent,
            py_rvp::reference_internal, py::arg("geometry_id"),
            cls_doc.GetPoseInParent.doc)
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
            py_rvp::reference_internal, cls_doc.get_source_pose_port.doc);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls  // BR
        .def("get_pose_bundle_output_port",
            WrapDeprecated(cls_doc.get_pose_bundle_output_port.doc_deprecated,
                &Class::get_pose_bundle_output_port),
            py_rvp::reference_internal,
            cls_doc.get_pose_bundle_output_port.doc_deprecated);
#pragma GCC diagnostic pop

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
        .def("collision_filter_manager",
            overload_cast_explicit<CollisionFilterManager, Context<T>*>(
                &Class::collision_filter_manager),
            py::arg("context"), cls_doc.collision_filter_manager.doc_1args)
        .def("collision_filter_manager",
            overload_cast_explicit<CollisionFilterManager>(
                &Class::collision_filter_manager),
            cls_doc.collision_filter_manager.doc_0args);

// TODO(2021-11-01) Remove these bindings with deprecated code. We can also
//  eliminate the breaking "cls //BR" below and put it all together in a stream
//  of .defs.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls  // BR
        .def("ExcludeCollisionsBetween",
            WrapDeprecated(
                cls_doc.ExcludeCollisionsBetween.doc_deprecated_2args,
                py::overload_cast<const GeometrySet&, const GeometrySet&>(
                    &Class::ExcludeCollisionsBetween)),
            py_rvp::reference_internal, py::arg("setA"), py::arg("setB"),
            cls_doc.ExcludeCollisionsBetween.doc_deprecated_2args)
        .def("ExcludeCollisionsBetween",
            WrapDeprecated(
                cls_doc.ExcludeCollisionsBetween.doc_deprecated_3args,
                overload_cast_explicit<void, Context<T>*, const GeometrySet&,
                    const GeometrySet&>(&Class::ExcludeCollisionsBetween)),
            py_rvp::reference_internal, py::arg("context"), py::arg("setA"),
            py::arg("setB"),
            cls_doc.ExcludeCollisionsBetween.doc_deprecated_3args)
        .def("ExcludeCollisionsWithin",
            WrapDeprecated(cls_doc.ExcludeCollisionsWithin.doc_deprecated_1args,
                py::overload_cast<const GeometrySet&>(
                    &Class::ExcludeCollisionsWithin)),
            py_rvp::reference_internal, py::arg("set"),
            cls_doc.ExcludeCollisionsWithin.doc_deprecated_1args)
        .def("ExcludeCollisionsWithin",
            WrapDeprecated(cls_doc.ExcludeCollisionsWithin.doc_deprecated_2args,
                overload_cast_explicit<void, Context<T>*, const GeometrySet&>(
                    &Class::ExcludeCollisionsWithin)),
            py_rvp::reference_internal, py::arg("context"), py::arg("set"),
            cls_doc.ExcludeCollisionsWithin.doc_deprecated_2args);
#pragma GCC diagnostic pop

    cls  // BR
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
        // - Begin: RemoveRole Overloads
        .def(
            "RemoveRole",
            [](Class& self, SourceId source_id, GeometryId geometry_id,
                Role role) {
              return self.RemoveRole(source_id, geometry_id, role);
            },
            py::arg("source_id"), py::arg("geometry_id"), py::arg("role"),
            cls_doc.RemoveRole.doc_geometry_direct)
        // - End: RemoveRole Overloads.
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

  // ContactSurface
  {
    using Class = ContactSurface<T>;
    auto cls = DefineTemplateClassWithDefault<Class>(
        m, "ContactSurface", param, doc.ContactSurface.doc);
    cls  // BR
        .def("id_M", &Class::id_M, doc.ContactSurface.id_M.doc)
        .def("id_N", &Class::id_N, doc.ContactSurface.id_N.doc)
        .def("mesh_W", &Class::mesh_W, doc.ContactSurface.mesh_W.doc);
  }
}  // NOLINT(readability/fn_size)

void def_geometry(py::module m) {
  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      NonSymbolicScalarPack{});
}

void def_geometry_all(py::module m) {
  py::dict vars = m.attr("__dict__");
  py::exec(
      "from pydrake.geometry import *\n"
      "from pydrake.geometry.render import *\n"
      "from pydrake.geometry.optimization import *\n",
      py::globals(), vars);
}

PYBIND11_MODULE(geometry, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  py::module::import("pydrake.common");
  py::module::import("pydrake.math");
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.lcm");

  /* The order of execution matters -- a module may rely on the definition
   of bindings executed prior to it. */
  DefineGeometryCommon(m);
  DefineGeometryHydro(m);
  def_geometry(m);
  def_geometry_render(m.def_submodule("render"));
  DefineGeometryOptimization(m.def_submodule("optimization"));
  DefineGeometryVisualizers(m);
  def_geometry_all(m.def_submodule("all"));
}

}  // namespace
}  // namespace pydrake
}  // namespace drake
