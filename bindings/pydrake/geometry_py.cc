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
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/geometry/drake_visualizer.h"
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

using Eigen::Vector3d;
using geometry::GeometryId;
using geometry::PerceptionProperties;
using geometry::Shape;
using geometry::render::CameraProperties;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthCameraProperties;
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

 private:
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  // TODO(SeanCurtis-TRI) We will be removing the Render*Image API when we
  // deprecate CameraProperties. They must be implemented in order to build
  // this class; so we'll make sure they clearly signal if they get invoked.
  void RenderColorImage(
      CameraProperties const&, bool, ImageRgba8U*) const override {
    throw std::runtime_error(
        "Python should not be able to invoke RenderColorImage with "
        "CameraProperties");
  }

  void RenderDepthImage(
      DepthCameraProperties const&, ImageDepth32F*) const override {
    throw std::runtime_error(
        "Python should not be able to invoke RenderDepthImage with "
        "CameraProperties");
  }

  void RenderLabelImage(
      CameraProperties const&, bool, ImageLabel16I*) const override {
    throw std::runtime_error(
        "Python should not be able to invoke RenderLabelImage with "
        "CameraProperties");
  }
#pragma GCC diagnostic pop
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
            py::arg("show_window") = false, cls_doc.ctor.doc_2args)
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
            py::arg("depth_range"), cls_doc.ctor.doc_2args_core_depth_range)
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
            py::arg("clipping"), py::arg("X_BS"), cls_doc.ctor.doc_4args)
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  {
    using Class = CameraProperties;
    py::class_<Class> cls(
        m, "CameraProperties", doc.CameraProperties.doc_deprecated);
    cls  // BR
        .def(py_init_deprecated<Class, int, int, double, std::string>(
                 "Deprecated; due to be removed after 2021-04-01. Please use "
                 "ColorRenderCamera instead"),
            py::arg("width"), py::arg("height"), py::arg("fov_y"),
            py::arg("renderer_name"), doc.CameraProperties.ctor.doc)
        .def_readwrite("width", &Class::width, doc.CameraProperties.width.doc)
        .def_readwrite(
            "height", &Class::height, doc.CameraProperties.height.doc)
        .def_readwrite("fov_y", &Class::fov_y, doc.CameraProperties.fov_y.doc)
        .def_readwrite("renderer_name", &Class::renderer_name,
            doc.CameraProperties.renderer_name.doc);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = DepthCameraProperties;
    py::class_<Class, CameraProperties> cls(
        m, "DepthCameraProperties", doc.DepthCameraProperties.doc_deprecated);
    cls  // BR
        .def(py_init_deprecated<Class, int, int, double, std::string, double,
                 double>(
                 "Deprecated; due to be removed after 2021-04-01. Please use "
                 "DepthRenderCamera instead"),
            py::arg("width"), py::arg("height"), py::arg("fov_y"),
            py::arg("renderer_name"), py::arg("z_near"), py::arg("z_far"),
            doc.DepthCameraProperties.ctor.doc)
        .def_readwrite(
            "z_near", &Class::z_near, doc.DepthCameraProperties.z_near.doc)
        .def_readwrite(
            "z_far", &Class::z_far, doc.DepthCameraProperties.z_far.doc);
    DefCopyAndDeepCopy(&cls);
  }
#pragma GCC diagnostic pop

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
        .def("num_frames", &Class::num_frames, cls_doc.num_frames.doc)
        .def(
            "all_frame_ids",
            [](Class* self) -> std::vector<FrameId> {
              std::vector<FrameId> frame_ids;
              frame_ids.reserve(self->num_frames());
              for (FrameId id : self->all_frame_ids()) {
                frame_ids.push_back(id);
              }
              return frame_ids;
            },
            cls_doc.all_frame_ids.doc)
        .def("world_frame_id", &Class::world_frame_id,
            cls_doc.world_frame_id.doc)
        .def("num_geometries", &Class::num_geometries,
            cls_doc.num_geometries.doc)
        .def("GetAllGeometryIds", &Class::GetAllGeometryIds,
            cls_doc.GetAllGeometryIds.doc)
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
        .def("SourceIsRegistered",
            WrapDeprecated(
                "Please use SceneGraphInspector.SourceIsRegistered("
                "source_id=value) instead. This variant will be removed after"
                " after 2021-04-01",
                &Class::SourceIsRegistered),
            py::arg("id"), cls_doc.SourceIsRegistered.doc)
        .def("GetName",
            overload_cast_explicit<const std::string&, SourceId>(
                &Class::GetName),
            py_rvp::reference_internal, py::arg("source_id"),
            cls_doc.GetName.doc_1args_source_id);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls  // BR
        .def("GetSourceName",
            WrapDeprecated(
                cls_doc.GetSourceName.doc_deprecated, &Class::GetSourceName),
            py::arg("source_id"), cls_doc.GetSourceName.doc_deprecated);
#pragma GCC diagnostic pop
    cls  // BR
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
        .def(
            "get_pose_bundle_output_port",
            [](Class* self) -> const systems::OutputPort<T>& {
              return self->get_pose_bundle_output_port();
            },
            py_rvp::reference_internal, cls_doc.get_pose_bundle_output_port.doc)
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
        .def("ExcludeCollisionsBetween",
            py::overload_cast<const GeometrySet&, const GeometrySet&>(
                &Class::ExcludeCollisionsBetween),
            py_rvp::reference_internal, py::arg("setA"), py::arg("setB"),
            cls_doc.ExcludeCollisionsBetween.doc_2args)
        .def("ExcludeCollisionsBetween",
            overload_cast_explicit<void, Context<T>*, const GeometrySet&,
                const GeometrySet&>(&Class::ExcludeCollisionsBetween),
            py_rvp::reference_internal, py::arg("context"), py::arg("setA"),
            py::arg("setB"), cls_doc.ExcludeCollisionsBetween.doc_3args)
        .def("ExcludeCollisionsWithin",
            py::overload_cast<const GeometrySet&>(
                &Class::ExcludeCollisionsWithin),
            py_rvp::reference_internal, py::arg("set"),
            cls_doc.ExcludeCollisionsWithin.doc_1args)
        .def("ExcludeCollisionsWithin",
            overload_cast_explicit<void, Context<T>*, const GeometrySet&>(
                &Class::ExcludeCollisionsWithin),
            py_rvp::reference_internal, py::arg("context"), py::arg("set"),
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
            cls_doc.GetPoseInWorld.doc_1args_geometry_id);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls  // BR
        .def("X_WF",
            WrapDeprecated(cls_doc.X_WF.doc_deprecated, &QueryObject<T>::X_WF),
            py::arg("id"), py_rvp::reference_internal,
            cls_doc.X_WF.doc_deprecated)
        .def("X_PF",
            WrapDeprecated(cls_doc.X_PF.doc_deprecated, &QueryObject<T>::X_PF),
            py::arg("id"), py_rvp::reference_internal,
            cls_doc.X_PF.doc_deprecated)
        .def("X_WG",
            WrapDeprecated(cls_doc.X_WG.doc_deprecated, &QueryObject<T>::X_WG),
            py::arg("id"), py_rvp::reference_internal,
            cls_doc.X_WG.doc_deprecated);
#pragma GCC diagnostic pop
    cls  // BR
        .def("ComputeSignedDistancePairwiseClosestPoints",
            &QueryObject<T>::ComputeSignedDistancePairwiseClosestPoints,
            py::arg("max_distance") = std::numeric_limits<double>::infinity(),
            cls_doc.ComputeSignedDistancePairwiseClosestPoints.doc)
        .def("ComputeSignedDistancePairClosestPoints",
            &QueryObject<T>::ComputeSignedDistancePairClosestPoints,
            py::arg("geometry_id_A"), py::arg("geometry_id_B"),
            cls_doc.ComputeSignedDistancePairClosestPoints.doc)
        .def("ComputeSignedDistancePairClosestPoints",
            WrapDeprecated("Please use "
                           "QueryObject.ComputeSignedDistancePairClosestPoints("
                           "geometry_id_A=value1, geometry_id_B=value2). This "
                           "variant will be removed on or after 2021-04-01.",
                &QueryObject<T>::ComputeSignedDistancePairClosestPoints),
            py::arg("id_A"), py::arg("id_B"),
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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    cls  // BR
        .def("RenderColorImage",
            WrapDeprecated(cls_doc.RenderColorImage.doc_deprecated,
                [](const Class* self, const render::CameraProperties& camera,
                    FrameId parent_frame, const math::RigidTransformd& X_PC,
                    bool show_window) {
                  systems::sensors::ImageRgba8U img(
                      camera.width, camera.height);
                  self->RenderColorImage(
                      camera, parent_frame, X_PC, show_window, &img);
                  return img;
                }),
            py::arg("camera"), py::arg("parent_frame"), py::arg("X_PC"),
            py::arg("show_window") = false,
            cls_doc.RenderColorImage.doc_deprecated)
        .def("RenderDepthImage",
            WrapDeprecated(cls_doc.RenderDepthImage.doc_deprecated,
                [](const Class* self,
                    const render::DepthCameraProperties& camera,
                    FrameId parent_frame, const math::RigidTransformd& X_PC) {
                  systems::sensors::ImageDepth32F img(
                      camera.width, camera.height);
                  self->RenderDepthImage(camera, parent_frame, X_PC, &img);
                  return img;
                }),
            py::arg("camera"), py::arg("parent_frame"), py::arg("X_PC"),
            cls_doc.RenderDepthImage.doc_deprecated)
        .def("RenderLabelImage",
            WrapDeprecated(cls_doc.RenderLabelImage.doc_deprecated,
                [](const Class* self, const render::CameraProperties& camera,
                    FrameId parent_frame, const math::RigidTransformd& X_PC,
                    bool show_window = false) {
                  systems::sensors::ImageLabel16I img(
                      camera.width, camera.height);
                  self->RenderLabelImage(
                      camera, parent_frame, X_PC, show_window, &img);
                  return img;
                }),
            py::arg("camera"), py::arg("parent_frame"), py::arg("X_PC"),
            py::arg("show_window") = false,
            cls_doc.RenderLabelImage.doc_deprecated);
#pragma GCC diagnostic pop

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
        .def("vertices", &Class::vertices, doc.SurfaceMesh.vertices.doc)
        .def("centroid", &Class::centroid, doc.SurfaceMesh.centroid.doc);
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

  // DrakeVisualizer
  {
    using Class = DrakeVisualizer<T>;
    constexpr auto& cls_doc = doc.DrakeVisualizer;
    auto cls = DefineTemplateClassWithDefault<Class, LeafSystem<T>>(
        m, "DrakeVisualizer", param, cls_doc.doc);
    cls  // BR
        .def(py::init<lcm::DrakeLcmInterface*, DrakeVisualizerParams>(),
            py::arg("lcm") = nullptr,
            py::arg("params") = DrakeVisualizerParams{},
            // Keep alive, reference: `self` keeps `lcm` alive.
            py::keep_alive<1, 2>(),  // BR
            cls_doc.ctor.doc)
        .def("query_object_input_port", &Class::query_object_input_port,
            py_rvp::reference_internal, cls_doc.query_object_input_port.doc)
        .def_static("AddToBuilder",
            py::overload_cast<systems::DiagramBuilder<T>*, const SceneGraph<T>&,
                lcm::DrakeLcmInterface*, DrakeVisualizerParams>(
                &DrakeVisualizer<T>::AddToBuilder),
            py::arg("builder"), py::arg("scene_graph"),
            py::arg("lcm") = nullptr,
            py::arg("params") = DrakeVisualizerParams{},
            // Keep alive, ownership: `return` keeps `builder` alive.
            py::keep_alive<0, 1>(),
            // Keep alive, reference: `builder` keeps `lcm` alive.
            py::keep_alive<1, 3>(), py_rvp::reference,
            cls_doc.AddToBuilder.doc_4args_builder_scene_graph_lcm_params)
        .def_static("AddToBuilder",
            py::overload_cast<systems::DiagramBuilder<T>*,
                const systems::OutputPort<T>&, lcm::DrakeLcmInterface*,
                DrakeVisualizerParams>(&DrakeVisualizer<T>::AddToBuilder),
            py::arg("builder"), py::arg("query_object_port"),
            py::arg("lcm") = nullptr,
            py::arg("params") = DrakeVisualizerParams{},
            // Keep alive, ownership: `return` keeps `builder` alive.
            py::keep_alive<0, 1>(),
            // Keep alive, reference: `builder` keeps `lcm` alive.
            py::keep_alive<1, 3>(), py_rvp::reference,
            cls_doc.AddToBuilder.doc_4args_builder_query_object_port_lcm_params)
        .def_static("DispatchLoadMessage",
            &DrakeVisualizer<T>::DispatchLoadMessage, py::arg("scene_graph"),
            py::arg("lcm"), py::arg("params") = DrakeVisualizerParams{},
            cls_doc.DispatchLoadMessage.doc);
  }
}  // NOLINT(readability/fn_size)

void DoScalarIndependentDefinitions(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;
  constexpr auto& doc = pydrake_doc.drake.geometry;

  {
    using Class = Rgba;
    constexpr auto& cls_doc = doc.Rgba;
    py::class_<Class> cls(m, "Rgba", cls_doc.doc);
    cls  // BR
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
    DefCopyAndDeepCopy(&cls);
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  m.def("ConnectDrakeVisualizer",
      WrapDeprecated(doc.ConnectDrakeVisualizer.doc_deprecated_4args,
          py::overload_cast<systems::DiagramBuilder<double>*,
              const SceneGraph<double>&, lcm::DrakeLcmInterface*,
              geometry::Role>(&ConnectDrakeVisualizer)),
      py::arg("builder"), py::arg("scene_graph"), py::arg("lcm") = nullptr,
      py::arg("role") = geometry::Role::kIllustration,
      // Keep alive, ownership: `return` keeps `builder` alive.
      py::keep_alive<0, 1>(),
      // Keep alive, reference: `builder` keeps `lcm` alive.
      py::keep_alive<1, 3>(),
      // See #11531 for why `py_rvp::reference` is needed.
      py_rvp::reference, doc.ConnectDrakeVisualizer.doc_deprecated_4args);
  m.def("ConnectDrakeVisualizer",
      WrapDeprecated(doc.ConnectDrakeVisualizer.doc_deprecated_5args,
          py::overload_cast<systems::DiagramBuilder<double>*,
              const SceneGraph<double>&, const systems::OutputPort<double>&,
              lcm::DrakeLcmInterface*, geometry::Role>(
              &ConnectDrakeVisualizer)),
      py::arg("builder"), py::arg("scene_graph"),
      py::arg("pose_bundle_output_port"), py::arg("lcm") = nullptr,
      py::arg("role") = geometry::Role::kIllustration,
      // Keep alive, ownership: `return` keeps `builder` alive.
      py::keep_alive<0, 1>(),
      // Keep alive, reference: `builder` keeps `lcm` alive.
      py::keep_alive<1, 3>(),
      // See #11531 for why `py_rvp::reference` is needed.
      py_rvp::reference, doc.ConnectDrakeVisualizer.doc_deprecated_5args);
  m.def("DispatchLoadMessage",
      WrapDeprecated(
          doc.DispatchLoadMessage.doc_deprecated, &DispatchLoadMessage),
      py::arg("scene_graph"), py::arg("lcm"),
      py::arg("role") = geometry::Role::kIllustration,
      doc.DispatchLoadMessage.doc_deprecated);
#pragma GCC diagnostic pop

  {
    using Class = DrakeVisualizerParams;
    constexpr auto& cls_doc = doc.DrakeVisualizerParams;
    py::class_<Class>(m, "DrakeVisualizerParams", cls_doc.doc)
        .def(ParamInit<Class>())
        .def_readwrite("publish_period", &DrakeVisualizerParams::publish_period,
            cls_doc.publish_period.doc)
        .def_readwrite("role", &DrakeVisualizerParams::role, cls_doc.role.doc)
        .def_readwrite("default_color", &DrakeVisualizerParams::default_color,
            cls_doc.default_color.doc);
  }

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
        .def("size", &Box::size, py_rvp::reference_internal, doc.Box.size.doc);
    py::class_<Capsule, Shape>(m, "Capsule", doc.Capsule.doc)
        .def(py::init<double, double>(), py::arg("radius"), py::arg("length"),
            doc.Capsule.ctor.doc)
        .def("radius", &Capsule::radius, doc.Capsule.radius.doc)
        .def("length", &Capsule::length, doc.Capsule.length.doc);
    py::class_<Ellipsoid, Shape>(m, "Ellipsoid", doc.Ellipsoid.doc)
        .def(py::init<double, double, double>(), py::arg("a"), py::arg("b"),
            py::arg("c"), doc.Ellipsoid.ctor.doc)
        .def("a", &Ellipsoid::a, doc.Ellipsoid.a.doc)
        .def("b", &Ellipsoid::b, doc.Ellipsoid.b.doc)
        .def("c", &Ellipsoid::c, doc.Ellipsoid.c.doc);
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
    py::class_<Class> cls(m, "GeometryFrame", cls_doc.doc);
    cls  // BR
        .def(py::init<const std::string&, int>(), py::arg("frame_name"),
            py::arg("frame_group_id") = 0, cls_doc.ctor.doc)
        .def("id", &Class::id, cls_doc.id.doc)
        .def("name", &Class::name, cls_doc.name.doc)
        .def("frame_group", &Class::frame_group, cls_doc.frame_group.doc);
    DefCopyAndDeepCopy(&cls);
  }

  // GeometryInstance
  {
    using Class = GeometryInstance;
    constexpr auto& cls_doc = doc.GeometryInstance;
    py::class_<Class> cls(m, "GeometryInstance", cls_doc.doc);
    cls  // BR
        .def(py::init<const math::RigidTransform<double>&,
                 std::unique_ptr<Shape>, const std::string&>(),
            py::arg("X_PG"), py::arg("shape"), py::arg("name"),
            cls_doc.ctor.doc)
        .def("id", &Class::id, cls_doc.id.doc)
        .def("pose", &Class::pose, py_rvp::reference_internal, cls_doc.pose.doc)
        .def(
            "set_pose", &Class::set_pose, py::arg("X_PG"), cls_doc.set_pose.doc)
        .def("shape", &Class::shape, py_rvp::reference_internal,
            cls_doc.shape.doc)
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
            &Class::mutable_proximity_properties, py_rvp::reference_internal,
            cls_doc.mutable_proximity_properties.doc)
        .def("proximity_properties", &Class::proximity_properties,
            py_rvp::reference_internal, cls_doc.proximity_properties.doc)
        .def("mutable_illustration_properties",
            &Class::mutable_illustration_properties, py_rvp::reference_internal,
            cls_doc.mutable_illustration_properties.doc)
        .def("illustration_properties", &Class::illustration_properties,
            py_rvp::reference_internal, cls_doc.illustration_properties.doc)
        .def("mutable_perception_properties",
            &Class::mutable_perception_properties, py_rvp::reference_internal,
            cls_doc.mutable_perception_properties.doc)
        .def("perception_properties", &Class::perception_properties,
            py_rvp::reference_internal, cls_doc.perception_properties.doc);
    DefCopyAndDeepCopy(&cls);
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
              py::object py_self = py::cast(&self, py_rvp::reference);
              for (auto& [name, abstract] :
                  self.GetPropertiesInGroup(group_name)) {
                out[name.c_str()] = py::cast(
                    abstract.get(), py_rvp::reference_internal, py_self);
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
        .def(
            "UpdateProperty",
            [abstract_value_cls](Class& self, const std::string& group_name,
                const std::string& name, py::object value) {
              py::object abstract = abstract_value_cls.attr("Make")(value);
              self.UpdatePropertyAbstract(
                  group_name, name, abstract.cast<const AbstractValue&>());
            },
            py::arg("group_name"), py::arg("name"), py::arg("value"),
            cls_doc.UpdateProperty.doc)
        .def("HasProperty", &Class::HasProperty, py::arg("group_name"),
            py::arg("name"), cls_doc.HasProperty.doc)
        .def(
            "GetProperty",
            [](const Class& self, const std::string& group_name,
                const std::string& name) {
              py::object abstract =
                  py::cast(self.GetPropertyAbstract(group_name, name),
                      py_rvp::reference);
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
                py::object py_self = py::cast(&self, py_rvp::reference);
                return py_self.attr("GetProperty")(group_name, name);
              } else {
                return default_value;
              }
            },
            py::arg("group_name"), py::arg("name"), py::arg("default_value"),
            cls_doc.GetPropertyOrDefault.doc)
        .def("RemoveProperty", &Class::RemoveProperty, py::arg("group_name"),
            py::arg("name"), cls_doc.RemoveProperty.doc)
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

  // GeometryVersion
  {
    using Class = GeometryVersion;
    constexpr auto& cls_doc = doc.GeometryVersion;
    py::class_<Class> cls(m, "GeometryVersion", cls_doc.doc);
    cls.def(py::init(), cls_doc.ctor.doc)
        .def(py::init<const GeometryVersion&>(), py::arg("other"),
            "Creates a copy of the GeometryVersion.")
        .def("IsSameAs", &Class::IsSameAs, py::arg("other"), py::arg("role"),
            cls_doc.IsSameAs.doc);
    DefCopyAndDeepCopy(&cls);
  }

  // ProximityProperties
  {
    py::class_<ProximityProperties, GeometryProperties> cls(
        m, "ProximityProperties", doc.ProximityProperties.doc);
    cls.def(py::init(), doc.ProximityProperties.ctor.doc)
        .def(py::init<const ProximityProperties&>(), py::arg("other"),
            "Creates a copy of the properties");
    DefCopyAndDeepCopy(&cls);
  }

  // IllustrationProperties
  {
    py::class_<IllustrationProperties, GeometryProperties> cls(
        m, "IllustrationProperties", doc.IllustrationProperties.doc);
    cls.def(py::init(), doc.IllustrationProperties.ctor.doc)
        .def(py::init<const IllustrationProperties&>(), py::arg("other"),
            "Creates a copy of the properties");
    DefCopyAndDeepCopy(&cls);
  }

  // PerceptionProperties
  {
    py::class_<PerceptionProperties, GeometryProperties> cls(
        m, "PerceptionProperties", doc.PerceptionProperties.doc);
    cls.def(py::init(), doc.PerceptionProperties.ctor.doc)
        .def(py::init<const PerceptionProperties&>(), py::arg("other"),
            "Creates a copy of the properties");
    DefCopyAndDeepCopy(&cls);
  }

  m.def("MakePhongIllustrationProperties", &MakePhongIllustrationProperties,
      py_rvp::reference_internal, py::arg("diffuse"),
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
