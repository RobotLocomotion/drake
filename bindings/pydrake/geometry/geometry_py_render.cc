/* This contains the bindings for the public API in the drake/geometry/render
 and drake/geometry/render* directories. They can be found in the
 pydrake.geometry module. */

#include "pybind11/operators.h"

#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/render_gl/factory.h"
#include "drake/geometry/render_gltf_client/factory.h"
#include "drake/geometry/render_vtk/factory.h"

namespace drake {
namespace pydrake {

using Eigen::Vector3d;
using geometry::GeometryId;
using geometry::PerceptionProperties;
using geometry::Shape;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRenderCamera;
using geometry::render::RenderEngine;
using math::RigidTransformd;
using systems::sensors::CameraInfo;
using systems::sensors::ColorD;
using systems::sensors::ColorI;
using systems::sensors::Image;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;
using systems::sensors::PixelType;

namespace {

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

void DoScalarIndependentDefinitions(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry::render;
  constexpr auto& doc_geometry = pydrake_doc.drake.geometry;
  constexpr auto& doc = doc_geometry.render;

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

  {
    py::class_<RenderLabel> render_label(m, "RenderLabel", doc.RenderLabel.doc);
    render_label
        .def(py::init<int>(), py::arg("value"), doc.RenderLabel.ctor.doc_1args)
        .def("is_reserved", &RenderLabel::is_reserved)
        .def("__int__", [](const RenderLabel& self) -> int { return self; })
        .def("__repr__",
            [](const RenderLabel& self) -> std::string {
              if (self == RenderLabel::kEmpty) {
                return "RenderLabel.kEmpty";
              }
              if (self == RenderLabel::kDoNotRender) {
                return "RenderLabel.kDoNotRender";
              }
              if (self == RenderLabel::kDontCare) {
                return "RenderLabel.kDontCare";
              }
              if (self == RenderLabel::kUnspecified) {
                return "RenderLabel.kUnspecified";
              }
              if (self == RenderLabel::kMaxUnreserved) {
                return "RenderLabel.kMaxUnreserved";
              }
              return fmt::format("RenderLabel({})", int{self});
            })
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

  {
    using Class = RenderEngineVtkParams;
    constexpr auto& cls_doc = doc_geometry.RenderEngineVtkParams;
    py::class_<Class> cls(m, "RenderEngineVtkParams", cls_doc.doc);
    cls  // BR
        .def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  m.def("MakeRenderEngineVtk", &MakeRenderEngineVtk, py::arg("params"),
      doc_geometry.MakeRenderEngineVtk.doc);

  {
    // TODO(zachfang): During the 2023-07-01 deprecation removals also
    // remove the spurious `geometry::` qualifier on this typename.
    using Class = geometry::RenderEngineGlParams;
    constexpr auto& cls_doc = doc_geometry.RenderEngineGlParams;
    py::class_<Class> cls(m, "RenderEngineGlParams", cls_doc.doc);
    cls  // BR
        .def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  // TODO(zachfang): During the 2023-07-01 deprecation removals also
  // remove the spurious `geometry::` qualifier on this typename.
  m.def("MakeRenderEngineGl", &geometry::MakeRenderEngineGl,
      py::arg("params") = geometry::RenderEngineGlParams(),
      doc_geometry.MakeRenderEngineGl.doc);

  {
    using Class = RenderEngineGltfClientParams;
    constexpr auto& cls_doc = doc_geometry.RenderEngineGltfClientParams;
    py::class_<Class> cls(m, "RenderEngineGltfClientParams", cls_doc.doc);
    cls  // BR
        .def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  m.def("MakeRenderEngineGltfClient", &MakeRenderEngineGltfClient,
      py::arg("params") = RenderEngineGltfClientParams(),
      doc_geometry.MakeRenderEngineGltfClient.doc);

  AddValueInstantiation<RenderLabel>(m);
}
}  // namespace

void DefineGeometryRender(py::module m) {
  m.doc() =
      "Local bindings for render artifacts found in `drake::geometry` and "
      "`drake::geometry::render`";
  DoScalarIndependentDefinitions(m);
}

}  // namespace pydrake
}  // namespace drake
