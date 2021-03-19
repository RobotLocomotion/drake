#include "drake/geometry/render/render_engine.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace drake::geometry::render;

class RenderEngine_publicist : public RenderEngine {
public:
  using RenderEngine::DoClone;
  using RenderEngine::DoRegisterVisual;
  using RenderEngine::DoRemoveGeometry;
  using RenderEngine::DoRenderColorImage;
  using RenderEngine::DoRenderDepthImage;
  using RenderEngine::DoRenderLabelImage;
  using RenderEngine::DoUpdateVisualPose;
  using RenderEngine::GetColorDFromLabel;
  using RenderEngine::GetColorIFromLabel;
  using RenderEngine::GetRenderLabelOrThrow;
  using RenderEngine::LabelFromColor;
  using RenderEngine::SetDefaultLightPosition;
  using RenderEngine::ThrowIfInvalid;
};

namespace py = pybind11;
void apb11_pydrake_RenderEngine_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  py::class_<RenderEngine> PyRenderEngine(m, "RenderEngine");

  PyRenderEngine
      .def("Clone",
           static_cast<::std::unique_ptr<RenderEngine,
                                         std::default_delete<RenderEngine>> (
               RenderEngine::*)() const>(&RenderEngine::Clone))
      .def("DoClone",
           static_cast<::std::unique_ptr<RenderEngine,
                                         std::default_delete<RenderEngine>> (
               RenderEngine::*)() const>(&RenderEngine_publicist::DoClone))
      .def("DoRegisterVisual",
           static_cast<bool (RenderEngine::*)(
               ::drake::geometry::GeometryId, ::drake::geometry::Shape const &,
               ::drake::geometry::PerceptionProperties const &,
               ::drake::math::RigidTransformd const &)>(
               &RenderEngine_publicist::DoRegisterVisual),
           py::arg("id"), py::arg("shape"), py::arg("properties"),
           py::arg("X_WG"))
      .def("DoRemoveGeometry",
           static_cast<bool (RenderEngine::*)(::drake::geometry::GeometryId)>(
               &RenderEngine_publicist::DoRemoveGeometry),
           py::arg("id"))
      .def("DoRenderColorImage",
           static_cast<void (RenderEngine::*)(
               ColorRenderCamera const &,
               ::drake::systems::sensors::ImageRgba8U *) const>(
               &RenderEngine_publicist::DoRenderColorImage),
           py::arg("camera"), py::arg("color_image_out"))
      .def("DoRenderDepthImage",
           static_cast<void (RenderEngine::*)(
               DepthRenderCamera const &,
               ::drake::systems::sensors::ImageDepth32F *) const>(
               &RenderEngine_publicist::DoRenderDepthImage),
           py::arg("camera"), py::arg("depth_image_out"))
      .def("DoRenderLabelImage",
           static_cast<void (RenderEngine::*)(
               ColorRenderCamera const &,
               ::drake::systems::sensors::ImageLabel16I *) const>(
               &RenderEngine_publicist::DoRenderLabelImage),
           py::arg("camera"), py::arg("label_image_out"))
      .def("DoUpdateVisualPose",
           static_cast<void (RenderEngine::*)(
               ::drake::geometry::GeometryId,
               ::drake::math::RigidTransformd const &)>(
               &RenderEngine_publicist::DoUpdateVisualPose),
           py::arg("id"), py::arg("X_WG"))
      .def_static("GetColorDFromLabel",
                  static_cast<::drake::systems::sensors::ColorD (*)(
                      RenderLabel const &)>(
                      &RenderEngine_publicist::GetColorDFromLabel),
                  py::arg("label"))
      .def_static("GetColorIFromLabel",
                  static_cast<::drake::systems::sensors::ColorI (*)(
                      RenderLabel const &)>(
                      &RenderEngine_publicist::GetColorIFromLabel),
                  py::arg("label"))
      .def("GetRenderLabelOrThrow",
           static_cast<RenderLabel (RenderEngine::*)(
               ::drake::geometry::PerceptionProperties const &) const>(
               &RenderEngine_publicist::GetRenderLabelOrThrow),
           py::arg("properties"))
      .def_static("LabelFromColor",
                  static_cast<RenderLabel (*)(
                      ::drake::systems::sensors::ColorI const &)>(
                      &RenderEngine_publicist::LabelFromColor),
                  py::arg("color"))
      .def("RegisterVisual",
           static_cast<bool (RenderEngine::*)(
               ::drake::geometry::GeometryId, ::drake::geometry::Shape const &,
               ::drake::geometry::PerceptionProperties const &,
               ::drake::math::RigidTransformd const &, bool)>(
               &RenderEngine::RegisterVisual),
           py::arg("id"), py::arg("shape"), py::arg("properties"),
           py::arg("X_WG"), py::arg("needs_updates") = bool(true))
      .def("RemoveGeometry",
           static_cast<bool (RenderEngine::*)(::drake::geometry::GeometryId)>(
               &RenderEngine::RemoveGeometry),
           py::arg("id"))
      .def("RenderColorImage",
           static_cast<void (RenderEngine::*)(
               CameraProperties const &, bool,
               ::drake::systems::sensors::ImageRgba8U *) const>(
               &RenderEngine::RenderColorImage),
           py::arg("camera"), py::arg("show_window"),
           py::arg("color_image_out"))
      .def("RenderColorImage",
           static_cast<void (RenderEngine::*)(
               ColorRenderCamera const &,
               ::drake::systems::sensors::ImageRgba8U *) const>(
               &RenderEngine::RenderColorImage),
           py::arg("camera"), py::arg("color_image_out"))
      .def("RenderDepthImage",
           static_cast<void (RenderEngine::*)(
               DepthCameraProperties const &,
               ::drake::systems::sensors::ImageDepth32F *) const>(
               &RenderEngine::RenderDepthImage),
           py::arg("camera"), py::arg("depth_image_out"))
      .def("RenderDepthImage",
           static_cast<void (RenderEngine::*)(
               DepthRenderCamera const &,
               ::drake::systems::sensors::ImageDepth32F *) const>(
               &RenderEngine::RenderDepthImage),
           py::arg("camera"), py::arg("depth_image_out"))
      .def("RenderLabelImage",
           static_cast<void (RenderEngine::*)(
               CameraProperties const &, bool,
               ::drake::systems::sensors::ImageLabel16I *) const>(
               &RenderEngine::RenderLabelImage),
           py::arg("camera"), py::arg("show_window"),
           py::arg("label_image_out"))
      .def("RenderLabelImage",
           static_cast<void (RenderEngine::*)(
               ColorRenderCamera const &,
               ::drake::systems::sensors::ImageLabel16I *) const>(
               &RenderEngine::RenderLabelImage),
           py::arg("camera"), py::arg("label_image_out"))
      .def("SetDefaultLightPosition",
           [](RenderEngine &self,
              Eigen::Ref<::Eigen::Matrix<double, 3, 1, 0, 3, 1> const &, 0,
                         Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>> &X_DL) {
             return self.SetDefaultLightPosition(X_DL);
           })
      .def_static("ThrowIfInvalid",
                  static_cast<void (*)(
                      ::drake::systems::sensors::CameraInfo const &,
                      ::drake::systems::sensors::Image<
                          drake::systems::sensors::PixelType::kRgba8U> const *,
                      char const *)>(&RenderEngine_publicist::ThrowIfInvalid),
                  py::arg("intrinsics"), py::arg("image"),
                  py::arg("image_type"))
      .def_static(
          "ThrowIfInvalid",
          static_cast<void (*)(
              ::drake::systems::sensors::CameraInfo const &,
              ::drake::systems::sensors::Image<
                  drake::systems::sensors::PixelType::kDepth32F> const *,
              char const *)>(&RenderEngine_publicist::ThrowIfInvalid),
          py::arg("intrinsics"), py::arg("image"), py::arg("image_type"))
      .def_static(
          "ThrowIfInvalid",
          static_cast<void (*)(
              ::drake::systems::sensors::CameraInfo const &,
              ::drake::systems::sensors::Image<
                  drake::systems::sensors::PixelType::kLabel16I> const *,
              char const *)>(&RenderEngine_publicist::ThrowIfInvalid),
          py::arg("intrinsics"), py::arg("image"), py::arg("image_type"))
      .def("UpdateViewpoint",
           static_cast<void (RenderEngine::*)(
               ::drake::math::RigidTransformd const &)>(
               &RenderEngine::UpdateViewpoint),
           py::arg("X_WR"))
      .def("default_render_label",
           static_cast<RenderLabel (RenderEngine::*)() const>(
               &RenderEngine::default_render_label))
      .def("has_geometry",
           static_cast<bool (RenderEngine::*)(::drake::geometry::GeometryId)
                           const>(&RenderEngine::has_geometry),
           py::arg("id"))

      ;
}
