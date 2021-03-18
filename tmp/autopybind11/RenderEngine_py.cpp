#include "drake/geometry/render/render_engine.h"
#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

class RenderEngine_publicist : public ::drake::geometry::render::RenderEngine {
public:
  using ::drake::geometry::render::RenderEngine::DoClone;
  using ::drake::geometry::render::RenderEngine::DoRegisterVisual;
  using ::drake::geometry::render::RenderEngine::DoRemoveGeometry;
  using ::drake::geometry::render::RenderEngine::DoRenderColorImage;
  using ::drake::geometry::render::RenderEngine::DoRenderDepthImage;
  using ::drake::geometry::render::RenderEngine::DoRenderLabelImage;
  using ::drake::geometry::render::RenderEngine::DoUpdateVisualPose;
  using ::drake::geometry::render::RenderEngine::
      DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE;
  using ::drake::geometry::render::RenderEngine::GetColorDFromLabel;
  using ::drake::geometry::render::RenderEngine::GetColorIFromLabel;
  using ::drake::geometry::render::RenderEngine::GetRenderLabelOrThrow;
  using ::drake::geometry::render::RenderEngine::LabelFromColor;
  using ::drake::geometry::render::RenderEngine::SetDefaultLightPosition;
  using ::drake::geometry::render::RenderEngine::ThrowIfInvalid;
};

namespace py = pybind11;
void apb11_pydrake_RenderEngine_py_register(py::module &m) {
  static bool called = false;
  if (called) {
    return;
  }
  called = true;
  using namespace drake::geometry::render;

  py::class_<RenderEngine> PyRenderEngine(
      m, "RenderEngine",
      R"""(/** The engine for performing rasterization operations on geometry. This 
 includes rgb images and depth images. The coordinate system of 
 %RenderEngine's viewpoint `R` is `X-right`, `Y-down` and `Z-forward` 
 with respect to the rendered images. 
 
 <h3>Output image format</h3> 
 
   - RGB (ImageRgba8U) : the RGB image has four channels in the following 
     order: red, green, blue, and alpha. Each channel is represented by 
     a uint8_t. 
 
   - Depth (ImageDepth32F) : the depth image has a depth channel represented 
     by a float. For a point in space `P`, the value stored in the depth 
     channel holds *the Z-component of the position vector `p_RP`.* 
     Note that this is different from the range data used by laser 
     range finders (like that provided by DepthSensor) in which the depth 
     value represents the distance from the sensor origin to the object's 
     surface. 
 
   - Label (ImageLabel16I) : the label image has single channel represented 
     by an int16_t. The value stored in the channel holds a RenderLabel value 
     which corresponds to an object class in the scene or an "empty" pixel (see 
     RenderLabel for more details). 
 
 @anchor render_engine_default_label 
 <h3>RenderLabels, registering geometry, and derived classes</h3> 
 
 By convention, when registering a geometry, the provided properties should 
 contain no more than one RenderLabel instance, and that should be the 
 `(label, id)` property. %RenderEngine provides the notion of a 
 _default render label_ that will be applied where no `(label, id)` RenderLabel 
 property is found.  This default value can be one of two values: 
 RenderLabel::kDontCare or RenderLabel::kUnspecified. The choice of default 
 RenderLabel can be made at construction and it affects registration behavior 
 when the `(label, id)` property is absent: 
 
   - RenderLabel::kUnspecified: throws an exception. 
   - RenderLabel::kDontCare: the geometry will be included in label images as 
     the generic, non-distinguishing label. 
 
 Choosing RenderLabel::kUnspecified is best in a system that wants explicit 
 feedback and strict enforcement on a policy of strict label enforcement -- 
 everything should receive a meaningful label. The choice of 
 RenderLabel::kDontCare is best for a less strict system in which only some 
 subset of geometry need be explicitly specified. 
 
 Derived classes configure their _de facto_ default RenderLabel value, or 
 a user-configured default value, at construction, subject to the requirements 
 outlined above. 
 
 Derived classes should not access the `(label, id)` property directly. 
 %RenderEngine provides a method to safely extract a RenderLabel value from 
 the PerceptionProperties, taking into account the configured default value and 
 the documented @ref reserved_render_label "RenderLabel semantics"; see 
 GetRenderLabelOrThrow().  */)""");

  PyRenderEngine
      .def(
          "Clone",
          static_cast<::std::unique_ptr<
              drake::geometry::render::RenderEngine,
              std::default_delete<drake::geometry::render::RenderEngine>> (
              RenderEngine::*)() const>(&RenderEngine::Clone),
          R"""(/** Clones the render engine -- making the %RenderEngine compatible with 
 copyable_unique_ptr.  */)""")
      .def_static(
          "DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE",
          static_cast<void (*)()>(
              &RenderEngine_publicist::DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE))
      .def("DoClone",
           static_cast<::std::unique_ptr<
               drake::geometry::render::RenderEngine,
               std::default_delete<drake::geometry::render::RenderEngine>> (
               RenderEngine::*)() const>(&RenderEngine_publicist::DoClone),
           R"""(/** The NVI-function for cloning this render engine.  */)""")
      .def(
          "DoRegisterVisual",
          static_cast<bool (RenderEngine::*)(
              ::drake::geometry::GeometryId, ::drake::geometry::Shape const &,
              ::drake::geometry::PerceptionProperties const &,
              ::drake::math::RigidTransformd const &)>(
              &RenderEngine_publicist::DoRegisterVisual),
          py::arg("id"), py::arg("shape"), py::arg("properties"),
          py::arg("X_WG"),
          R"""(/** The NVI-function for sub-classes to implement actual geometry 
 registration. If the derived class chooses not to register this particular 
 shape, it should return false. 
 
 A derived render engine can choose not to register geometry because, e.g., it 
 doesn't have default properties. This is the primary mechanism which enables 
 different renderers to use different geometries for the same frame. 
 For example, a low-fidelity renderer may use simple geometry whereas a 
 high-fidelity renderer would require a very detailed geometry. Both 
 geometries would have PerceptionProperties, but, based on the provided 
 property groups and values, one would be accepted and registered with one 
 render engine implementation and the other geometry with another render 
 engine. 
 
 In accessing the RenderLabel property in `properties` derived class should 
 _exclusively_ use GetRenderLabelOrThrow().  */)""")
      .def(
          "DoRemoveGeometry",
          static_cast<bool (RenderEngine::*)(::drake::geometry::GeometryId)>(
              &RenderEngine_publicist::DoRemoveGeometry),
          py::arg("id"),
          R"""(/** The NVI-function for removing the geometry with the given `id`. 
 @param id  The id of the geometry to remove. 
 @return  True if the geometry was registered with this %RenderEngine and 
          removed, false if it wasn't registered in the first place.  */)""")
      .def(
          "DoRenderColorImage",
          static_cast<void (RenderEngine::*)(
              ColorRenderCamera const &,
              ::drake::systems::sensors::ImageRgba8U *) const>(
              &RenderEngine_publicist::DoRenderColorImage),
          py::arg("camera"), py::arg("color_image_out"),
          R"""(/** The NVI-function for rendering color with a fully-specified camera. 
 When RenderColorImage calls this, it has already confirmed that 
 `color_image_out` is not `nullptr` and its size is consistent with the 
 camera intrinsics. 
 
 During the deprecation period, the default implementation strips out 
 intrinsics unsupported by the simple render API, prints a warning, and 
 attempts to delegate to the simple-camera RenderColorImage. After 
 the deprecation period, it will throw a "not implemented"-style exception. 
 */)""")
      .def(
          "DoRenderDepthImage",
          static_cast<void (RenderEngine::*)(
              DepthRenderCamera const &,
              ::drake::systems::sensors::ImageDepth32F *) const>(
              &RenderEngine_publicist::DoRenderDepthImage),
          py::arg("camera"), py::arg("depth_image_out"),
          R"""(/** The NVI-function for rendering depth with a fully-specified camera. 
 When RenderDepthImage calls this, it has already confirmed that 
 `depth_image_out` is not `nullptr` and its size is consistent with the 
 camera intrinsics. 
 
 During the deprecation period, the default implementation strips out 
 intrinsics unsupported by the simple render API, prints a warning, and 
 attempts to delegate to the simple-camera RenderDepthImage. After 
 the deprecation period, it will throw a "not implemented"-style exception. 
 */)""")
      .def(
          "DoRenderLabelImage",
          static_cast<void (RenderEngine::*)(
              ColorRenderCamera const &,
              ::drake::systems::sensors::ImageLabel16I *) const>(
              &RenderEngine_publicist::DoRenderLabelImage),
          py::arg("camera"), py::arg("label_image_out"),
          R"""(/** The NVI-function for rendering label with a fully-specified camera. 
 When RenderLabelImage calls this, it has already confirmed that 
 `label_image_out` is not `nullptr` and its size is consistent with the 
 camera intrinsics. 
 
 During the deprecation period, the default implementation strips out 
 intrinsics unsupported by the simple render API, prints a warning, and 
 attempts to delegate to the simple-camera RenderLabelImage. After 
 the deprecation period, it will throw a "not implemented"-style exception. 
 */)""")
      .def(
          "DoUpdateVisualPose",
          static_cast<void (RenderEngine::*)(
              ::drake::geometry::GeometryId,
              ::drake::math::RigidTransformd const &)>(
              &RenderEngine_publicist::DoUpdateVisualPose),
          py::arg("id"), py::arg("X_WG"),
          R"""(/** The NVI-function for updating the pose of a render geometry (identified 
 by `id`) to the given pose X_WG. 
 
 @param id       The id of the render geometry whose pose is being set. 
 @param X_WG     The pose of the render geometry in the world frame.  */)""")
      .def_static(
          "GetColorDFromLabel",
          static_cast<::drake::systems::sensors::ColorD (*)(
              RenderLabel const &)>(
              &RenderEngine_publicist::GetColorDFromLabel),
          py::arg("label"),
          R"""(/** Transforms `this` render label into a double-valued RGB color.  */)""")
      .def_static(
          "GetColorIFromLabel",
          static_cast<::drake::systems::sensors::ColorI (*)(
              RenderLabel const &)>(
              &RenderEngine_publicist::GetColorIFromLabel),
          py::arg("label"),
          R"""(/** Transforms `this` render label into a byte-valued RGB color.  */)""")
      .def(
          "GetRenderLabelOrThrow",
          static_cast<RenderLabel (RenderEngine::*)(
              ::drake::geometry::PerceptionProperties const &) const>(
              &RenderEngine_publicist::GetRenderLabelOrThrow),
          py::arg("properties"),
          R"""(/** Extracts the `(label, id)` RenderLabel property from the given 
 `properties` and validates it (or the configured default if no such 
 property is defined). 
 @throws std::logic_error If the tested render label value is deemed invalid. 
 */)""")
      .def_static(
          "LabelFromColor",
          static_cast<RenderLabel (*)(
              ::drake::systems::sensors::ColorI const &)>(
              &RenderEngine_publicist::LabelFromColor),
          py::arg("color"),
          R"""(/** Transforms the given byte-valued RGB color value into its corresponding 
 RenderLabel.  */)""")
      .def(
          "RegisterVisual",
          static_cast<bool (RenderEngine::*)(
              ::drake::geometry::GeometryId,
              ::drake::geometry::Shape const &,
              ::drake::geometry::PerceptionProperties const &,
              ::drake::math::RigidTransformd const &, bool)>(
              &RenderEngine::RegisterVisual),
          py::arg("id"), py::arg("shape"), py::arg("properties"),
          py::arg("X_WG"), py::arg("needs_updates") = bool(true),
          R"""(/** Requests registration of the given shape with this render engine. The 
 geometry is uniquely identified by the given `id`. The renderer is allowed to 
 examine the given `properties` and choose to _not_ register the geometry. 
 
 Typically, derived classes will attempt to validate the RenderLabel value 
 stored in the `(label, id)` property (or its configured default value if 
 no such property exists). In that case, attempting to assign 
 RenderLabel::kEmpty or RenderLabel::kUnspecified will cause an exception to 
 be thrown (as @ref reserved_render_label "documented"). 
 
 @param id             The geometry id of the shape to register. 
 @param shape          The shape specification to add to the render engine. 
 @param properties     The perception properties provided for this geometry. 
 @param X_WG           The pose of the geometry relative to the world frame W. 
 @param needs_updates  If true, the geometry's pose will be updated via 
                       UpdatePoses(). 
 @returns True if the %RenderEngine implementation accepted the shape for 
          registration. 
 @throws std::runtime_error if the shape is an unsupported type, the 
                            shape's RenderLabel value is 
                            RenderLabel::kUnspecified or RenderLabel::kEmpty, 
                            or a geometry has already been registered with the 
                            given `id`. 
*/)""")
      .def(
          "RemoveGeometry",
          static_cast<bool (RenderEngine::*)(::drake::geometry::GeometryId)>(
              &RenderEngine::RemoveGeometry),
          py::arg("id"),
          R"""(/** Removes the geometry indicated by the given `id` from the engine. 
 @param id    The id of the geometry to remove. 
 @returns True if the geometry was removed (false implies that this id wasn't 
          registered with this engine).  */)""")
      .def("RenderColorImage",
           static_cast<void (RenderEngine::*)(
               CameraProperties const &, bool,
               ::drake::systems::sensors::ImageRgba8U *) const>(
               &RenderEngine::RenderColorImage),
           py::arg("camera"), py::arg("show_window"),
           py::arg("color_image_out"))
      .def(
          "RenderColorImage",
          static_cast<void (RenderEngine::*)(
              ColorRenderCamera const &,
              ::drake::systems::sensors::ImageRgba8U *) const>(
              &RenderEngine::RenderColorImage),
          py::arg("camera"), py::arg("color_image_out"),
          R"""(/** Renders the registered geometry into the given color (rgb) image based on 
 a _fully_ specified camera. 
 
 @param camera                The _render engine_ camera properties. 
 @param[out] color_image_out  The rendered color image. 
 @throws std::logic_error if `color_image_out` is `nullptr` or the size of the 
                          given input image doesn't match the size declared in 
                          `camera`.  */)""")
      .def("RenderDepthImage",
           static_cast<void (RenderEngine::*)(
               DepthCameraProperties const &,
               ::drake::systems::sensors::ImageDepth32F *) const>(
               &RenderEngine::RenderDepthImage),
           py::arg("camera"), py::arg("depth_image_out"))
      .def(
          "RenderDepthImage",
          static_cast<void (RenderEngine::*)(
              DepthRenderCamera const &,
              ::drake::systems::sensors::ImageDepth32F *) const>(
              &RenderEngine::RenderDepthImage),
          py::arg("camera"), py::arg("depth_image_out"),
          R"""(/** Renders the registered geometry into the given depth image based on 
 a _fully_ specified camera. In contrast to the other rendering operations, 
 depth images don't have an option to display the window; generally, basic 
 depth images are not readily communicative to humans. 
 
 @param camera                The _render engine_ camera properties. 
 @param[out] depth_image_out  The rendered depth image. 
 @throws std::logic_error if `depth_image_out` is `nullptr` or the size of the 
                          given input image doesn't match the size declared in 
                          `camera`.  */)""")
      .def("RenderLabelImage",
           static_cast<void (RenderEngine::*)(
               CameraProperties const &, bool,
               ::drake::systems::sensors::ImageLabel16I *) const>(
               &RenderEngine::RenderLabelImage),
           py::arg("camera"), py::arg("show_window"),
           py::arg("label_image_out"))
      .def(
          "RenderLabelImage",
          static_cast<void (RenderEngine::*)(
              ColorRenderCamera const &,
              ::drake::systems::sensors::ImageLabel16I *) const>(
              &RenderEngine::RenderLabelImage),
          py::arg("camera"), py::arg("label_image_out"),
          R"""(/** Renders the registered geometry into the given label image based on 
 a _fully_ specified camera. 
 
 @note This uses the ColorRenderCamera as label images are typically rendered 
 to be exactly registered with a corresponding color image. 
 
 @param camera                The _render engine_ camera properties. 
 @param[out] label_image_out  The rendered label image. 
 @throws std::logic_error if `label_image_out` is `nullptr` or the size of the 
                          given input image doesn't match the size declared in 
                          `camera`.  */)""")
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
           py::arg("X_WR"),
           R"""(/** Updates the renderer's viewpoint with given pose X_WR. 
 
 @param X_WR  The pose of renderer's viewpoint in the world coordinate 
              system.  */)""")
      .def(
          "default_render_label",
          static_cast<RenderLabel (RenderEngine::*)() const>(
              &RenderEngine::default_render_label),
          R"""(/** Reports the render label value this render engine has been configured to 
 use.  */)""")
      .def(
          "has_geometry",
          static_cast<bool (RenderEngine::*)(::drake::geometry::GeometryId)
                          const>(&RenderEngine::has_geometry),
          py::arg("id"),
          R"""(/** Reports true if a geometry with the given `id` has been registered with 
 `this` engine.  */)""")

      ;
}
