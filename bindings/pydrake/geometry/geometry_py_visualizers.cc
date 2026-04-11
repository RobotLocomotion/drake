/* @file This contains the bindings for the various visualizer System types
 found in drake::geometry. They can be found in the pydrake.geometry module. */

#include <memory>
#include <string>
#include <vector>

#include "drake/bindings/generated_docstrings/geometry.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/ref_cycle_pybind.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/type_safe_index_pybind.h"
#include "drake/bindings/pydrake/geometry/geometry_py.h"
#include "drake/bindings/pydrake/systems/builder_life_support_pybind.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_animation.h"
#include "drake/geometry/meshcat_point_cloud_visualizer.h"
#include "drake/geometry/meshcat_visualizer.h"

namespace drake {
namespace pydrake {
namespace {

using math::RigidTransformd;
using systems::Context;
using systems::LeafSystem;

// NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
using namespace drake::geometry;
constexpr auto& doc = pydrake_doc_geometry.drake.geometry;

// TODO(jwnimmer-tri) Reformat this entire file to remove the unnecessary
// indentation.

template <typename T>
void DefineDrakeVisualizer(py::module m, T) {
  py::tuple param = GetPyParam<T>();
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
            // `return` and `builder` join ref cycle.
            internal::ref_cycle<0, 1>(),
            // Using builder_life_support_stash makes the builder temporarily
            // immortal (uncollectible self cycle). This will be resolved by
            // the Build() step. See BuilderLifeSupport for rationale.
            internal::builder_life_support_stash<T, 1>(),
            // Keep alive, reference: `return` keeps `lcm` alive.
            py::keep_alive<0, 3>(), py_rvp::reference,
            cls_doc.AddToBuilder.doc_4args_builder_scene_graph_lcm_params)
        .def_static("AddToBuilder",
            py::overload_cast<systems::DiagramBuilder<T>*,
                const systems::OutputPort<T>&, lcm::DrakeLcmInterface*,
                DrakeVisualizerParams>(&DrakeVisualizer<T>::AddToBuilder),
            py::arg("builder"), py::arg("query_object_port"),
            py::arg("lcm") = nullptr,
            py::arg("params") = DrakeVisualizerParams{},
            // `return` and `builder` join ref cycle.
            internal::ref_cycle<0, 1>(),
            // Using builder_life_support_stash makes the builder temporarily
            // immortal (uncollectible self cycle). This will be resolved by
            // the Build() step. See BuilderLifeSupport for rationale.
            internal::builder_life_support_stash<T, 1>(),
            // Keep alive, reference: `return` keeps `lcm` alive.
            py::keep_alive<0, 3>(), py_rvp::reference,
            cls_doc.AddToBuilder.doc_4args_builder_query_object_port_lcm_params)
        .def_static("DispatchLoadMessage",
            &DrakeVisualizer<T>::DispatchLoadMessage, py::arg("scene_graph"),
            py::arg("lcm"), py::arg("params") = DrakeVisualizerParams{},
            cls_doc.DispatchLoadMessage.doc);
  }
}

template <typename T>
void DefineMeshcatPointCloudVisualizer(py::module m, T) {
  py::tuple param = GetPyParam<T>();
  {
    using Class = MeshcatPointCloudVisualizer<T>;
    constexpr auto& cls_doc = doc.MeshcatPointCloudVisualizer;
    auto cls = DefineTemplateClassWithDefault<Class, LeafSystem<T>>(
        m, "MeshcatPointCloudVisualizer", param, cls_doc.doc);
    cls  // BR
        .def(py::init<std::shared_ptr<Meshcat>, std::string, double>(),
            py::arg("meshcat"), py::arg("path"),
            py::arg("publish_period") = 1 / 32.0,
            // `meshcat` is a shared_ptr, so does not need a keep_alive.
            cls_doc.ctor.doc)
        .def("set_point_size", &Class::set_point_size,
            cls_doc.set_point_size.doc)
        .def("set_default_rgba", &Class::set_default_rgba,
            cls_doc.set_default_rgba.doc)
        .def("Delete", &Class::Delete, cls_doc.Delete.doc)
        .def("cloud_input_port", &Class::cloud_input_port,
            py_rvp::reference_internal, cls_doc.cloud_input_port.doc)
        .def("pose_input_port", &Class::pose_input_port,
            py_rvp::reference_internal, cls_doc.pose_input_port.doc);
  }
}

template <typename T>
void DefineMeshcatVisualizer(py::module m, T) {
  py::tuple param = GetPyParam<T>();
  {
    using Class = MeshcatVisualizer<T>;
    constexpr auto& cls_doc = doc.MeshcatVisualizer;
    auto cls = DefineTemplateClassWithDefault<Class, LeafSystem<T>>(
        m, "MeshcatVisualizer", param, cls_doc.doc);
    cls  // BR
        .def(py::init<std::shared_ptr<Meshcat>, MeshcatVisualizerParams>(),
            py::arg("meshcat"), py::arg("params") = MeshcatVisualizerParams{},
            // `meshcat` is a shared_ptr, so does not need a keep_alive.
            cls_doc.ctor.doc)
        .def("Delete", &Class::Delete, cls_doc.Delete.doc)
        .def("StartRecording", &Class::StartRecording,
            py::arg("set_transforms_while_recording") = true,
            py_rvp::reference_internal, cls_doc.StartRecording.doc)
        .def("StopRecording", &Class::StopRecording, cls_doc.StopRecording.doc)
        .def("PublishRecording", &Class::PublishRecording,
            cls_doc.PublishRecording.doc)
        .def("DeleteRecording", &Class::DeleteRecording,
            cls_doc.DeleteRecording.doc)
        .def("get_mutable_recording", &Class::get_mutable_recording,
            py_rvp::reference_internal, cls_doc.get_mutable_recording.doc)
        .def("query_object_input_port", &Class::query_object_input_port,
            py_rvp::reference_internal, cls_doc.query_object_input_port.doc)
        .def_static("AddToBuilder",
            py::overload_cast<systems::DiagramBuilder<T>*, const SceneGraph<T>&,
                std::shared_ptr<Meshcat>, MeshcatVisualizerParams>(
                &MeshcatVisualizer<T>::AddToBuilder),
            py::arg("builder"), py::arg("scene_graph"), py::arg("meshcat"),
            py::arg("params") = MeshcatVisualizerParams{},
            // Keep alive, ownership: `return` keeps `builder` alive.
            py::keep_alive<0, 1>(),
            // `meshcat` is a shared_ptr, so does not need a keep_alive.
            py_rvp::reference,
            cls_doc.AddToBuilder.doc_4args_builder_scene_graph_meshcat_params)
        .def_static("AddToBuilder",
            py::overload_cast<systems::DiagramBuilder<T>*,
                const systems::OutputPort<T>&, std::shared_ptr<Meshcat>,
                MeshcatVisualizerParams>(&MeshcatVisualizer<T>::AddToBuilder),
            py::arg("builder"), py::arg("query_object_port"),
            py::arg("meshcat"), py::arg("params") = MeshcatVisualizerParams{},
            // Keep alive, ownership: `return` keeps `builder` alive.
            py::keep_alive<0, 1>(),
            // `meshcat` is a shared_ptr, so does not need a keep_alive.
            py_rvp::reference,
            cls_doc.AddToBuilder
                .doc_4args_builder_query_object_port_meshcat_params);
  }
}

void DefineDrakeVisualizerParams(py::module m) {
  {
    using Class = DrakeVisualizerParams;
    constexpr auto& cls_doc = doc.DrakeVisualizerParams;
    py::class_<Class> cls(
        m, "DrakeVisualizerParams", py::dynamic_attr(), cls_doc.doc);
    cls  // BR
        .def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }
}

void DefineMeshcatParams(py::module m) {
  {
    using Class = MeshcatParams;
    constexpr auto& cls_doc = doc.MeshcatParams;
    py::class_<Class, std::shared_ptr<Class>> cls(
        m, "MeshcatParams", py::dynamic_attr(), cls_doc.doc);
    // MeshcatParams::PropertyTuple
    {
      using Nested = MeshcatParams::PropertyTuple;
      constexpr auto& nested_doc = doc.MeshcatParams.PropertyTuple;
      py::class_<Nested> nested(cls, "PropertyTuple", nested_doc.doc);
      nested.def(ParamInit<Nested>());
      DefAttributesUsingSerialize(&nested, nested_doc);
      DefReprUsingSerialize(&nested);
      DefCopyAndDeepCopy(&nested);
    }
    cls.def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }
}

void DefineMeshcat(py::module m) {
  {
    using Class = Meshcat;
    constexpr auto& cls_doc = doc.Meshcat;
    py::class_<Class, std::shared_ptr<Class>> meshcat(
        m, "Meshcat", cls_doc.doc);

    // Meshcat::SideOfFaceToRender enumeration
    constexpr auto& side_doc = doc.Meshcat.SideOfFaceToRender;
    py::enum_<Meshcat::SideOfFaceToRender>(
        meshcat, "SideOfFaceToRender", side_doc.doc)
        .value("kFrontSide", Meshcat::SideOfFaceToRender::kFrontSide,
            side_doc.kFrontSide.doc)
        .value("kBackSide", Meshcat::SideOfFaceToRender::kBackSide,
            side_doc.kBackSide.doc)
        .value("kDoubleSide", Meshcat::SideOfFaceToRender::kDoubleSide,
            side_doc.kDoubleSide.doc);

    const auto& perspective_camera_doc = doc.Meshcat.PerspectiveCamera;
    py::class_<Meshcat::PerspectiveCamera> perspective_camera_cls(
        meshcat, "PerspectiveCamera", perspective_camera_doc.doc);
    perspective_camera_cls  // BR
        .def(ParamInit<Meshcat::PerspectiveCamera>());
    DefAttributesUsingSerialize(
        &perspective_camera_cls, perspective_camera_doc);
    DefReprUsingSerialize(&perspective_camera_cls);
    DefCopyAndDeepCopy(&perspective_camera_cls);

    const auto& orthographic_camera_doc = doc.Meshcat.OrthographicCamera;
    py::class_<Meshcat::OrthographicCamera> orthographic_camera_cls(
        meshcat, "OrthographicCamera", orthographic_camera_doc.doc);
    orthographic_camera_cls  // BR
        .def(ParamInit<Meshcat::OrthographicCamera>());
    DefAttributesUsingSerialize(
        &orthographic_camera_cls, orthographic_camera_doc);
    DefReprUsingSerialize(&orthographic_camera_cls);
    DefCopyAndDeepCopy(&orthographic_camera_cls);

    const auto& gamepad_doc = doc.Meshcat.Gamepad;
    py::class_<Meshcat::Gamepad> gamepad_cls(
        meshcat, "Gamepad", gamepad_doc.doc);
    gamepad_cls  // BR
        .def(ParamInit<Meshcat::Gamepad>());
    DefAttributesUsingSerialize(&gamepad_cls, gamepad_doc);
    DefReprUsingSerialize(&gamepad_cls);
    DefCopyAndDeepCopy(&gamepad_cls);

    meshcat  // BR
        .def(py::init<std::optional<int>>(), py::arg("port") = std::nullopt,
            cls_doc.ctor.doc_1args_port)
        .def(py::init<const MeshcatParams&>(), py::arg("params"),
            cls_doc.ctor.doc_1args_params)
        .def("web_url", &Class::web_url, cls_doc.web_url.doc)
        .def("port", &Class::port, cls_doc.port.doc)
        .def("ws_url", &Class::ws_url, cls_doc.ws_url.doc)
        .def("GetNumActiveConnections", &Class::GetNumActiveConnections,
            cls_doc.GetNumActiveConnections.doc)
        .def("Flush", &Class::Flush,
            // Internally this function both blocks on a worker thread and
            // sleeps; for both reasons, we must release the GIL.
            py::call_guard<py::gil_scoped_release>(), cls_doc.Flush.doc)
        .def("SetObject",
            py::overload_cast<std::string_view, const Shape&, const Rgba&>(
                &Class::SetObject),
            py::arg("path"), py::arg("shape"),
            py::arg("rgba") = Rgba(.9, .9, .9, 1.), cls_doc.SetObject.doc_shape)
        .def("SetObject",
            py::overload_cast<std::string_view, const perception::PointCloud&,
                double, const Rgba&>(&Class::SetObject),
            py::arg("path"), py::arg("cloud"), py::arg("point_size") = 0.001,
            py::arg("rgba") = Rgba(.9, .9, .9, 1.), cls_doc.SetObject.doc_cloud)
        .def("SetObject",
            py::overload_cast<std::string_view,
                const TriangleSurfaceMesh<double>&, const Rgba&, bool, double,
                Meshcat::SideOfFaceToRender>(&Class::SetObject),
            py::arg("path"), py::arg("mesh"),
            py::arg("rgba") = Rgba(0.1, 0.1, 0.1, 1.0),
            py::arg("wireframe") = false, py::arg("wireframe_line_width") = 1.0,
            py::arg("side") = Meshcat::SideOfFaceToRender::kDoubleSide,
            cls_doc.SetObject.doc_triangle_surface_mesh)
        .def("SetLine", &Class::SetLine, py::arg("path"), py::arg("vertices"),
            py::arg("line_width") = 1.0,
            py::arg("rgba") = Rgba(0.1, 0.1, 0.1, 1.0), cls_doc.SetLine.doc)
        .def("SetLineSegments", &Class::SetLineSegments, py::arg("path"),
            py::arg("start"), py::arg("end"), py::arg("line_width") = 1.0,
            py::arg("rgba") = Rgba(0.1, 0.1, 0.1, 1.0),
            cls_doc.SetLineSegments.doc)
        .def("SetTriangleMesh", &Class::SetTriangleMesh, py::arg("path"),
            py::arg("vertices"), py::arg("faces"),
            py::arg("rgba") = Rgba(0.1, 0.1, 0.1, 1.0),
            py::arg("wireframe") = false, py::arg("wireframe_line_width") = 1.0,
            py::arg("side") = Meshcat::SideOfFaceToRender::kDoubleSide,
            cls_doc.SetTriangleMesh.doc)
        .def("SetTriangleColorMesh", &Class::SetTriangleColorMesh,
            py::arg("path"), py::arg("vertices"), py::arg("faces"),
            py::arg("colors"), py::arg("wireframe") = false,
            py::arg("wireframe_line_width") = 1.0,
            py::arg("side") = Meshcat::SideOfFaceToRender::kDoubleSide,
            cls_doc.SetTriangleColorMesh.doc)
        .def("PlotSurface", &Class::PlotSurface, py::arg("path"), py::arg("X"),
            py::arg("Y"), py::arg("Z"),
            py::arg("rgba") = Rgba(0.1, 0.1, 0.9, 1.0),
            py::arg("wireframe") = false, py::arg("wireframe_line_width") = 1.0,
            cls_doc.PlotSurface.doc)
        .def("SetCamera",
            py::overload_cast<Meshcat::PerspectiveCamera, std::string>(
                &Class::SetCamera),
            py::arg("camera"), py::arg("path") = "/Cameras/default/rotated",
            cls_doc.SetCamera.doc_perspective)
        .def("SetCamera",
            py::overload_cast<Meshcat::OrthographicCamera, std::string>(
                &Class::SetCamera),
            py::arg("camera"), py::arg("path") = "/Cameras/default/rotated",
            cls_doc.SetCamera.doc_orthographic)
        .def("Set2dRenderMode", &Class::Set2dRenderMode,
            py::arg("X_WC") = RigidTransformd{Eigen::Vector3d{0, -1, 0}},
            py::arg("xmin") = -1.0, py::arg("xmax") = 1.0,
            py::arg("ymin") = -1.0, py::arg("ymax") = 1.0,
            cls_doc.Set2dRenderMode.doc)
        .def("ResetRenderMode", &Class::ResetRenderMode,
            cls_doc.ResetRenderMode.doc)
        .def("SetCameraTarget", &Class::SetCameraTarget,
            py::arg("target_in_world"), cls_doc.SetCameraTarget.doc)
        .def("SetCameraPose", &Class::SetCameraPose, py::arg("camera_in_world"),
            py::arg("target_in_world"), cls_doc.SetCameraPose.doc)
        .def("GetTrackedCameraPose", &Class::GetTrackedCameraPose,
            cls_doc.GetTrackedCameraPose.doc)
        .def("SetTransform",
            py::overload_cast<std::string_view, const math::RigidTransformd&,
                std::optional<double>>(&Class::SetTransform),
            py::arg("path"), py::arg("X_ParentPath"),
            py::arg("time_in_recording") = std::nullopt,
            cls_doc.SetTransform.doc_RigidTransform)
        .def("SetTransform",
            py::overload_cast<std::string_view,
                const Eigen::Ref<const Eigen::Matrix4d>&>(&Class::SetTransform),
            py::arg("path"), py::arg("matrix"), cls_doc.SetTransform.doc_matrix)
        .def("Delete", &Class::Delete, py::arg("path") = "", cls_doc.Delete.doc)
        .def("SetSimulationTime", &Class::SetSimulationTime,
            py::arg("sim_time"), cls_doc.SetSimulationTime.doc)
        .def("SetRealtimeRate", &Class::SetRealtimeRate, py::arg("rate"),
            cls_doc.SetRealtimeRate.doc)
        .def("GetRealtimeRate", &Class::GetRealtimeRate,
            cls_doc.GetRealtimeRate.doc)
        .def("SetProperty",
            py::overload_cast<std::string_view, std::string, bool,
                std::optional<double>>(&Class::SetProperty),
            py::arg("path"), py::arg("property"), py::arg("value").noconvert(),
            py::arg("time_in_recording") = std::nullopt,
            cls_doc.SetProperty.doc_bool)
        .def("SetProperty",
            py::overload_cast<std::string_view, std::string, double,
                std::optional<double>>(&Class::SetProperty),
            py::arg("path"), py::arg("property"), py::arg("value"),
            py::arg("time_in_recording") = std::nullopt,
            cls_doc.SetProperty.doc_double)
        .def("SetProperty",
            py::overload_cast<std::string_view, std::string,
                const std::vector<double>&, std::optional<double>>(
                &Class::SetProperty),
            py::arg("path"), py::arg("property"), py::arg("value"),
            py::arg("time_in_recording") = std::nullopt,
            cls_doc.SetProperty.doc_vector_double)
        .def("SetEnvironmentMap", &Class::SetEnvironmentMap,
            py::arg("image_path"), cls_doc.SetEnvironmentMap.doc)
        .def("SetAnimation", &Class::SetAnimation, py::arg("animation"),
            +cls_doc.SetAnimation.doc)
        .def("AddButton", &Class::AddButton, py::arg("name"),
            py::arg("keycode") = "", cls_doc.AddButton.doc)
        .def("GetButtonClicks", &Class::GetButtonClicks, py::arg("name"),
            cls_doc.GetButtonClicks.doc)
        .def("GetButtonNames", &Class::GetButtonNames,
            cls_doc.GetButtonNames.doc)
        .def("DeleteButton", &Class::DeleteButton, py::arg("name"),
            py::arg("strict") = true, cls_doc.DeleteButton.doc)
        .def("AddSlider", &Class::AddSlider, py::arg("name"), py::arg("min"),
            py::arg("max"), py::arg("step"), py::arg("value"),
            py::arg("decrement_keycode") = "",
            py::arg("increment_keycode") = "", cls_doc.AddSlider.doc)
        .def("SetSliderValue", &Class::SetSliderValue, py::arg("name"),
            py::arg("value"), cls_doc.SetSliderValue.doc)
        .def("GetSliderValue", &Class::GetSliderValue, py::arg("name"),
            cls_doc.GetSliderValue.doc)
        .def("GetSliderNames", &Class::GetSliderNames,
            cls_doc.GetSliderNames.doc)
        .def("DeleteSlider", &Class::DeleteSlider, py::arg("name"),
            py::arg("strict") = true, cls_doc.DeleteSlider.doc)
        .def("DeleteAddedControls", &Class::DeleteAddedControls,
            cls_doc.DeleteAddedControls.doc)
        .def("GetGamepad", &Class::GetGamepad, cls_doc.GetGamepad.doc)
        .def("StaticHtml", &Class::StaticHtml,
            // This function costs a non-trivial amount of CPU time and blocks
            // on a worker thread; for both reasons, we must release the GIL.
            py::call_guard<py::gil_scoped_release>(), cls_doc.StaticHtml.doc)
        .def(
            "StaticZip",
            [](const Class& self) {
              // This function costs a non-trivial amount of CPU time and blocks
              // on a worker thread; for both reasons, we must release the GIL.
              // We must then re-acquire it before touching py::bytes.
              std::string result;
              {
                py::gil_scoped_release unlock;
                result = self.StaticZip();
              }
              return py::bytes(result);
            },
            cls_doc.StaticZip.doc)
        .def("StartRecording", &Class::StartRecording,
            py::arg("frames_per_second") = 64.0,
            py::arg("set_visualizations_while_recording") = true,
            cls_doc.StartRecording.doc)
        .def("StopRecording", &Class::StopRecording, cls_doc.StopRecording.doc)
        .def("PublishRecording", &Class::PublishRecording,
            cls_doc.PublishRecording.doc)
        .def("DeleteRecording", &Class::DeleteRecording,
            cls_doc.DeleteRecording.doc)
        .def("get_mutable_recording", &Class::get_mutable_recording,
            py_rvp::reference_internal, cls_doc.get_mutable_recording.doc)
        .def("HasPath", &Class::HasPath, py::arg("path"),
            // This function blocks on a worker thread so must release the GIL.
            py::call_guard<py::gil_scoped_release>(), cls_doc.HasPath.doc);

    // This helper wraps a Meshcat::GetPacked{Foo} member function to release
    // the GIL during the call (because the member function blocks to wait for a
    // worker thread) and then copies the result into py::bytes while holding
    // the GIL.
    auto wrap_get_packed_foo =
        []<typename... Args>(std::string (Class::*member_func)(Args...) const) {
          return [member_func](const Class& self, Args... args) {
            std::string result;
            {
              py::gil_scoped_release unlock;
              result = (self.*member_func)(args...);
            }
            return py::bytes(result);
          };
        };  // NOLINT(readability/braces)

    // The remaining methods are intended to primarily for testing. Because they
    // are excluded from C++ Doxygen, we bind them privately here.
    meshcat  // BR
        .def("_GetPackedObject", wrap_get_packed_foo(&Class::GetPackedObject),
            py::arg("path"))
        .def("_GetPackedTransform",
            wrap_get_packed_foo(&Class::GetPackedTransform), py::arg("path"))
        .def("_GetPackedProperty",
            wrap_get_packed_foo(&Class::GetPackedProperty), py::arg("path"),
            py::arg("property"))
        .def(
            "_InjectWebsocketMessage",
            [](Class& self, py::bytes message) {
              std::string_view message_view = message;
              // This call blocks on a worker thread so must release the GIL.
              py::gil_scoped_release unlock;
              self.InjectWebsocketMessage(message_view);
            },
            py::arg("message"));
  }
}

void DefineMeshcatAnimation(py::module m) {
  {
    using Class = MeshcatAnimation;
    constexpr auto& cls_doc = doc.MeshcatAnimation;
    py::class_<Class> cls(m, "MeshcatAnimation", cls_doc.doc);

    // MeshcatAnimation::LoopMode enumeration
    constexpr auto& loop_doc = doc.MeshcatAnimation.LoopMode;
    py::enum_<MeshcatAnimation::LoopMode>(cls, "LoopMode", loop_doc.doc)
        .value("kLoopOnce", MeshcatAnimation::LoopMode::kLoopOnce,
            loop_doc.kLoopOnce.doc)
        .value("kLoopRepeat", MeshcatAnimation::LoopMode::kLoopRepeat,
            loop_doc.kLoopRepeat.doc)
        .value("kLoopPingPong", MeshcatAnimation::LoopMode::kLoopPingPong,
            loop_doc.kLoopPingPong.doc);

    cls  // BR
        .def(py::init<double>(), py::arg("frames_per_second") = 64.0,
            cls_doc.ctor.doc)
        .def("frames_per_second", &Class::frames_per_second,
            cls_doc.frames_per_second.doc)
        .def("frame", &Class::frame, py::arg("time_from_start"),
            cls_doc.frame.doc)
        .def("start_time", &Class::start_time, cls_doc.start_time.doc)
        .def("autoplay", &Class::autoplay, cls_doc.autoplay.doc)
        .def("loop_mode", &Class::loop_mode, cls_doc.loop_mode.doc)
        .def("repetitions", &Class::repetitions, cls_doc.repetitions.doc)
        .def("clamp_when_finished", &Class::clamp_when_finished,
            cls_doc.clamp_when_finished.doc)
        .def("set_start_time", &Class::set_start_time, py::arg("time"),
            cls_doc.set_start_time.doc)
        .def("set_autoplay", &Class::set_autoplay, py::arg("play"),
            cls_doc.set_autoplay.doc)
        .def("set_loop_mode", &Class::set_loop_mode, py::arg("mode"),
            cls_doc.set_loop_mode.doc)
        .def("set_repetitions", &Class::set_repetitions, py::arg("repetitions"),
            cls_doc.set_repetitions.doc)
        .def("set_clamp_when_finished", &Class::set_clamp_when_finished,
            py::arg("clamp"), cls_doc.set_clamp_when_finished.doc)
        .def("SetTransform", &Class::SetTransform, py::arg("frame"),
            py::arg("path"), py::arg("X_ParentPath"), cls_doc.SetTransform.doc)
        .def("SetProperty",
            // Note: overload_cast and overload_cast_explicit did not work here.
            static_cast<void (Class::*)(int, std::string_view, std::string_view,
                bool)>(&Class::SetProperty),
            py::arg("frame"), py::arg("path"), py::arg("property"),
            py::arg("value"), cls_doc.SetProperty.doc_bool)
        .def("SetProperty",
            static_cast<void (Class::*)(int, std::string_view, std::string_view,
                double)>(&Class::SetProperty),
            py::arg("frame"), py::arg("path"), py::arg("property"),
            py::arg("value"), cls_doc.SetProperty.doc_double)
        .def("SetProperty",
            static_cast<void (Class::*)(int, std::string_view, std::string_view,
                const std::vector<double>&)>(&Class::SetProperty),
            py::arg("frame"), py::arg("path"), py::arg("property"),
            py::arg("value"), cls_doc.SetProperty.doc_vector_double);
    // Note: We don't bind get_key_frame and get_javascript_type (at least
    // not yet); they are meant primarily for testing.
  }
}

void DefineMeshcatVisualizerParams(py::module m) {
  {
    using Class = MeshcatVisualizerParams;
    constexpr auto& cls_doc = doc.MeshcatVisualizerParams;
    py::class_<Class> cls(
        m, "MeshcatVisualizerParams", py::dynamic_attr(), cls_doc.doc);
    cls  // BR
        .def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }
}

}  // namespace

void DefineGeometryVisualizers(py::module m) {
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.lcm");

  // This list must remain in topological dependency order.
  DefineMeshcatParams(m);
  DefineDrakeVisualizerParams(m);
  DefineMeshcatVisualizerParams(m);
  DefineMeshcatAnimation(m);
  DefineMeshcat(m);
  type_visit(
      [m](auto dummy) {
        DefineDrakeVisualizer(m, dummy);
        DefineMeshcatPointCloudVisualizer(m, dummy);
        DefineMeshcatVisualizer(m, dummy);
      },
      NonSymbolicScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
