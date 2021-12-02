/* @file This contains the bindings for the various visualizer System types
 found in drake::geometry. They can be found in the pydrake.geometry module. */

#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/type_safe_index_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
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

template <typename T>
void DoScalarDependentDefinitions(py::module m, T) {
  py::tuple param = GetPyParam<T>();
  constexpr auto& doc = pydrake_doc.drake.geometry;

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.systems.lcm");

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

  // MeshcatPointCloudVisualizer
  {
    using Class = MeshcatPointCloudVisualizer<T>;
    constexpr auto& cls_doc = doc.MeshcatPointCloudVisualizer;
    auto cls = DefineTemplateClassWithDefault<Class, LeafSystem<T>>(m,
        "MeshcatPointCloudVisualizerCpp", param,
        (std::string(cls_doc.doc) + R"""(
Note that we are temporarily re-mapping MeshcatPointCloudVisualizer =>
MeshcatPointCloudVisualizerCpp to avoid collisions with the python
MeshcatPointCloudVisualizer.  See #13038.)""")
            .c_str());
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

  // MeshcatVisualizer
  {
    using Class = MeshcatVisualizer<T>;
    constexpr auto& cls_doc = doc.MeshcatVisualizer;
    auto cls = DefineTemplateClassWithDefault<Class, LeafSystem<T>>(m,
        "MeshcatVisualizerCpp", param,
        (std::string(cls_doc.doc) + R"""(
Note that we are temporarily re-mapping MeshcatVisualizer =>
MeshcatVisualizerCpp to avoid collisions with the python
MeshcatVisualizer.  See #13038.)""")
            .c_str());
    cls  // BR
        .def(py::init<std::shared_ptr<Meshcat>, MeshcatVisualizerParams>(),
            py::arg("meshcat"), py::arg("params") = MeshcatVisualizerParams{},
            // `meshcat` is a shared_ptr, so does not need a keep_alive.
            cls_doc.ctor.doc)
        .def("Delete", &Class::Delete, cls_doc.Delete.doc)
        .def("StartRecording", &Class::StartRecording,
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

void DoScalarIndependentDefinitions(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;
  constexpr auto& doc = pydrake_doc.drake.geometry;

  // DrakeVisualizerParams
  {
    using Class = DrakeVisualizerParams;
    constexpr auto& cls_doc = doc.DrakeVisualizerParams;
    py::class_<Class>(
        m, "DrakeVisualizerParams", py::dynamic_attr(), cls_doc.doc)
        .def(ParamInit<Class>())
        .def_readwrite("publish_period", &DrakeVisualizerParams::publish_period,
            cls_doc.publish_period.doc)
        .def_readwrite("role", &DrakeVisualizerParams::role, cls_doc.role.doc)
        .def_readwrite("default_color", &DrakeVisualizerParams::default_color,
            cls_doc.default_color.doc)
        .def_readwrite("show_hydroelastic",
            &DrakeVisualizerParams::show_hydroelastic,
            cls_doc.show_hydroelastic.doc)
        .def("__repr__", [](const Class& self) {
          return py::str(
              "DrakeVisualizerParams("
              "publish_period={}, "
              "role={}, "
              "default_color={}, "
              "show_hydroelastic={})")
              .format(self.publish_period, self.role, self.default_color,
                  self.show_hydroelastic);
        });
  }

  // Meshcat
  {
    using Class = Meshcat;
    constexpr auto& cls_doc = doc.Meshcat;
    py::class_<Class, std::shared_ptr<Class>> cls(m, "Meshcat", cls_doc.doc);
    cls  // BR
        .def(py::init<const std::optional<int>&>(),
            py::arg("port") = std::nullopt, cls_doc.ctor.doc)
        .def("web_url", &Class::web_url, cls_doc.web_url.doc)
        .def("port", &Class::port, cls_doc.port.doc)
        .def("ws_url", &Class::ws_url, cls_doc.ws_url.doc)
        .def("Flush", &Class::Flush, cls_doc.Flush.doc)
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
                const TriangleSurfaceMesh<double>&, const Rgba&, bool, double>(
                &Class::SetObject),
            py::arg("path"), py::arg("mesh"),
            py::arg("rgba") = Rgba(0.1, 0.1, 0.1, 1.0),
            py::arg("wireframe") = false, py::arg("wireframe_line_width") = 1.0,
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
            cls_doc.SetTriangleMesh.doc)
        // TODO(russt): Bind SetCamera.
        .def("Set2dRenderMode", &Class::Set2dRenderMode,
            py::arg("X_WC") = RigidTransformd{Eigen::Vector3d{0, -1, 0}},
            py::arg("xmin") = -1.0, py::arg("xmax") = 1.0,
            py::arg("ymin") = -1.0, py::arg("ymax") = 1.0,
            cls_doc.Set2dRenderMode.doc)
        .def("ResetRenderMode", &Class::ResetRenderMode,
            cls_doc.ResetRenderMode.doc)
        .def("SetTransform",
            py::overload_cast<std::string_view, const math::RigidTransformd&>(
                &Class::SetTransform),
            py::arg("path"), py::arg("X_ParentPath"),
            cls_doc.SetTransform.doc_RigidTransform)
        .def("SetTransform",
            py::overload_cast<std::string_view,
                const Eigen::Ref<const Eigen::Matrix4d>&>(&Class::SetTransform),
            py::arg("path"), py::arg("matrix"), cls_doc.SetTransform.doc_matrix)
        .def("Delete", &Class::Delete, py::arg("path") = "", cls_doc.Delete.doc)
        .def("SetProperty",
            py::overload_cast<std::string_view, std::string, bool>(
                &Class::SetProperty),
            py::arg("path"), py::arg("property"), py::arg("value"),
            cls_doc.SetProperty.doc_bool)
        .def("SetProperty",
            py::overload_cast<std::string_view, std::string, double>(
                &Class::SetProperty),
            py::arg("path"), py::arg("property"), py::arg("value"),
            cls_doc.SetProperty.doc_double)
        .def("SetProperty",
            py::overload_cast<std::string_view, std::string,
                const std::vector<double>&>(&Class::SetProperty),
            py::arg("path"), py::arg("property"), py::arg("value"),
            cls_doc.SetProperty.doc_vector_double)
        .def("SetAnimation", &Class::SetAnimation, py::arg("animation"),
            +cls_doc.SetAnimation.doc)
        .def("AddButton", &Class::AddButton, py::arg("name"),
            cls_doc.AddButton.doc)
        .def("GetButtonClicks", &Class::GetButtonClicks, py::arg("name"),
            cls_doc.GetButtonClicks.doc)
        .def("DeleteButton", &Class::DeleteButton, py::arg("name"),
            cls_doc.DeleteButton.doc)
        .def("AddSlider", &Class::AddSlider, py::arg("name"), py::arg("min"),
            py::arg("max"), py::arg("step"), py::arg("value"),
            cls_doc.AddSlider.doc)
        .def("SetSliderValue", &Class::SetSliderValue, py::arg("name"),
            py::arg("value"), cls_doc.SetSliderValue.doc)
        .def("GetSliderValue", &Class::GetSliderValue, py::arg("name"),
            cls_doc.GetSliderValue.doc)
        .def("DeleteSlider", &Class::DeleteSlider, py::arg("name"),
            cls_doc.DeleteSlider.doc)
        .def("DeleteAddedControls", &Class::DeleteAddedControls,
            cls_doc.DeleteAddedControls.doc)
        .def("StaticHtml", &Class::StaticHtml, cls_doc.StaticHtml.doc);
    // Note: we intentionally do not bind the advanced methods (HasProperty and
    // GetPacked*) which were intended primarily for testing in C++.
  }

  // MeshcatAnimation
  {
    using Class = MeshcatAnimation;
    constexpr auto& cls_doc = doc.MeshcatAnimation;
    py::class_<Class> cls(m, "MeshcatAnimation", cls_doc.doc);
    cls  // BR
        .def(py::init<double>(), py::arg("frames_per_second") = 32.0,
            cls_doc.ctor.doc)
        .def("frames_per_second", &Class::frames_per_second,
            cls_doc.frames_per_second.doc)
        .def("frame", &Class::frame, py::arg("time_from_start"),
            cls_doc.frame.doc)
        .def("autoplay", &Class::autoplay, cls_doc.autoplay.doc)
        .def("loop_mode", &Class::loop_mode, cls_doc.loop_mode.doc)
        .def("repetitions", &Class::repetitions, cls_doc.repetitions.doc)
        .def("clamp_when_finished", &Class::clamp_when_finished,
            cls_doc.clamp_when_finished.doc)
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
            static_cast<void (Class::*)(int, const std::string&,
                const std::string&, bool)>(&Class::SetProperty),
            py::arg("frame"), py::arg("path"), py::arg("property"),
            py::arg("value"), cls_doc.SetProperty.doc_bool)
        .def("SetProperty",
            static_cast<void (Class::*)(int, const std::string&,
                const std::string&, double)>(&Class::SetProperty),
            py::arg("frame"), py::arg("path"), py::arg("property"),
            py::arg("value"), cls_doc.SetProperty.doc_double)
        .def("SetProperty",
            static_cast<void (Class::*)(int, const std::string&,
                const std::string&, const std::vector<double>&)>(
                &Class::SetProperty),
            py::arg("frame"), py::arg("path"), py::arg("property"),
            py::arg("value"), cls_doc.SetProperty.doc_vector_double);
    // Note: We don't bind get_key_frame and get_javascript_type (at least
    // not yet); they are meant primarily for testing.

    // MeshcatAnimation::LoopMode enumeration
    constexpr auto& loop_doc = doc.MeshcatAnimation.LoopMode;
    py::enum_<MeshcatAnimation::LoopMode>(cls, "LoopMode", loop_doc.doc)
        .value("kLoopOnce", MeshcatAnimation::LoopMode::kLoopOnce,
            loop_doc.kLoopOnce.doc)
        .value("kLoopRepeat", MeshcatAnimation::LoopMode::kLoopRepeat,
            loop_doc.kLoopRepeat.doc)
        .value("kLoopPingPong", MeshcatAnimation::LoopMode::kLoopPingPong,
            loop_doc.kLoopPingPong.doc);
  }

  // MeshcatVisualizerParams
  {
    using Class = MeshcatVisualizerParams;
    constexpr auto& cls_doc = doc.MeshcatVisualizerParams;
    py::class_<Class>(
        m, "MeshcatVisualizerParams", py::dynamic_attr(), cls_doc.doc)
        .def(ParamInit<Class>())
        .def_readwrite("publish_period",
            &MeshcatVisualizerParams::publish_period,
            cls_doc.publish_period.doc)
        .def_readwrite("role", &MeshcatVisualizerParams::role, cls_doc.role.doc)
        .def_readwrite("default_color", &MeshcatVisualizerParams::default_color,
            cls_doc.default_color.doc)
        .def_readwrite(
            "prefix", &MeshcatVisualizerParams::prefix, cls_doc.prefix.doc)
        .def_readwrite("delete_on_initialization_event",
            &MeshcatVisualizerParams::delete_on_initialization_event,
            cls_doc.delete_on_initialization_event.doc)
        .def("__repr__", [](const Class& self) {
          return py::str(
              "MeshcatVisualizerParams("
              "publish_period={}, "
              "role={}, "
              "default_color={}, "
              "prefix={}, "
              "delete_on_initialization_event={}")
              .format(self.publish_period, self.role, self.default_color,
                  self.prefix, self.delete_on_initialization_event);
        });
  }
}

}  // namespace

void DefineGeometryVisualizers(py::module m) {
  DoScalarIndependentDefinitions(m);
  type_visit([m](auto dummy) { DoScalarDependentDefinitions(m, dummy); },
      NonSymbolicScalarPack{});
}
}  // namespace pydrake
}  // namespace drake
