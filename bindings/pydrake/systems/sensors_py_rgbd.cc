#include <memory>

#include "drake/bindings/generated_docstrings/systems_sensors.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/ref_cycle_pybind.h"
#include "drake/bindings/pydrake/systems/sensors_py.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/rgbd_sensor.h"
#include "drake/systems/sensors/rgbd_sensor_async.h"
#include "drake/systems/sensors/rgbd_sensor_discrete.h"

namespace drake {
namespace pydrake {
namespace internal {

using geometry::render::ColorRenderCamera;
using geometry::render::DepthRenderCamera;
using systems::Diagram;
using systems::LeafSystem;

void DefineSensorsRgbd(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::sensors;
  constexpr auto& doc = pydrake_doc_systems_sensors.drake.systems.sensors;

  auto def_camera_ports = [](auto* ppy_class, auto cls_doc) {
    auto& py_class = *ppy_class;
    using PyClass = std::decay_t<decltype(py_class)>;
    using Class = typename PyClass::type;
    py_class
        .def("query_object_input_port", &Class::query_object_input_port,
            py_rvp::reference_internal, cls_doc.query_object_input_port.doc)
        .def("color_image_output_port", &Class::color_image_output_port,
            py_rvp::reference_internal, cls_doc.color_image_output_port.doc)
        .def("depth_image_32F_output_port", &Class::depth_image_32F_output_port,
            py_rvp::reference_internal, cls_doc.depth_image_32F_output_port.doc)
        .def("depth_image_16U_output_port", &Class::depth_image_16U_output_port,
            py_rvp::reference_internal, cls_doc.depth_image_16U_output_port.doc)
        .def("label_image_output_port", &Class::label_image_output_port,
            py_rvp::reference_internal, cls_doc.label_image_output_port.doc)
        .def("body_pose_in_world_output_port",
            &Class::body_pose_in_world_output_port, py_rvp::reference_internal,
            cls_doc.body_pose_in_world_output_port.doc)
        .def("image_time_output_port", &Class::image_time_output_port,
            py_rvp::reference_internal, cls_doc.image_time_output_port.doc);
  };

  {
    using Class = CameraInfo;
    constexpr auto& cls_doc = doc.CameraInfo;
    py::class_<Class> cls(m, "CameraInfo", cls_doc.doc);
    cls  // BR
        .def(py::init<int, int, double>(), py::arg("width"), py::arg("height"),
            py::arg("fov_y"), cls_doc.ctor.doc_3args_width_height_fov_y)
        .def(py::init<int, int, const Eigen::Matrix3d&>(), py::arg("width"),
            py::arg("height"), py::arg("intrinsic_matrix"),
            cls_doc.ctor.doc_3args_width_height_intrinsic_matrix)
        .def(py::init<int, int, double, double, double, double>(),
            py::arg("width"), py::arg("height"), py::arg("focal_x"),
            py::arg("focal_y"), py::arg("center_x"), py::arg("center_y"),
            cls_doc.ctor
                .doc_6args_width_height_focal_x_focal_y_center_x_center_y)
        .def("width", &Class::width, cls_doc.width.doc)
        .def("height", &Class::height, cls_doc.height.doc)
        .def("focal_x", &Class::focal_x, cls_doc.focal_x.doc)
        .def("focal_y", &Class::focal_y, cls_doc.focal_y.doc)
        .def("fov_x", &Class::fov_x, cls_doc.fov_x.doc)
        .def("fov_y", &Class::fov_y, cls_doc.fov_y.doc)
        .def("center_x", &Class::center_x, cls_doc.center_x.doc)
        .def("center_y", &Class::center_y, cls_doc.center_y.doc)
        .def("intrinsic_matrix", &Class::intrinsic_matrix,
            cls_doc.intrinsic_matrix.doc)
        .def(py::pickle(
            [](const Class& self) {
              return py::make_tuple(self.width(), self.height(), self.focal_x(),
                  self.focal_y(), self.center_x(), self.center_y());
            },
            [](py::tuple t) {
              DRAKE_DEMAND(t.size() == 6);
              return Class(t[0].cast<int>(), t[1].cast<int>(),
                  t[2].cast<double>(), t[3].cast<double>(), t[4].cast<double>(),
                  t[5].cast<double>());
            }));
  }

  py::class_<RgbdSensor, LeafSystem<double>> rgbd_sensor(
      m, "RgbdSensor", doc.RgbdSensor.doc);

  rgbd_sensor
      .def(py::init<geometry::FrameId, const math::RigidTransformd&,
               ColorRenderCamera, DepthRenderCamera>(),
          py::arg("parent_id"), py::arg("X_PB"), py::arg("color_camera"),
          py::arg("depth_camera"),
          doc.RgbdSensor.ctor.doc_individual_intrinsics)
      .def(py::init<geometry::FrameId, const math::RigidTransformd&,
               const DepthRenderCamera&, bool>(),
          py::arg("parent_id"), py::arg("X_PB"), py::arg("depth_camera"),
          py::arg("show_window") = false,
          doc.RgbdSensor.ctor.doc_combined_intrinsics)
      .def("default_color_render_camera",
          &RgbdSensor::default_color_render_camera, py_rvp::reference_internal,
          doc.RgbdSensor.default_color_render_camera.doc)
      .def("set_default_color_render_camera",
          &RgbdSensor::set_default_color_render_camera, py::arg("color_camera"),
          doc.RgbdSensor.set_default_color_render_camera.doc)
      .def("GetColorRenderCamera", &RgbdSensor::GetColorRenderCamera,
          py::arg("context"), py_rvp::reference_internal,
          doc.RgbdSensor.GetColorRenderCamera.doc)
      .def("SetColorRenderCamera", &RgbdSensor::SetColorRenderCamera,
          py::arg("context"), py::arg("color_camera"),
          doc.RgbdSensor.SetColorRenderCamera.doc)
      .def("default_depth_render_camera",
          &RgbdSensor::default_depth_render_camera, py_rvp::reference_internal,
          doc.RgbdSensor.default_depth_render_camera.doc)
      .def("set_default_depth_render_camera",
          &RgbdSensor::set_default_depth_render_camera, py::arg("depth_camera"),
          doc.RgbdSensor.set_default_depth_render_camera.doc)
      .def("GetDepthRenderCamera", &RgbdSensor::GetDepthRenderCamera,
          py::arg("context"), py_rvp::reference_internal,
          doc.RgbdSensor.GetDepthRenderCamera.doc)
      .def("SetDepthRenderCamera", &RgbdSensor::SetDepthRenderCamera,
          py::arg("context"), py::arg("depth_camera"),
          doc.RgbdSensor.SetDepthRenderCamera.doc)
      .def("default_X_PB", &RgbdSensor::default_X_PB,
          py_rvp::reference_internal, doc.RgbdSensor.default_X_PB.doc)
      .def("set_default_X_PB", &RgbdSensor::set_default_X_PB,
          py::arg("sensor_pose"), doc.RgbdSensor.set_default_X_PB.doc)
      .def("GetX_PB", &RgbdSensor::GetX_PB, py::arg("context"),
          py_rvp::reference_internal, doc.RgbdSensor.GetX_PB.doc)
      .def("SetX_PB", &RgbdSensor::SetX_PB, py::arg("context"),
          py::arg("sensor_pose"), doc.RgbdSensor.SetX_PB.doc)
      .def("default_parent_frame_id", &RgbdSensor::default_parent_frame_id,
          doc.RgbdSensor.default_parent_frame_id.doc)
      .def("set_default_parent_frame_id",
          &RgbdSensor::set_default_parent_frame_id, py::arg("id"),
          doc.RgbdSensor.set_default_parent_frame_id.doc)
      .def("GetParentFrameId", &RgbdSensor::GetParentFrameId,
          py::arg("context"), doc.RgbdSensor.GetParentFrameId.doc)
      .def("SetParentFrameId", &RgbdSensor::SetParentFrameId,
          py::arg("context"), py::arg("id"),
          doc.RgbdSensor.SetParentFrameId.doc);
  def_camera_ports(&rgbd_sensor, doc.RgbdSensor);

  py::class_<RgbdSensorDiscrete, Diagram<double>> rgbd_camera_discrete(
      m, "RgbdSensorDiscrete", doc.RgbdSensorDiscrete.doc);
  rgbd_camera_discrete
      .def(py::init(
               [](RgbdSensor& sensor, double period, bool render_label_image) {
                 // The C++ constructor doesn't offer a bare-pointer overload,
                 // only shared_ptr. Because object lifetime is already handled
                 // by the ref_cycle annotation below (as required for all
                 // subclasses of Diagram), we can pass the `sensor` as an
                 // unowned shared_ptr.
                 return std::make_unique<RgbdSensorDiscrete>(
                     make_unowned_shared_ptr_from_raw(&sensor), period,
                     render_label_image);
               }),
          py::arg("sensor"),
          py::arg("period") = double{RgbdSensorDiscrete::kDefaultPeriod},
          py::arg("render_label_image") = true,
          // `self` and `sensor` form a cycle as part of the Diagram.
          internal::ref_cycle<1, 2>(), doc.RgbdSensorDiscrete.ctor.doc)
      // N.B. Since `camera` is already connected, we do not need additional
      // `keep_alive`s.
      .def("sensor", &RgbdSensorDiscrete::sensor, py_rvp::reference_internal,
          doc.RgbdSensorDiscrete.sensor.doc)
      .def("period", &RgbdSensorDiscrete::period,
          doc.RgbdSensorDiscrete.period.doc);
  def_camera_ports(&rgbd_camera_discrete, doc.RgbdSensorDiscrete);
  rgbd_camera_discrete.attr("kDefaultPeriod") =
      double{RgbdSensorDiscrete::kDefaultPeriod};

  {
    using Class = RgbdSensorAsync;
    constexpr auto& cls_doc = doc.RgbdSensorAsync;
    py::class_<Class, LeafSystem<double>> rgbd_sensor_async(
        m, "RgbdSensorAsync", cls_doc.doc);
    rgbd_sensor_async
        .def(py::init<const geometry::SceneGraph<double>*, geometry::FrameId,
                 const math::RigidTransformd&, double, double, double,
                 std::optional<ColorRenderCamera>,
                 std::optional<DepthRenderCamera>, bool>(),
            py::arg("scene_graph"), py::arg("parent_id"), py::arg("X_PB"),
            py::arg("fps"), py::arg("capture_offset"), py::arg("output_delay"),
            py::arg("color_camera"), py::arg("depth_camera") = std::nullopt,
            py::arg("render_label_image") = false, cls_doc.ctor.doc)
        .def("fps", &Class::fps, cls_doc.fps.doc)
        .def("capture_offset", &Class::capture_offset,
            cls_doc.capture_offset.doc)
        .def("output_delay", &Class::output_delay, cls_doc.output_delay.doc)
        .def("default_parent_frame_id", &Class::default_parent_frame_id,
            cls_doc.default_parent_frame_id.doc)
        .def("set_default_parent_frame_id", &Class::set_default_parent_frame_id,
            py::arg("id"), cls_doc.set_default_parent_frame_id.doc)
        .def("GetParentFrameId", &Class::GetParentFrameId, py::arg("context"),
            cls_doc.GetParentFrameId.doc)
        .def("SetParentFrameId", &Class::SetParentFrameId, py::arg("context"),
            py::arg("id"), cls_doc.SetParentFrameId.doc)
        .def("default_X_PB", &Class::default_X_PB, py_rvp::reference_internal,
            cls_doc.default_X_PB.doc)
        .def("set_default_X_PB", &Class::set_default_X_PB,
            py::arg("sensor_pose"), cls_doc.set_default_X_PB.doc)
        .def(
            "GetX_PB", &Class::GetX_PB, py::arg("context"), cls_doc.GetX_PB.doc)
        .def("SetX_PB", &Class::SetX_PB, py::arg("context"),
            py::arg("sensor_pose"), cls_doc.SetX_PB.doc)
        .def("default_color_render_camera", &Class::default_color_render_camera,
            py_rvp::reference_internal, cls_doc.default_color_render_camera.doc)
        .def("set_default_color_render_camera",
            &Class::set_default_color_render_camera, py::arg("color_camera"),
            cls_doc.set_default_color_render_camera.doc)
        .def("GetColorRenderCamera", &Class::GetColorRenderCamera,
            py::arg("context"), cls_doc.GetColorRenderCamera.doc)
        .def("SetColorRenderCamera", &Class::SetColorRenderCamera,
            py::arg("context"), py::arg("color_camera"),
            cls_doc.SetColorRenderCamera.doc)
        .def("default_depth_render_camera", &Class::default_depth_render_camera,
            py_rvp::reference_internal, cls_doc.default_depth_render_camera.doc)
        .def("set_default_depth_render_camera",
            &Class::set_default_depth_render_camera, py::arg("depth_camera"),
            cls_doc.set_default_depth_render_camera.doc)
        .def("GetDepthRenderCamera", &Class::GetDepthRenderCamera,
            py::arg("context"), cls_doc.GetDepthRenderCamera.doc)
        .def("SetDepthRenderCamera", &Class::SetDepthRenderCamera,
            py::arg("context"), py::arg("depth_camera"),
            cls_doc.SetDepthRenderCamera.doc)
        .def("color_image_output_port", &Class::color_image_output_port,
            py_rvp::reference_internal, cls_doc.color_image_output_port.doc)
        .def("depth_image_32F_output_port", &Class::depth_image_32F_output_port,
            py_rvp::reference_internal, cls_doc.depth_image_32F_output_port.doc)
        .def("depth_image_16U_output_port", &Class::depth_image_16U_output_port,
            py_rvp::reference_internal, cls_doc.depth_image_16U_output_port.doc)
        .def("label_image_output_port", &Class::label_image_output_port,
            py_rvp::reference_internal, cls_doc.label_image_output_port.doc)
        .def("body_pose_in_world_output_port",
            &Class::body_pose_in_world_output_port, py_rvp::reference_internal,
            cls_doc.body_pose_in_world_output_port.doc)
        .def("image_time_output_port", &Class::image_time_output_port,
            py_rvp::reference_internal, cls_doc.image_time_output_port.doc);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
