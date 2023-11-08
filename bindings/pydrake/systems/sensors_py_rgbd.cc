#include "drake/bindings/pydrake/documentation_pybind.h"
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
  constexpr auto& doc = pydrake_doc.drake.systems.sensors;

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
      .def("color_camera_info", &RgbdSensor::color_camera_info,
          py_rvp::reference_internal, doc.RgbdSensor.color_camera_info.doc)
      .def("depth_camera_info", &RgbdSensor::depth_camera_info,
          py_rvp::reference_internal, doc.RgbdSensor.depth_camera_info.doc)
      .def("X_BC", &RgbdSensor::X_BC, doc.RgbdSensor.X_BC.doc)
      .def("X_BD", &RgbdSensor::X_BD, doc.RgbdSensor.X_BD.doc)
      .def("parent_frame_id", &RgbdSensor::parent_frame_id,
          py_rvp::reference_internal, doc.RgbdSensor.parent_frame_id.doc);
  def_camera_ports(&rgbd_sensor, doc.RgbdSensor);

  py::class_<RgbdSensorDiscrete, Diagram<double>> rgbd_camera_discrete(
      m, "RgbdSensorDiscrete", doc.RgbdSensorDiscrete.doc);
  rgbd_camera_discrete
      .def(py::init<std::unique_ptr<RgbdSensor>, double, bool>(),
          py::arg("sensor"),
          py::arg("period") = double{RgbdSensorDiscrete::kDefaultPeriod},
          py::arg("render_label_image") = true,
          // Keep alive, ownership: `sensor` keeps `self` alive.
          py::keep_alive<2, 1>(), doc.RgbdSensorDiscrete.ctor.doc)
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
    py::class_<Class, LeafSystem<double>>(m, "RgbdSensorAsync", cls_doc.doc)
        .def(py::init<const geometry::SceneGraph<double>*, geometry::FrameId,
                 const math::RigidTransformd&, double, double, double,
                 std::optional<ColorRenderCamera>,
                 std::optional<DepthRenderCamera>, bool>(),
            py::arg("scene_graph"), py::arg("parent_id"), py::arg("X_PB"),
            py::arg("fps"), py::arg("capture_offset"), py::arg("output_delay"),
            py::arg("color_camera"), py::arg("depth_camera") = std::nullopt,
            py::arg("render_label_image") = false, cls_doc.ctor.doc)
        .def("parent_id", &Class::parent_id, cls_doc.parent_id.doc)
        .def("X_PB", &Class::X_PB, cls_doc.X_PB.doc)
        .def("fps", &Class::fps, cls_doc.fps.doc)
        .def("capture_offset", &Class::capture_offset,
            cls_doc.capture_offset.doc)
        .def("output_delay", &Class::output_delay, cls_doc.output_delay.doc)
        .def("color_camera", &Class::color_camera, cls_doc.color_camera.doc)
        .def("depth_camera", &Class::depth_camera, cls_doc.depth_camera.doc)
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
