#include <memory>
#include <string>

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/systems/sensors/rgbd_camera.h"

using std::string;
using std::unique_ptr;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(sensors, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::sensors;
  constexpr auto& doc = pydrake_doc.drake.systems.sensors;

  m.doc() = "Attic bindings for the sensors portion of the Systems framework.";

  py::module::import("pydrake.attic.multibody.rigid_body_tree");
  py::module::import("pydrake.systems.sensors");

  using T = double;

  auto def_camera_ports = [](auto* ppy_class) {
    auto& py_class = *ppy_class;
    using PyClass = std::decay_t<decltype(py_class)>;
    using Class = typename PyClass::type;
    py_class
        .def(
            "state_input_port", &Class::state_input_port, py_reference_internal)
        .def("color_image_output_port", &Class::color_image_output_port,
            py_reference_internal)
        .def("depth_image_output_port", &Class::depth_image_output_port,
            py_reference_internal)
        .def("label_image_output_port", &Class::label_image_output_port,
            py_reference_internal)
        .def("camera_base_pose_output_port",
            &Class::camera_base_pose_output_port, py_reference_internal);
  };

  py::class_<RgbdCamera, LeafSystem<T>> rgbd_camera(m, "RgbdCamera");
  rgbd_camera
      .def(py::init<string, const RigidBodyTree<T>&, const RigidBodyFrame<T>&,
               double, double, double, bool, int, int>(),
          py::arg("name"), py::arg("tree"), py::arg("frame"),
          py::arg("z_near") = 0.5, py::arg("z_far") = 5.0,
          py::arg("fov_y") = M_PI_4,
          py::arg("show_window") = bool{RenderingConfig::kDefaultShowWindow},
          py::arg("width") = int{RenderingConfig::kDefaultWidth},
          py::arg("height") = int{RenderingConfig::kDefaultHeight},
          // Keep alive, reference: `this` keeps `RigidBodyTree` alive.
          py::keep_alive<1, 3>(), doc.RgbdCamera.ctor.doc_10args)
      .def("color_camera_info", &RgbdCamera::color_camera_info,
          py_reference_internal, doc.RgbdCamera.color_camera_info.doc)
      .def("depth_camera_info", &RgbdCamera::depth_camera_info,
          py_reference_internal, doc.RgbdCamera.depth_camera_info.doc)
      .def("color_camera_optical_pose", &RgbdCamera::color_camera_optical_pose,
          doc.RgbdCamera.color_camera_optical_pose.doc)
      .def("depth_camera_optical_pose", &RgbdCamera::depth_camera_optical_pose,
          doc.RgbdCamera.depth_camera_optical_pose.doc)
      .def("frame", &RgbdCamera::frame, py_reference_internal,
          doc.RgbdCamera.frame.doc)
      .def("frame", &RgbdCamera::frame, py_reference_internal,
          doc.RgbdCamera.frame.doc)
      .def("tree", &RgbdCamera::tree, py_reference, doc.RgbdCamera.tree.doc);
  def_camera_ports(&rgbd_camera);

  py::class_<RgbdCameraDiscrete, Diagram<T>> rgbd_camera_discrete(
      m, "RgbdCameraDiscrete", doc.RgbdCameraDiscrete.doc);
  rgbd_camera_discrete
      .def(py::init<unique_ptr<RgbdCamera>, double, bool>(), py::arg("camera"),
          py::arg("period") = double{RgbdCameraDiscrete::kDefaultPeriod},
          py::arg("render_label_image") = true,
          // Keep alive, ownership: `RgbdCamera` keeps `this` alive.
          py::keep_alive<2, 1>(), doc.RgbdCameraDiscrete.ctor.doc)
      // N.B. Since `camera` is already connected, we do not need additional
      // `keep_alive`s.
      .def("camera", &RgbdCameraDiscrete::camera,
          doc.RgbdCameraDiscrete.camera.doc)
      .def("mutable_camera", &RgbdCameraDiscrete::mutable_camera,
          doc.RgbdCameraDiscrete.mutable_camera.doc)
      .def("period", &RgbdCameraDiscrete::period,
          doc.RgbdCameraDiscrete.period.doc);
  def_camera_ports(&rgbd_camera_discrete);
  rgbd_camera_discrete.attr("kDefaultPeriod") =
      double{RgbdCameraDiscrete::kDefaultPeriod};
}

}  // namespace pydrake
}  // namespace drake
