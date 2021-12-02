#include <memory>
#include <string>
#include <vector>

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/eigen_geometry_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/sensors/pixel_types.h"
#include "drake/systems/sensors/rgbd_sensor.h"

using std::string;
using std::unique_ptr;
using std::vector;

namespace drake {
namespace pydrake {

template <typename T, T kPixelType>
using constant = std::integral_constant<T, kPixelType>;

template <typename T, T... kPixelTypes>
using constant_pack = type_pack<type_pack<constant<T, kPixelTypes>>...>;

using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using geometry::FrameId;
using geometry::render::ColorRenderCamera;
using geometry::render::DepthRenderCamera;
using math::RigidTransformd;
using math::RollPitchYawd;

PYBIND11_MODULE(sensors, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::sensors;
  constexpr auto& doc = pydrake_doc.drake.systems.sensors;

  m.doc() = "Bindings for the sensors portion of the Systems framework.";

  py::module::import("pydrake.common.eigen_geometry");
  py::module::import("pydrake.geometry.render");
  py::module::import("pydrake.systems.framework");

  // Expose only types that are used.
  py::enum_<PixelFormat>(m, "PixelFormat")
      .value("kRgba", PixelFormat::kRgba)
      .value("kDepth", PixelFormat::kDepth)
      .value("kLabel", PixelFormat::kLabel);

  vector<string> pixel_type_names = {
      "kRgba8U",
      "kDepth16U",
      "kDepth32F",
      "kLabel16I",
  };

  // This list should match pixel_type_names.
  using PixelTypeList = constant_pack<PixelType,  //
      PixelType::kRgba8U,                         //
      PixelType::kDepth16U,                       //
      PixelType::kDepth32F,                       //
      PixelType::kLabel16I>;

  {
    // Expose image types and their traits.
    py::enum_<PixelType> pixel_type(m, "PixelType");

    // This uses the `type_visit` pattern for looping. See `type_pack_test.cc`
    // for more information on the pattern.
    int pixel_type_index = 0;
    auto instantiation_visitor = [&](auto param) {
      // Extract information from inferred parameter.
      constexpr PixelType kPixelType =
          decltype(param)::template type_at<0>::value;
      using ImageT = Image<kPixelType>;
      using ImageTraitsT = ImageTraits<kPixelType>;
      using T = typename ImageTraitsT::ChannelType;

      // Get associated properites, and iterate.
      const std::string pixel_type_name = pixel_type_names[pixel_type_index];
      ++pixel_type_index;

      // Add definition to enum, before requesting the Python parameter.
      pixel_type.value(pixel_type_name.c_str(), kPixelType);
      py::tuple py_param = GetPyParam(param);

      // Add traits.
      py::class_<ImageTraitsT> traits(
          m, TemporaryClassName<ImageTraitsT>().c_str());
      traits.attr("ChannelType") = GetPyParam<T>()[0];
      traits.attr("kNumChannels") = int{ImageTraitsT::kNumChannels};
      traits.attr("kPixelFormat") = PixelFormat{ImageTraitsT::kPixelFormat};
      AddTemplateClass(m, "ImageTraits", traits, py_param);

      auto at = [](ImageT* self, int x, int y) {
        // Since Image<>::at(...) uses DRAKE_ASSERT for performance reasons,
        // rewrite the checks here using DRAKE_THROW_UNLESS so that it will not
        // segfault in Python.
        DRAKE_THROW_UNLESS(x >= 0 && x < self->width());
        DRAKE_THROW_UNLESS(y >= 0 && y < self->height());
        Map<VectorX<T>> pixel(self->at(x, y), int{ImageTraitsT::kNumChannels});
        return pixel;
      };
      // Shape for use with NumPy, OpenCV, etc. Using same shape as what is
      // present in `show_images.py`.
      auto get_shape = [](const ImageT* self) {
        return py::make_tuple(
            self->height(), self->width(), int{ImageTraitsT::kNumChannels});
      };
      auto get_data = [=](const ImageT* self) {
        py::object array =
            ToArray(self->at(0, 0), self->size(), get_shape(self));
        py_keep_alive(array, py::cast(self));
        return array;
      };
      auto get_mutable_data = [=](ImageT* self) {
        py::object array =
            ToArray(self->at(0, 0), self->size(), get_shape(self));
        py_keep_alive(array, py::cast(self));
        return array;
      };

      py::class_<ImageT> image(m, TemporaryClassName<ImageT>().c_str());
      AddTemplateClass(m, "Image", image, py_param);
      image  // BR
          .def(py::init<int, int>(), py::arg("width"), py::arg("height"),
              doc.Image.ctor.doc_2args)
          .def(py::init<int, int, T>(), py::arg("width"), py::arg("height"),
              py::arg("initial_value"), doc.Image.ctor.doc_3args)
          .def("width", &ImageT::width, doc.Image.width.doc)
          .def("height", &ImageT::height, doc.Image.height.doc)
          .def("size", &ImageT::size, doc.Image.size.doc)
          .def("resize", &ImageT::resize, doc.Image.resize.doc)
          .def("at", at, py::arg("x"), py::arg("y"), py_rvp::reference_internal,
              doc.Image.at.doc_2args_x_y_nonconst)
          // Non-C++ properties. Make them Pythonic.
          .def_property_readonly("shape", get_shape)
          .def_property_readonly("data", get_data)
          .def_property_readonly("mutable_data", get_mutable_data);
      // Constants.
      image.attr("Traits") = traits;
      // - Do not duplicate aliases (e.g. `kNumChannels`) for now.
      // Add type alias for instantiation.
      const std::string suffix = pixel_type_name.substr(1);
      m.attr(("Image" + suffix).c_str()) = image;
      // Add abstract values.
      AddValueInstantiation<ImageT>(m);
    };
    type_visit(instantiation_visitor, PixelTypeList{});
  }

  using T = double;

  // Systems.

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
            cls_doc.body_pose_in_world_output_port.doc);
  };

  py::class_<RgbdSensor, LeafSystem<T>> rgbd_sensor(
      m, "RgbdSensor", doc.RgbdSensor.doc);

  rgbd_sensor
      .def(py::init<FrameId, const RigidTransformd&, ColorRenderCamera,
               DepthRenderCamera>(),
          py::arg("parent_id"), py::arg("X_PB"), py::arg("color_camera"),
          py::arg("depth_camera"),
          doc.RgbdSensor.ctor.doc_individual_intrinsics)
      .def(py::init<FrameId, const RigidTransformd&, const DepthRenderCamera&,
               bool>(),
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

  py::class_<RgbdSensorDiscrete, Diagram<T>> rgbd_camera_discrete(
      m, "RgbdSensorDiscrete", doc.RgbdSensorDiscrete.doc);
  rgbd_camera_discrete
      .def(py::init<unique_ptr<RgbdSensor>, double, bool>(), py::arg("sensor"),
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
    using Class = CameraInfo;
    constexpr auto& cls_doc = doc.CameraInfo;
    py::class_<Class> cls(m, "CameraInfo", cls_doc.doc);
    cls  // BR
        .def(py::init<int, int, double>(), py::arg("width"), py::arg("height"),
            py::arg("fov_y"), cls_doc.ctor.doc_3args_width_height_fov_y)
        .def(py::init<int, int, const Matrix3d&>(), py::arg("width"),
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

  {
    using Class = ImageToLcmImageArrayT;
    constexpr auto& cls_doc = doc.ImageToLcmImageArrayT;
    py::class_<Class, LeafSystem<T>> cls(
        m, "ImageToLcmImageArrayT", cls_doc.doc);
    cls  // BR
        .def(py::init<const string&, const string&, const string&, bool>(),
            py::arg("color_frame_name"), py::arg("depth_frame_name"),
            py::arg("label_frame_name"), py::arg("do_compress") = false,
            cls_doc.ctor.doc_4args)
        .def(py::init<bool>(), py::arg("do_compress") = false,
            cls_doc.ctor.doc_1args)
        .def("color_image_input_port", &Class::color_image_input_port,
            py_rvp::reference_internal, cls_doc.color_image_input_port.doc)
        .def("depth_image_input_port", &Class::depth_image_input_port,
            py_rvp::reference_internal, cls_doc.depth_image_input_port.doc)
        .def("label_image_input_port", &Class::label_image_input_port,
            py_rvp::reference_internal, cls_doc.label_image_input_port.doc)
        .def("image_array_t_msg_output_port",
            &Class::image_array_t_msg_output_port, py_rvp::reference_internal,
            cls_doc.image_array_t_msg_output_port.doc);
    // Because the public interface requires templates and it's hard to
    // reproduce the logic publicly (e.g. no overload that just takes
    // `AbstractValue` and the pixel type), go ahead and bind the templated
    // methods.
    auto def_image_input_port = [&cls, cls_doc](auto param) {
      constexpr PixelType kPixelType =
          decltype(param)::template type_at<0>::value;
      AddTemplateMethod(cls, "DeclareImageInputPort",
          &Class::DeclareImageInputPort<kPixelType>, GetPyParam(param),
          py::arg("name"), py_rvp::reference_internal,
          cls_doc.DeclareImageInputPort.doc);
    };
    type_visit(def_image_input_port, PixelTypeList{});
  }
}

}  // namespace pydrake
}  // namespace drake
