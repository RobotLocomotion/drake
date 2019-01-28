#include <memory>
#include <string>
#include <vector>

#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/eigen_geometry_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/common/type_pack.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/systems/systems_pybind.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/sensors/pixel_types.h"
#include "drake/systems/sensors/rgbd_camera.h"

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

PYBIND11_MODULE(sensors, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems;
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::sensors;
  constexpr auto& doc = pydrake_doc.drake.systems.sensors;

  m.doc() = "Bindings for the sensors portion of the Systems framework.";

  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.common.eigen_geometry");

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
        return ToArray(self->at(0, 0), self->size(), get_shape(self));
      };
      auto get_mutable_data = [=](ImageT* self) {
        return ToArray(self->at(0, 0), self->size(), get_shape(self));
      };

      py::class_<ImageT> image(m, TemporaryClassName<ImageT>().c_str());
      image  // BR
          .def(py::init<int, int>(), py::arg("width"), py::arg("height"),
              doc.Image.ctor.doc_2args)
          .def(py::init<int, int, T>(), py::arg("width"), py::arg("height"),
              py::arg("initial_value"), doc.Image.ctor.doc_3args)
          .def("width", &ImageT::width, doc.Image.width.doc)
          .def("height", &ImageT::height, doc.Image.height.doc)
          .def("size", &ImageT::size, doc.Image.size.doc)
          .def("resize", &ImageT::resize, doc.Image.resize.doc)
          .def("at", at, py::arg("x"), py::arg("y"), py_reference_internal,
              doc.Image.at.doc_2args_x_y_nonconst)
          // Non-C++ properties. Make them Pythonic.
          .def_property_readonly("shape", get_shape)
          .def_property_readonly("data", get_data, py_reference_internal)
          .def_property_readonly(
              "mutable_data", get_mutable_data, py_reference_internal);
      // Constants.
      image.attr("Traits") = traits;
      // - Do not duplicate aliases (e.g. `kNumChannels`) for now.
      AddTemplateClass(m, "Image", image, py_param);
      // Add type alias for instantiation.
      const std::string suffix = pixel_type_name.substr(1);
      m.attr(("Image" + suffix).c_str()) = image;
      // Add abstract values.
      pysystems::AddValueInstantiation<ImageT>(m);
    };
    type_visit(instantiation_visitor, PixelTypeList{});
  }

  // Constants.
  py::class_<InvalidDepth> invalid_depth(m, "InvalidDepth");
  invalid_depth.attr("kTooFar") = InvalidDepth::kTooFar;
  invalid_depth.attr("kTooClose") = InvalidDepth::kTooClose;

  py::class_<Label> label(m, "Label");
  label.attr("kNoBody") = Label::kNoBody;
  label.attr("kFlatTerrain") = Label::kFlatTerrain;

  using T = double;

  // Systems.
  py::class_<CameraInfo>(m, "CameraInfo", doc.CameraInfo.doc)
      .def(py::init<int, int, double>(), py::arg("width"), py::arg("height"),
          py::arg("fov_y"), doc.CameraInfo.ctor.doc_3args)
      .def(py::init<int, int, double, double, double, double>(),
          py::arg("width"), py::arg("height"), py::arg("focal_x"),
          py::arg("focal_y"), py::arg("center_x"), py::arg("center_y"),
          doc.CameraInfo.ctor.doc_6args)
      .def("width", &CameraInfo::width, doc.CameraInfo.width.doc)
      .def("height", &CameraInfo::height, doc.CameraInfo.height.doc)
      .def("focal_x", &CameraInfo::focal_x, doc.CameraInfo.focal_x.doc)
      .def("focal_y", &CameraInfo::focal_y, doc.CameraInfo.focal_y.doc)
      .def("center_x", &CameraInfo::center_x, doc.CameraInfo.center_x.doc)
      .def("center_y", &CameraInfo::center_y, doc.CameraInfo.center_y.doc)
      .def("intrinsic_matrix", &CameraInfo::intrinsic_matrix,
          doc.CameraInfo.intrinsic_matrix.doc);

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

  // TODO(eric.cousineau): Use something like `RenderingConfig`, per (#8123).
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
          // Keep alive, reference: `this` keeps  `RigidBodyTree` alive.
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

  {
    constexpr auto& cls_doc = doc.ImageToLcmImageArrayT;
    using Class = ImageToLcmImageArrayT;
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
            py_reference_internal, cls_doc.color_image_input_port.doc)
        .def("depth_image_input_port", &Class::depth_image_input_port,
            py_reference_internal, cls_doc.depth_image_input_port.doc)
        .def("label_image_input_port", &Class::label_image_input_port,
            py_reference_internal, cls_doc.label_image_input_port.doc)
        .def("image_array_t_msg_output_port",
            &Class::image_array_t_msg_output_port, py_reference_internal,
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
          py::arg("name"), py_reference_internal,
          cls_doc.DeclareImageInputPort.doc);
    };
    type_visit(def_image_input_port, PixelTypeList{});
  }
}

}  // namespace pydrake
}  // namespace drake
