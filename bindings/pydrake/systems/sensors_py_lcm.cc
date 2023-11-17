#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/systems/sensors_py.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"
#include "drake/systems/sensors/lcm_image_array_to_images.h"

namespace drake {
namespace pydrake {
namespace internal {

using systems::LeafSystem;

void DefineSensorsLcm(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::sensors;
  constexpr auto& doc = pydrake_doc.drake.systems.sensors;

  {
    using Class = LcmImageArrayToImages;
    constexpr auto& cls_doc = doc.LcmImageArrayToImages;
    py::class_<Class, LeafSystem<double>> cls(
        m, "LcmImageArrayToImages", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc)
        .def("image_array_t_input_port", &Class::image_array_t_input_port,
            py_rvp::reference_internal, cls_doc.image_array_t_input_port.doc)
        .def("color_image_output_port", &Class::color_image_output_port,
            py_rvp::reference_internal, cls_doc.color_image_output_port.doc)
        .def("depth_image_output_port", &Class::depth_image_output_port,
            py_rvp::reference_internal, cls_doc.depth_image_output_port.doc);
  }

  {
    using Class = ImageToLcmImageArrayT;
    constexpr auto& cls_doc = doc.ImageToLcmImageArrayT;
    py::class_<Class, LeafSystem<double>> cls(
        m, "ImageToLcmImageArrayT", cls_doc.doc);
    cls  // BR
        .def(py::init<const std::string&, const std::string&,
                 const std::string&, bool>(),
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

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
