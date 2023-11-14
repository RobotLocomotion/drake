#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/eigen_pybind.h"
#include "drake/bindings/pydrake/common/value_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/systems/sensors_py.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/pixel_types.h"

namespace drake {
namespace pydrake {
namespace internal {

using Eigen::Map;

void DefineSensorsImage(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::sensors;
  constexpr auto& doc = pydrake_doc.drake.systems.sensors;

  // Note: for this module's C++ enums we choose not to bind the C++ `to_string`
  // functions as `__str__` in Python. The `enum.Enum` class already provides a
  // `__str__` that looks more Pythonic than our C++ `to_string`.

  py::enum_<PixelFormat>(m, "PixelFormat")
      .value("kRgb", PixelFormat::kRgb)
      .value("kBgr", PixelFormat::kBgr)
      .value("kRgba", PixelFormat::kRgba)
      .value("kBgra", PixelFormat::kBgra)
      .value("kGrey", PixelFormat::kGrey)
      .value("kDepth", PixelFormat::kDepth)
      .value("kLabel", PixelFormat::kLabel);

  py::enum_<PixelScalar>(m, "PixelScalar")
      .value("k8U", PixelScalar::k8U)
      .value("k16I", PixelScalar::k16I)
      .value("k16U", PixelScalar::k16U)
      .value("k32F", PixelScalar::k32F);

  {
    // Expose image types and their traits.
    py::enum_<PixelType> pixel_type(m, "PixelType");

    // This uses the `type_visit` pattern for looping. See `type_pack_test.cc`
    // for more information on the pattern.
    auto instantiation_visitor = [&](auto param) {
      // Extract information from inferred parameter.
      constexpr PixelType kPixelType =
          decltype(param)::template type_at<0>::value;
      using ImageT = Image<kPixelType>;
      using ImageTraitsT = ImageTraits<kPixelType>;
      using T = typename ImageTraitsT::ChannelType;
      const std::string pixel_type_name = "k" + to_string(kPixelType);

      // Add definition to enum, before requesting the Python parameter.
      pixel_type.value(pixel_type_name.c_str(), kPixelType);
      py::tuple py_param = GetPyParam(param);

      // Add traits. Note that we choose not to bind kPixelScalar because the
      // ChannelType already makes the same information easily available.
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
        py::object array = ToArray(self->at(0, 0), self->size(),
            get_shape(self), py_rvp::reference_internal, py::cast(self));
        return array;
      };
      auto get_mutable_data = [=](ImageT* self) {
        py::object array = ToArray(self->at(0, 0), self->size(),
            get_shape(self), py_rvp::reference_internal, py::cast(self));
        return array;
      };

      py::class_<ImageT> image(m, TemporaryClassName<ImageT>().c_str());
      AddTemplateClass(m, "Image", image, py_param);
      image  // BR
          .def(py::init<>(), doc.Image.ctor.doc_0args)
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

  // Image conversion functions.
  m  // BR
      .def("ConvertDepth32FTo16U", &ConvertDepth32FTo16U, py::arg("input"),
          py::arg("output"))
      .def("ConvertDepth16UTo32F", &ConvertDepth16UTo32F, py::arg("input"),
          py::arg("output"));
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
