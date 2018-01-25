#include <string>
#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake/bindings/pydrake/util/cpp_template_pybind.h"
#include "drake/bindings/pydrake/util/type_pack.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/pixel_types.h"

using std::string;
using std::vector;

namespace drake {
namespace pydrake {

template <typename T, T Value>
using constant = std::integral_constant<T, Value>;

template <typename T, T ... Values>
using constant_pack = type_pack<type_pack<constant<T, Values>>...>;

using Eigen::Map;
using Eigen::Ref;

// TODO(eric.cousineau): Place in `pydrake_pybind.h`.
template <typename T>
py::object ToArray(T* ptr, int size, py::tuple shape) {
  // Create flat array to be reshaped in numpy.
  using Vector = VectorX<T>;
  Map<Vector> data(ptr, size);
  return py::cast(Ref<Vector>(data), py_reference).attr("reshape")(shape);
}

// `const` variant.
template <typename T>
py::object ToArray(const T* ptr, int size, py::tuple shape) {
  // Create flat array to be reshaped in numpy.
  using Vector = const VectorX<T>;
  Map<Vector> data(ptr, size);
  return py::cast(Ref<Vector>(data), py_reference).attr("reshape")(shape);
}

PYBIND11_MODULE(sensors, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::sensors;

  m.doc() = "Bindings for the sensors portion of the Systems framework.";

  // Expose only types that are used.
  py::enum_<PixelFormat>(m, "PixelFormat")
      .value("kRgba", PixelFormat::kRgba)
      .value("kDepth", PixelFormat::kDepth)
      .value("kLabel", PixelFormat::kLabel);

  {
    // Expose image types and their traits.
    py::enum_<PixelType> pixel_type(m, "PixelType");
    vector<string> enum_names = {
        "kRgba8U",
        "kDepth32F",
        "kLabel16I",
    };

    using ParamList = constant_pack<PixelType,
        PixelType::kRgba8U,
        PixelType::kDepth32F,
        PixelType::kLabel16I>;

    // Simple constexpr for-loop.
    int i = 0;
    auto instantiation_visitor = [&](auto param) {
      // Extract information from inferred parameter.
      using Param = decltype(param);
      static_assert(Param::size == 1, "Should have scalar type_pack");
      constexpr PixelType Value = Param::template type_at<0>::value;

      using ImageT = Image<Value>;
      using ImageTraitsT = ImageTraits<Value>;
      using T = typename ImageTraitsT::ChannelType;

      // Add definition to enum, before requesting the Python parameter.
      pixel_type.value(enum_names[i].c_str(), Value);
      py::tuple py_param = GetPyParam(param);

      // Add traits.
      py::class_<ImageTraitsT> traits(
          m, TemporaryClassName<ImageTraitsT>().c_str());
      traits.attr("ChannelType") = GetPyParam<T>()[0];
      traits.attr("kNumChannels") = int{ImageTraitsT::kNumChannels};
      traits.attr("kPixelFormat") = PixelFormat{ImageTraitsT::kPixelFormat};
      AddTemplateClass(m, "ImageTraits", traits, py_param);

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
      auto check_coord = [](const ImageT* self, int x, int y) {
        // Since Image<>::at(...) uses DRAKE_ASSERT for performance reasons,
        // rewrite the checks here using DRAKE_THROW_UNLESS so that it will not
        // segfault in Python.
        DRAKE_THROW_UNLESS(x >= 0 && x < self->width());
        DRAKE_THROW_UNLESS(y >= 0 && y < self->height());
      };

      py::class_<ImageT> image(m, TemporaryClassName<ImageT>().c_str());
      image
          .def(py::init<int, int>())
          .def(py::init<int, int, T>())
          .def("width", &ImageT::width)
          .def("height", &ImageT::height)
          .def("size", &ImageT::size)
          .def("resize", &ImageT::resize)
          .def("at", [=](ImageT* self, int x, int y) {
                check_coord(self, x, y);
                Map<VectorX<T>> pixel(
                    self->at(x, y), int{ImageTraitsT::kNumChannels});
                return pixel;
              }, py::arg("x"), py::arg("y"), py_reference_internal)
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
      m.attr(("Image" + enum_names[i].substr(1)).c_str()) = image;
      // Ensure that iterate.
      ++i;
    };
    type_visit(instantiation_visitor, ParamList{});
  }

  // Constants.
  py::class_<InvalidDepth> invalid_depth(m, "InvalidDepth");
  invalid_depth.attr("kTooFar") = InvalidDepth::kTooFar;
  invalid_depth.attr("kTooClose") = InvalidDepth::kTooClose;

  py::class_<Label> label(m, "Label");
  label.attr("kNoBody") = Label::kNoBody;
  label.attr("kFlatTerrain") = Label::kFlatTerrain;
}

}  // namespace pydrake
}  // namespace drake
