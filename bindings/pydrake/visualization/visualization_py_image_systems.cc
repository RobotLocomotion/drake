#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/bindings/pydrake/visualization/visualization_py.h"
#include "drake/visualization/colorize_depth_image.h"
#include "drake/visualization/colorize_label_image.h"
#include "drake/visualization/concatenate_images.h"

namespace drake {
namespace pydrake {
namespace internal {

void DefineVisualizationImageSystems(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::visualization;
  constexpr auto& doc = pydrake_doc.drake.visualization;

  {
    using Class = ColorizeDepthImage<double>;
    constexpr auto& cls_doc = doc.ColorizeDepthImage;
    py::class_<Class, systems::LeafSystem<double>>(
        m, "ColorizeDepthImage", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc)
        .def_property("invalid_color", &Class::get_invalid_color,
            &Class::set_invalid_color,
            "The color used for pixels with too-near or too-far depth.")
        .def("Calc",
            overload_cast_explicit<void, const systems::sensors::ImageDepth32F&,
                systems::sensors::ImageRgba8U*>(&Class::Calc),
            cls_doc.Calc.doc)
        .def("Calc",
            overload_cast_explicit<void, const systems::sensors::ImageDepth16U&,
                systems::sensors::ImageRgba8U*>(&Class::Calc),
            cls_doc.Calc.doc);
  }

  {
    using Class = ColorizeLabelImage<double>;
    constexpr auto& cls_doc = doc.ColorizeLabelImage;
    py::class_<Class, systems::LeafSystem<double>>(
        m, "ColorizeLabelImage", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc)
        .def_property("background_color", &Class::get_background_color,
            &Class::set_background_color,
            "The color used for pixels with no label.")
        .def("Calc", &Class::Calc, cls_doc.Calc.doc);
  }

  {
    using Class = ConcatenateImages<double>;
    constexpr auto& cls_doc = doc.ConcatenateImages;
    py::class_<Class, systems::LeafSystem<double>>(
        m, "ConcatenateImages", cls_doc.doc)
        .def(py::init<int, int>(), py::kw_only(), py::arg("rows") = 1,
            py::arg("cols") = 1, cls_doc.ctor.doc)
        .def("get_input_port", &Class::get_input_port, py::kw_only(),
            py::arg("row"), py::arg("col"), py_rvp::reference_internal,
            cls_doc.get_input_port.doc);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
