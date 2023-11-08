#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/systems/sensors_py.h"
#include "drake/systems/sensors/image_writer.h"

namespace drake {
namespace pydrake {
namespace internal {

using systems::LeafSystem;

void DefineSensorsImageIo(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::systems::sensors;
  constexpr auto& doc = pydrake_doc.drake.systems.sensors;

  {
    using Class = ImageWriter;
    constexpr auto& cls_doc = doc.ImageWriter;
    py::class_<Class, LeafSystem<double>> cls(m, "ImageWriter", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc)
        .def(
            "DeclareImageInputPort",
            [](Class& self, PixelType pixel_type, std::string port_name,
                std::string file_name_format, double publish_period,
                double start_time) -> const systems::InputPort<double>& {
              return self.DeclareImageInputPort(pixel_type,
                  std::move(port_name), std::move(file_name_format),
                  publish_period, start_time);
            },
            py::arg("pixel_type"), py::arg("port_name"),
            py::arg("file_name_format"), py::arg("publish_period"),
            py::arg("start_time"), py_rvp::reference_internal,
            cls_doc.DeclareImageInputPort.doc)
        .def("ResetAllImageCounts", &Class::ResetAllImageCounts,
            cls_doc.ResetAllImageCounts.doc);
  }
}

}  // namespace internal
}  // namespace pydrake
}  // namespace drake
