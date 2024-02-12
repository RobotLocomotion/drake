#include "drake/bindings/pydrake/common/cpp_template_pybind.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/systems/sensors_py.h"
#include "drake/systems/sensors/image_file_format.h"
#include "drake/systems/sensors/image_io.h"
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
    py::enum_<ImageFileFormat>(m, "ImageFileFormat")
        .value("kJpeg", ImageFileFormat::kJpeg)
        .value("kPng", ImageFileFormat::kPng)
        .value("kTiff", ImageFileFormat::kTiff);
  }

  {
    using Class = ImageIo;
    constexpr auto& cls_doc = doc.ImageIo;
    py::class_<Class> cls(m, "ImageIo", cls_doc.doc);

    {
      py::class_<Class::Metadata> metadata_cls(
          cls, "Metadata", cls_doc.Metadata.doc);
      metadata_cls.def(ParamInit<Class::Metadata>());
      DefAttributesUsingSerialize(&metadata_cls, cls_doc.Metadata);
      DefReprUsingSerialize(&metadata_cls);
      DefCopyAndDeepCopy(&metadata_cls);
    }

    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc)
        .def("LoadMetadata",
            overload_cast_explicit<std::optional<Class::Metadata>,
                const std::filesystem::path&>(&Class::LoadMetadata),
            py::arg("path"), cls_doc.LoadMetadata.doc_1args_path)
        .def(
            "LoadMetadata",
            [](const Class& self, py::bytes buffer) {
              const std::string_view view{buffer};
              return self.LoadMetadata(
                  Class::ByteSpan{view.data(), view.size()});
            },
            py::arg("buffer"), cls_doc.LoadMetadata.doc_1args_buffer)
        .def(
            "Load",
            [](const Class& self, const std::filesystem::path& path,
                std::optional<ImageFileFormat> format) {
              return self.Load(path, format);
            },
            py::arg("path"), py::arg("format") = std::nullopt,
            cls_doc.Load.doc_2args_path_format)
        .def(
            "Load",
            [](const Class& self, py::bytes buffer,
                std::optional<ImageFileFormat> format) {
              const std::string_view view{buffer};
              return self.Load(
                  Class::ByteSpan{view.data(), view.size()}, format);
            },
            py::arg("buffer"), py::arg("format") = std::nullopt,
            cls_doc.Load.doc_2args_buffer_format)
        .def(
            "Save",
            [](const Class& self, const ImageAny& image_any,
                const std::filesystem::path& path,
                std::optional<ImageFileFormat> format) {
              std::visit(
                  [&self, &path, &format](
                      const auto& image) { self.Save(image, path, format); },
                  image_any);
            },
            py::arg("image"), py::arg("path"), py::arg("format") = std::nullopt,
            cls_doc.Save.doc_3args)
        .def(
            "Save",
            [](const Class& self, const ImageAny& image_any,
                ImageFileFormat format) {
              const std::vector<uint8_t> result = std::visit(
                  [&self, &format](
                      const auto& image) { return self.Save(image, format); },
                  image_any);
              return py::bytes(
                  reinterpret_cast<const char*>(result.data()), result.size());
            },
            py::arg("image"), py::arg("format"), cls_doc.Save.doc_2args);
  }

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
