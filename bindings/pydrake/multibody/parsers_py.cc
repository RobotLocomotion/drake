#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/parsers/package_map.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(parsers, m) {
  using drake::multibody::parsing::PackageMap;
  auto& doc = pydrake_doc.drake.multibody.parsing;

  m.doc() = "Tools for loading robots from various files";

  py::class_<PackageMap>(m, "PackageMap", doc.PackageMap.doc)
    .def(py::init<>(), doc.PackageMap.ctor.doc)
    .def("Add", &PackageMap::Add, doc.PackageMap.Add.doc)
    .def("Contains", &PackageMap::Contains, doc.PackageMap.Contains.doc)
    .def("size", &PackageMap::size, doc.PackageMap.size.doc)
    .def("GetPath", &PackageMap::GetPath, doc.PackageMap.GetPath.doc)
    .def("PopulateFromFolder", &PackageMap::PopulateFromFolder,
         doc.PackageMap.PopulateFromFolder.doc)
    .def("PopulateFromEnvironment", &PackageMap::PopulateFromEnvironment,
         doc.PackageMap.PopulateFromEnvironment.doc)
    .def("PopulateUpstreamToDrake", &PackageMap::PopulateUpstreamToDrake,
         doc.PackageMap.PopulateUpstreamToDrake.doc);
}

}  // namespace pydrake
}  // namespace drake
