#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/parsers/package_map.h"

#define D(...) DOC(drake, multibody, parsing, PackageMap, __VA_ARGS__)

namespace drake {
namespace pydrake {

PYBIND11_MODULE(parsers, m) {
  using drake::parsers::PackageMap;

  m.doc() = "Tools for loading robots from various files";

  py::class_<PackageMap>(m, "PackageMap")
    .def(py::init<>(), D(PackageMap))
    .def("Add", &PackageMap::Add, D(Add))
    .def("Contains", &PackageMap::Contains, D(Contains))
    .def("size", &PackageMap::size, D(size))
    .def("GetPath", &PackageMap::GetPath, D(GetPath))
    .def("PopulateFromFolder", &PackageMap::PopulateFromFolder,
         D(PopulateFromFolder))
    .def("PopulateFromEnvironment", &PackageMap::PopulateFromEnvironment,
         D(PopulateFromEnvironment))
    .def("PopulateUpstreamToDrake", &PackageMap::PopulateUpstreamToDrake,
         D(PopulateUpstreamToDrake));
}

}  // namespace pydrake
}  // namespace drake
