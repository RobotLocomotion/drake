#include "pybind11/pybind11.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/parsers/package_map.h"

#define D(...) DOC(drake, multibody, parsing, __VA_ARGS__)

namespace drake {
namespace pydrake {

PYBIND11_MODULE(parsers, m) {
  using drake::parsers::PackageMap;

  m.doc() = "Tools for loading robots from various files";

  py::class_<PackageMap>(m, "PackageMap")
    .def(py::init<>(), D(PackageMap, PackageMap))
    .def("Add", &PackageMap::Add, D(PackageMap, Add))
    .def("Contains", &PackageMap::Contains, D(PackageMap, Contains))
    .def("size", &PackageMap::size, D(PackageMap, size))
    .def("GetPath", &PackageMap::GetPath, D(PackageMap, GetPath))
    .def("PopulateFromFolder", &PackageMap::PopulateFromFolder,
         D(PackageMap, PopulateFromFolder))
    .def("PopulateFromEnvironment", &PackageMap::PopulateFromEnvironment,
         D(PackageMap, PopulateFromEnvironment))
    .def("PopulateUpstreamToDrake", &PackageMap::PopulateUpstreamToDrake,
         D(PackageMap, PopulateUpstreamToDrake));
}

}  // namespace pydrake
}  // namespace drake
