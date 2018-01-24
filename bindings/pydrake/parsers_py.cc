#include <pybind11/pybind11.h>

#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/parsers/package_map.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(parsers, m) {
  using drake::parsers::PackageMap;

  m.doc() = "Tools for loading robots from various files";

  py::class_<PackageMap>(m, "PackageMap")
    .def(py::init<>(), py::return_value_policy::reference)
    .def("Add", &PackageMap::Add)
    .def("Contains", &PackageMap::Contains)
    .def("size", &PackageMap::size)
    .def("GetPath", &PackageMap::GetPath)
    .def("PopulateFromFolder", &PackageMap::PopulateFromFolder)
    .def("PopulateFromEnvironment", &PackageMap::PopulateFromEnvironment)
    .def("PopulateUpstreamToDrake", &PackageMap::PopulateUpstreamToDrake);
}

}  // namespace pydrake
}  // namespace drake
