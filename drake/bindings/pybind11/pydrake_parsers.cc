#include <pybind11/pybind11.h>

#include "drake/multibody/parsers/package_map.h"

namespace py = pybind11;

PYBIND11_PLUGIN(parsers) {
  using drake::parsers::PackageMap;

  py::module m("parsers", "Tools for loading robots from various files");

  py::class_<PackageMap>(m, "PackageMap")
    .def(py::init<>(), py::return_value_policy::reference)
    .def("Add", &PackageMap::Add)
    .def("Contains", &PackageMap::Contains)
    .def("size", &PackageMap::size)
    .def("GetPath", &PackageMap::GetPath)
    .def("PopulateFromFolder", &PackageMap::PopulateFromFolder)
    .def("PopulateFromEnvironment", &PackageMap::PopulateFromEnvironment)
    .def("PopulateUpstreamToDrake", &PackageMap::PopulateUpstreamToDrake);

  return m.ptr();
}
