#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/parsing/parser.h"

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using std::string;

namespace drake {
namespace pydrake {

PYBIND11_MODULE(parsing, m) {
  PYDRAKE_PREVENT_PYTHON3_MODULE_REIMPORT(m);
  m.doc() = "SDF and URDF parsing for MultibodyPlant and SceneGraph.";

  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::multibody;
  constexpr auto& doc = pydrake_doc.drake.multibody;

  // PackageMap
  {
    using Class = PackageMap;
    constexpr auto& cls_doc = doc.PackageMap;
    py::class_<Class>(m, "PackageMap", cls_doc.doc)
        .def(py::init<>(), cls_doc.ctor.doc)
        .def("Add", &Class::Add, cls_doc.Add.doc)
        .def("Contains", &Class::Contains, cls_doc.Contains.doc)
        .def("size", &Class::size, cls_doc.size.doc)
        .def("GetPath", &Class::GetPath, cls_doc.GetPath.doc)
        .def("PopulateFromFolder", &Class::PopulateFromFolder,
            cls_doc.PopulateFromFolder.doc)
        .def("PopulateFromEnvironment", &Class::PopulateFromEnvironment,
            cls_doc.PopulateFromEnvironment.doc)
        .def("PopulateUpstreamToDrake", &Class::PopulateUpstreamToDrake,
            cls_doc.PopulateUpstreamToDrake.doc);
  }

  // Parser
  {
    using Class = Parser;
    constexpr auto& cls_doc = doc.Parser;
    py::class_<Class>(m, "Parser", cls_doc.doc)
        .def(py::init<MultibodyPlant<double>*, SceneGraph<double>*>(),
            py::arg("plant"), py::arg("scene_graph") = nullptr,
            cls_doc.ctor.doc)
        .def("package_map", &Class::package_map, py_reference_internal,
            cls_doc.package_map.doc)
        .def("AddAllModelsFromFile", &Class::AddAllModelsFromFile,
            py::arg("file_name"), cls_doc.AddAllModelsFromFile.doc)
        .def("AddModelFromFile", &Class::AddModelFromFile, py::arg("file_name"),
            py::arg("model_name") = "", cls_doc.AddModelFromFile.doc);
  }
}

}  // namespace pydrake
}  // namespace drake
