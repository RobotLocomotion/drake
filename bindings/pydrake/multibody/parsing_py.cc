#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/multibody/parsing/package_map.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/parsing/process_model_directives.h"
#include "drake/multibody/parsing/scoped_names.h"

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
        .def("Add", &Class::Add, py::arg("package_name"),
            py::arg("package_path"), cls_doc.Add.doc)
        .def("AddMap", &Class::AddMap, py::arg("other_map"), cls_doc.AddMap.doc)
        .def("Contains", &Class::Contains, py::arg("package_name"),
            cls_doc.Contains.doc)
        .def("Remove", &Class::Remove, py::arg("package_name"),
            cls_doc.Remove.doc)
        .def("size", &Class::size, cls_doc.size.doc)
        .def("GetPackageNames", &Class::GetPackageNames,
            cls_doc.GetPackageNames.doc)
        .def("GetPath", &Class::GetPath, py::arg("package_name"),
            cls_doc.GetPath.doc)
        .def("AddPackageXml", &Class::AddPackageXml, py::arg("filename"),
            cls_doc.AddPackageXml.doc)
        .def("PopulateFromFolder", &Class::PopulateFromFolder, py::arg("path"),
            cls_doc.PopulateFromFolder.doc)
        .def("PopulateFromEnvironment", &Class::PopulateFromEnvironment,
            py::arg("environment_variable"),
            cls_doc.PopulateFromEnvironment.doc)
        .def("PopulateUpstreamToDrake", &Class::PopulateUpstreamToDrake,
            py::arg("model_file"), cls_doc.PopulateUpstreamToDrake.doc)
        .def_static("MakeEmpty", &Class::MakeEmpty, cls_doc.MakeEmpty.doc);
  }

  // Parser
  {
    using Class = Parser;
    constexpr auto& cls_doc = doc.Parser;
    py::class_<Class>(m, "Parser", cls_doc.doc)
        .def(py::init<MultibodyPlant<double>*, SceneGraph<double>*>(),
            py::arg("plant"), py::arg("scene_graph") = nullptr,
            cls_doc.ctor.doc)
        .def("package_map", &Class::package_map, py_rvp::reference_internal,
            cls_doc.package_map.doc)
        .def("AddAllModelsFromFile", &Class::AddAllModelsFromFile,
            py::arg("file_name"), cls_doc.AddAllModelsFromFile.doc)
        .def("AddModelFromFile", &Class::AddModelFromFile, py::arg("file_name"),
            py::arg("model_name") = "", cls_doc.AddModelFromFile.doc)
        .def("AddModelFromString", &Class::AddModelFromString,
            py::arg("file_contents"), py::arg("file_type"),
            py::arg("model_name") = "", cls_doc.AddModelFromString.doc);
  }

  // Model Directives
  {
    using Class = parsing::ModelDirectives;
    py::class_<Class>(m, "ModelDirectives", doc.parsing.ModelDirectives.doc);
  }

  m.def("LoadModelDirectives", &parsing::LoadModelDirectives,
      py::arg("filename"), doc.parsing.LoadModelDirectives.doc);

  // ModelInstanceInfo
  {
    using Class = parsing::ModelInstanceInfo;
    constexpr auto& cls_doc = doc.parsing.ModelInstanceInfo;
    py::class_<Class>(m, "ModelInstanceInfo", cls_doc.doc)
        .def_readonly("model_name", &Class::model_name, cls_doc.model_name.doc)
        .def_readonly("model_path", &Class::model_path, cls_doc.model_path.doc)
        .def_readonly("parent_frame_name", &Class::parent_frame_name,
            cls_doc.parent_frame_name.doc)
        .def_readonly("child_frame_name", &Class::child_frame_name,
            cls_doc.child_frame_name.doc)
        .def_readonly("X_PC", &Class::X_PC, cls_doc.X_PC.doc)
        .def_readonly("model_instance", &Class::model_instance,
            cls_doc.model_instance.doc);
  }

  // Individual directives are not bound here (they are generally loaded from
  // yaml rather than constructed explicitly), but some downstream users use
  // this type as a return value which requires an explicit binding.
  {
    using Class = drake::multibody::parsing::AddFrame;
    constexpr auto& cls_doc = doc.parsing.AddFrame;
    py::class_<Class>(m, "AddFrame")
        .def_readonly("name", &Class::name, cls_doc.name.doc)
        .def_readonly("X_PF", &Class::X_PF, cls_doc.X_PF.doc);
  }

  m.def(
      "ProcessModelDirectives",
      [](const parsing::ModelDirectives& directives,
          MultibodyPlant<double>* plant, Parser* parser = nullptr) {
        std::vector<parsing::ModelInstanceInfo> added_models;
        parsing::ProcessModelDirectives(
            directives, plant, &added_models, parser);
        return added_models;
      },
      py::arg("directives"), py::arg("plant"), py::arg("parser"),
      doc.parsing.ProcessModelDirectives.doc);

  m.def("GetScopedFrameByName", &parsing::GetScopedFrameByName,
      py::arg("plant"), py::arg("full_name"),
      py::return_value_policy::reference,
      py::keep_alive<0, 1>(),  // `return` keeps `plant` alive.
      doc.parsing.GetScopedFrameByName.doc);

  m.def("GetScopedFrameName", &parsing::GetScopedFrameName, py::arg("plant"),
      py::arg("frame"), doc.parsing.GetScopedFrameName.doc);
}

}  // namespace pydrake
}  // namespace drake
