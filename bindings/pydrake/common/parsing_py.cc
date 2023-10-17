#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/common/parsing/package_map.h"

namespace drake {
namespace pydrake {
PYBIND11_MODULE(parsing, m) {
  m.doc() = "Common parsing utilities.";

  constexpr auto& doc = pydrake_doc.drake;

  // PackageMap
  {
    using Class = PackageMap;
    constexpr auto& cls_doc = doc.PackageMap;
    py::class_<Class> cls(m, "PackageMap", cls_doc.doc);
    {
      using Nested = PackageMap::RemoteParams;
      constexpr auto& nested_doc = cls_doc.RemoteParams;
      py::class_<Nested> nested(cls, "RemoteParams", nested_doc.doc);
      nested.def(ParamInit<Nested>());
      nested.def("ToJson", &Nested::ToJson, nested_doc.ToJson.doc);
      DefAttributesUsingSerialize(&nested, nested_doc);
      DefReprUsingSerialize(&nested);
      DefCopyAndDeepCopy(&nested);
    }
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc)
        .def(py::init<const Class&>(), py::arg("other"), "Copy constructor")
        .def("Add", &Class::Add, py::arg("package_name"),
            py::arg("package_path"), cls_doc.Add.doc)
        .def("AddMap", &Class::AddMap, py::arg("other_map"), cls_doc.AddMap.doc)
        .def("AddPackageXml", &Class::AddPackageXml, py::arg("filename"),
            cls_doc.AddPackageXml.doc)
        .def("AddRemote", &Class::AddRemote, py::arg("package_name"),
            py::arg("params"))
        .def("Contains", &Class::Contains, py::arg("package_name"),
            cls_doc.Contains.doc)
        .def("Remove", &Class::Remove, py::arg("package_name"),
            cls_doc.Remove.doc)
        .def("size", &Class::size, cls_doc.size.doc)
        .def("GetPackageNames", &Class::GetPackageNames,
            cls_doc.GetPackageNames.doc)
        .def(
            "GetPath",
            [](const PackageMap& self, const std::string& package_name) {
              // Python does not support output arguments, so we cannot bind the
              // deprecated_message here.
              return self.GetPath(package_name);
            },
            py::arg("package_name"), cls_doc.GetPath.doc)
        .def("PopulateFromFolder", &Class::PopulateFromFolder, py::arg("path"),
            cls_doc.PopulateFromFolder.doc)
        .def("PopulateFromEnvironment", &Class::PopulateFromEnvironment,
            py::arg("environment_variable"),
            cls_doc.PopulateFromEnvironment.doc)
        .def("PopulateFromRosPackagePath", &Class::PopulateFromRosPackagePath,
            cls_doc.PopulateFromRosPackagePath.doc)
        .def_static("MakeEmpty", &Class::MakeEmpty, cls_doc.MakeEmpty.doc);
    DefCopyAndDeepCopy(&cls);
  }
}
}  // namespace pydrake
}  // namespace drake
