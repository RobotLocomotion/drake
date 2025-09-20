#include <string>
#include <vector>

#include "drake/bindings/generated_docstrings/multibody_parsing.h"
#include "drake/bindings/pydrake/common/default_scalars_pybind.h"
#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/common/sorted_pair_pybind.h"
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
  constexpr auto& doc = pydrake_doc_multibody_parsing.drake.multibody;

  py::module::import("pydrake.common.schema");
  py::module::import("pydrake.geometry");
  py::module::import("pydrake.multibody.tree");

  // CollisionFilterGroups
  {
    using Class = CollisionFilterGroups;
    constexpr auto& cls_doc = doc.CollisionFilterGroups;
    auto cls = py::class_<Class>(m, "CollisionFilterGroups", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc)
        .def("AddGroup", &Class::AddGroup, py::arg("name"), py::arg("members"),
            cls_doc.AddGroup.doc)
        .def("AddExclusionPair", &Class::AddExclusionPair, py::arg("pair"),
            cls_doc.AddExclusionPair.doc)
        .def("empty", &Class::empty, cls_doc.empty.doc)
        .def("groups", &Class::groups, cls_doc.groups.doc)
        .def("exclusion_pairs", &Class::exclusion_pairs,
            cls_doc.exclusion_pairs.doc)
        .def("__str__", &Class::to_string, cls_doc.to_string.doc);
  }

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
        .def("Add",
            py::overload_cast<const std::string&, const std::filesystem::path&>(
                &Class::Add),
            py::arg("package_name"), py::arg("package_path"), cls_doc.Add.doc)
        .def("AddMap", &Class::AddMap, py::arg("other_map"), cls_doc.AddMap.doc)
        .def("AddPackageXml",
            py::overload_cast<const std::filesystem::path&>(
                &Class::AddPackageXml),
            py::arg("filename"), cls_doc.AddPackageXml.doc)
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
        .def("ResolveUrl", &Class::ResolveUrl, py::arg("url"),
            cls_doc.ResolveUrl.doc)
        .def("PopulateFromFolder",
            py::overload_cast<const std::filesystem::path&>(
                &Class::PopulateFromFolder),
            py::arg("path"), cls_doc.PopulateFromFolder.doc)
        .def("PopulateFromEnvironment", &Class::PopulateFromEnvironment,
            py::arg("environment_variable"),
            cls_doc.PopulateFromEnvironment.doc)
        .def("PopulateFromRosPackagePath", &Class::PopulateFromRosPackagePath,
            cls_doc.PopulateFromRosPackagePath.doc)
        .def_static("MakeEmpty", &Class::MakeEmpty, cls_doc.MakeEmpty.doc);
    DefCopyAndDeepCopy(&cls);
  }

  // Parser
  {
    using Class = Parser;
    constexpr auto& cls_doc = doc.Parser;
    auto cls = py::class_<Class>(m, "Parser", cls_doc.doc);
    cls  // BR
        .def(py::init<MultibodyPlant<double>*, SceneGraph<double>*,
                 std::string_view>(),
            py::arg("plant"), py::arg("scene_graph") = nullptr,
            py::arg("model_name_prefix") = "",
            cls_doc.ctor.doc_3args_plant_scene_graph_model_name_prefix)
        .def(py::init<MultibodyPlant<double>*, std::string_view>(),
            py::arg("plant"), py::arg("model_name_prefix"),
            cls_doc.ctor.doc_2args_plant_model_name_prefix)
        .def(py::init<systems::DiagramBuilder<double>*, MultibodyPlant<double>*,
                 geometry::SceneGraph<double>*, std::string_view>(),
            py::arg("builder"), py::arg("plant") = nullptr,
            py::arg("scene_graph") = nullptr, py::arg("model_name_prefix") = "",
            cls_doc.ctor.doc_4args_builder_plant_scene_graph_model_name_prefix)
        .def("builder", &Class::builder, py_rvp::reference, cls_doc.builder.doc)
        .def("plant", &Class::plant, py_rvp::reference, cls_doc.plant.doc)
        .def("scene_graph", &Class::scene_graph, py_rvp::reference,
            cls_doc.scene_graph.doc)
        .def("package_map", &Class::package_map, py_rvp::reference_internal,
            cls_doc.package_map.doc)
        .def("AddModels", &Class::AddModels, py::arg("file_name"),
            cls_doc.AddModels.doc)
        .def("AddModelsFromUrl", &Class::AddModelsFromUrl, py::arg("url"),
            cls_doc.AddModelsFromUrl.doc)
        .def("AddModelsFromString", &Class::AddModelsFromString,
            py::arg("file_contents"), py::arg("file_type"),
            cls_doc.AddModelsFromString.doc)
        // Overload AddModels(url=) for some sugar.
        .def("AddModels", &Class::AddModelsFromUrl, py::kw_only(),
            py::arg("url"), cls_doc.AddModelsFromUrl.doc)
        // Overload AddModels(file_contents=, file_type=) for some sugar.
        .def("AddModels", &Class::AddModelsFromString, py::kw_only(),
            py::arg("file_contents"), py::arg("file_type"),
            cls_doc.AddModelsFromString.doc)
        .def("SetStrictParsing", &Class::SetStrictParsing,
            cls_doc.SetStrictParsing.doc)
        .def("SetAutoRenaming", &Class::SetAutoRenaming, py::arg("value"),
            cls_doc.SetAutoRenaming.doc)
        .def("GetAutoRenaming", &Class::GetAutoRenaming,
            cls_doc.GetAutoRenaming.doc)
        .def("GetCollisionFilterGroups", &Class::GetCollisionFilterGroups,
            cls_doc.GetCollisionFilterGroups.doc);
  }

  // Model Directives
  {
    using Class = parsing::AddWeld;
    constexpr auto& cls_doc = doc.parsing.AddWeld;
    py::class_<Class> cls(m, "AddWeld", cls_doc.doc);
    cls.def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = parsing::AddModel;
    constexpr auto& cls_doc = doc.parsing.AddModel;
    py::class_<Class> cls(m, "AddModel", cls_doc.doc);
    cls.def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = parsing::AddModelInstance;
    constexpr auto& cls_doc = doc.parsing.AddModelInstance;
    py::class_<Class> cls(m, "AddModelInstance", cls_doc.doc);
    cls.def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = parsing::AddFrame;
    constexpr auto& cls_doc = doc.parsing.AddFrame;
    py::class_<Class> cls(m, "AddFrame", cls_doc.doc);
    cls.def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = parsing::AddCollisionFilterGroup;
    constexpr auto& cls_doc = doc.parsing.AddCollisionFilterGroup;
    py::class_<Class> cls(m, "AddCollisionFilterGroup", cls_doc.doc);
    cls.def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = parsing::AddDirectives;
    constexpr auto& cls_doc = doc.parsing.AddDirectives;
    py::class_<Class> cls(m, "AddDirectives", cls_doc.doc);
    cls.def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = parsing::ModelDirective;
    constexpr auto& cls_doc = doc.parsing.ModelDirective;
    py::class_<Class> cls(m, "ModelDirective", cls_doc.doc);
    cls.def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = parsing::ModelDirectives;
    constexpr auto& cls_doc = doc.parsing.ModelDirectives;
    py::class_<Class> cls(m, "ModelDirectives", cls_doc.doc);
    cls.def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  m.def("LoadModelDirectives", &parsing::LoadModelDirectives,
      py::arg("filename"), doc.parsing.LoadModelDirectives.doc);

  m.def("LoadModelDirectivesFromString",
      &parsing::LoadModelDirectivesFromString, py::arg("model_directives"),
      doc.parsing.LoadModelDirectivesFromString.doc);

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

  m.def(
      "FlattenModelDirectives",
      [](const parsing::ModelDirectives& directives,
          const multibody::PackageMap& package_map) {
        parsing::ModelDirectives out;
        parsing::FlattenModelDirectives(directives, package_map, &out);
        return out;
      },
      py::arg("directives"), py::arg("package_map"),
      doc.parsing.FlattenModelDirectives.doc);

  m.def("ProcessModelDirectives",
      py::overload_cast<const parsing::ModelDirectives&, Parser*>(
          &parsing::ProcessModelDirectives),
      py::arg("directives"), py::arg("parser"),
      doc.parsing.ProcessModelDirectives.doc_2args);

  m.def(
      "ProcessModelDirectives",
      [](const parsing::ModelDirectives& directives,
          MultibodyPlant<double>* plant, Parser* parser) {
        std::vector<parsing::ModelInstanceInfo> added_models;
        parsing::ProcessModelDirectives(
            directives, plant, &added_models, parser);
        return added_models;
      },
      py::arg("directives"), py::arg("plant"), py::arg("parser") = nullptr,
      doc.parsing.ProcessModelDirectives.doc_4args);

  type_visit(
      [&m]<typename T>(T) {
        m.def("GetScopedFrameByName",
            overload_cast_explicit<const Frame<T>&, const MultibodyPlant<T>&,
                std::string_view>(&parsing::GetScopedFrameByName),
            py::arg("plant"), py::arg("full_name"),
            py::return_value_policy::reference,
            py::keep_alive<0, 1>(),  // `return` keeps `plant` alive.
            doc.parsing.GetScopedFrameByName.doc);
      },
      CommonScalarPack{});

  type_visit(
      [&m]<typename T>(T) {
        m.def("GetScopedFrameByNameMaybe",
            overload_cast_explicit<const Frame<T>*, const MultibodyPlant<T>&,
                std::string_view>(&parsing::GetScopedFrameByNameMaybe),
            py::arg("plant"), py::arg("full_name"),
            py::return_value_policy::reference,
            py::keep_alive<0, 1>(),  // `return` keeps `plant` alive.
            doc.parsing.GetScopedFrameByNameMaybe.doc);
      },
      CommonScalarPack{});
}

}  // namespace pydrake
}  // namespace drake
